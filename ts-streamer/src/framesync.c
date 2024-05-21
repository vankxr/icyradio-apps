#include "framesync.h"

typedef struct framesync_thread_data_t framesync_thread_data_t;

struct framesync_thread_data_t
{
    framesync_t *fs;
    size_t idx;
};

static void *framesync_detector_thread_entry(void *p)
{
    framesync_thread_data_t *data = (framesync_thread_data_t *)p;

    framesync_t *fs = data->fs;
    size_t idx = data->idx;
    qdetector_cccf detector = fs->detector[idx];

    framesync_detector_buffer_t *buf = NULL;

    while(1)
    {
        pthread_mutex_lock(&fs->detector_buf_mutex);

        while(1)
        {
            if(buf) // Ensure the main thread released the buffer
            {
                while(buf->sync_buf)
                {
                    pthread_cond_wait(&fs->detector_buf_cond, &fs->detector_buf_mutex);

                    if(!fs->detector_buf)
                        break;
                }

                buf = NULL;

                if(!fs->detector_buf)
                    break;
            }

            while(!fs->detector_buf_len)
            {
                pthread_cond_wait(&fs->detector_buf_cond, &fs->detector_buf_mutex);

                if(!fs->detector_buf)
                    break;
            }

            size_t i = (fs->detector_n_threads + fs->detector_buf_head - 1) % fs->detector_n_threads;

            for(size_t n = 0; n < fs->detector_n_threads; n++)
            {
                if(fs->detector_buf[i].state != FRAMESYNC_BUFFER_READY || !fs->detector_buf[i].buf || !fs->detector_buf[i].len)
                {
                    i = (i + 1) % fs->detector_n_threads;

                    continue;
                }

                buf = &fs->detector_buf[i];

                break;

                i = (i + 1) % fs->detector_n_threads;
            }

            if(buf)
                break;

            pthread_cond_wait(&fs->detector_buf_cond, &fs->detector_buf_mutex);

            if(!fs->detector_buf)
                break;
        }

        if(!buf)
        {
            pthread_mutex_unlock(&fs->detector_buf_mutex);

            break;
        }

        buf->state = FRAMESYNC_BUFFER_PROCESSING;

        pthread_mutex_unlock(&fs->detector_buf_mutex);

        pthread_mutex_lock(&fs->detector_mutex[idx]);

        float complex *sync_buf = NULL;
        float tau_hat = 0.0f;
        float gamma_hat = 0.0f;
        float dphi_hat = 0.0f;
        float phi_hat = 0.0f;
        size_t sync_len = 0;
        size_t sync_pos = 0;

        for(size_t i = 0; i < buf->len; i++)
        {
            sync_buf = qdetector_cccf_execute(detector, buf->buf[i]);

            if(sync_buf == NULL)
                continue;

            tau_hat = qdetector_cccf_get_tau(detector);
            gamma_hat = qdetector_cccf_get_gamma(detector);
            dphi_hat = qdetector_cccf_get_dphi(detector);
            phi_hat = qdetector_cccf_get_phi(detector);
            sync_len = qdetector_cccf_get_buf_len(detector);
            sync_pos = i + 1;

            break;
        }

        pthread_mutex_unlock(&fs->detector_mutex[idx]);

        pthread_mutex_lock(&fs->detector_buf_mutex);

        if(buf->state == FRAMESYNC_BUFFER_PROCESSING)
        {
            buf->state = FRAMESYNC_BUFFER_PROCESSED;

            if(sync_buf)
            {
                buf->sync_buf = sync_buf;
                buf->sync_len = sync_len;
                buf->sync_pos = sync_pos;
                buf->stats.tau_hat = tau_hat;
                buf->stats.gamma_hat = gamma_hat;
                buf->stats.dphi_hat = dphi_hat;
                buf->stats.phi_hat = phi_hat;
            }
        }

        pthread_mutex_unlock(&fs->detector_buf_mutex);
    }

    free(p);

    return NULL;
}

static uint8_t framesync_step(framesync_t *fs, float complex sym, float complex *out)
{
    float complex v;

    nco_crcf_mix_down(fs->mixer, sym, &v);
    nco_crcf_step(fs->mixer);

    firpfb_crcf_push(fs->mf, v);
    firpfb_crcf_execute(fs->mf, fs->mf_pfb_index, &v);

#if FRAMESYNC_ENABLE_EQ
    eqlms_cccf_push(fs->eq, v);
#endif

    fs->mf_counter++;

    uint8_t sample_available = (fs->mf_counter >= 1) ? 1 : 0;

    if(sample_available)
    {
#if FRAMESYNC_ENABLE_EQ
        eqlms_cccf_execute(fs->eq, &v);
#endif

        *out = v;
        fs->mf_counter -= 2;
    }

    return sample_available;
}
static void framesync_decode_header(framesync_t *fs)
{
    if(fs->header_props.pilots)
        qpilotsync_execute(fs->header_pilotsync, fs->header_sym, fs->header_mod);
    else
        memmove(fs->header_mod, fs->header_sym, fs->header_sym_len * sizeof(float complex));

    if(fs->header_soft)
        fs->header_valid = qpacketmodem_decode_soft(fs->header_decoder, fs->header_mod, fs->header_dec);
    else
        fs->header_valid = qpacketmodem_decode(fs->header_decoder, fs->header_mod, fs->header_dec);

    if(!fs->header_valid)
        return;

    if(fs->header_props.pilots)
    {
        // float gamma_hat = qpilotsync_get_gain(fs->header_pilotsync);
        float dphi_hat = qpilotsync_get_dphi(fs->header_pilotsync);
        float phi_hat = qpilotsync_get_phi(fs->header_pilotsync);

        // nco_crcf_adjust_frequency(fs->mixer, dphi_hat);
        nco_crcf_adjust_phase(fs->mixer, phi_hat + dphi_hat * fs->header_sym_len);

        // firpfb_crcf_set_scale(fs->mf, 0.5f / gamma_hat);
    }

    uint8_t protocol = fs->header_dec[0];

    if(protocol != 0x5A)
    {
        fs->header_valid = 0;

        return;
    }

    size_t payload_dec_len = ((size_t)fs->header_dec[1] << 8) | (size_t)fs->header_dec[2];
    modulation_scheme mod = (modulation_scheme)fs->header_dec[3];
    crc_scheme crc = (crc_scheme)((fs->header_dec[4] >> 5) & 0x07);
    fec_scheme i_fec  = (fec_scheme)((fs->header_dec[4]) & 0x1F);
    fec_scheme o_fec  = (fec_scheme)((fs->header_dec[5]) & 0x1F);
    uint8_t pilots = !!(fs->header_dec[5] & 0x40);
    uint8_t is_last = !!(fs->header_dec[5] & 0x80);

    if(crc == LIQUID_CRC_UNKNOWN || crc >= LIQUID_CRC_NUM_SCHEMES)
    {
        fs->header_valid = 0;

        return;
    }

    if(i_fec == LIQUID_FEC_UNKNOWN || i_fec >= LIQUID_FEC_NUM_SCHEMES)
    {
        fs->header_valid = 0;

        return;
    }

    if(o_fec == LIQUID_FEC_UNKNOWN || o_fec >= LIQUID_FEC_NUM_SCHEMES)
    {
        fs->header_valid = 0;

        return;
    }

    if(mod == LIQUID_MODEM_UNKNOWN || mod >= LIQUID_MODEM_NUM_SCHEMES)
    {
        fs->header_valid = 0;

        return;
    }

    fs->payload_dec_len = payload_dec_len;
    fs->payload_props.mod = mod;
    fs->payload_props.crc = crc;
    fs->payload_props.i_fec = i_fec;
    fs->payload_props.o_fec = o_fec;
    fs->payload_props.pilots = pilots;
    fs->header_is_last = is_last;

    fs->payload_dec = (uint8_t *)realloc(fs->payload_dec, payload_dec_len * sizeof(uint8_t));

    if(!fs->payload_dec)
    {
        fs->header_valid = 0;

        return;
    }

    qpacketmodem_configure(fs->payload_decoder, payload_dec_len, crc, i_fec, o_fec, mod);

    fs->payload_demod = modemcf_recreate(fs->payload_demod, mod);
    fs->payload_mod_len = qpacketmodem_get_frame_len(fs->payload_decoder);
    fs->payload_mod = (float complex *)realloc(fs->payload_mod, fs->payload_mod_len * sizeof(float complex));

    if(!fs->payload_mod)
    {
        fs->header_valid = 0;

        return;
    }

    fs->payload_pilotsync = qpilotsync_recreate(fs->payload_pilotsync, fs->payload_mod_len, 16);
    fs->payload_sym_len = pilots ? qpilotsync_get_frame_len(fs->payload_pilotsync) : fs->payload_mod_len;
    fs->payload_sym = (float complex *)realloc(fs->payload_sym, fs->payload_sym_len * sizeof(float complex));

    if(!fs->payload_sym)
    {
        fs->header_valid = 0;

        return;
    }
}

static void framesync_process_rxdelay(framesync_t *fs, float complex sym)
{
    float complex mf_out = 0.0f;

    if(!framesync_step(fs, sym, &mf_out))
        return;

#if FRAMESYNC_ENABLE_EQ
    size_t delay = 2 * fs->mf_m + 3;
#else
    size_t delay = 2 * fs->mf_m;
#endif

    fs->preamble_counter++;

    if(fs->preamble_counter == delay)
    {
        fs->preamble_counter = 0;
        fs->state = FRAMESYNC_STATE_RXPREAMBLE;
    }
}
static void framesync_process_rxpreamble(framesync_t *fs, float complex sym)
{
    float complex mf_out = 0.0f;

    if(!framesync_step(fs, sym, &mf_out))
        return;

    fs->preamble_rx[fs->preamble_counter] = mf_out;

#if FRAMESYNC_ENABLE_EQ
    eqlms_cccf_step(fs->eq, fs->preamble_pn[fs->preamble_counter], mf_out);
#endif

    fs->preamble_counter++;

    if(fs->preamble_counter == 64)
    {
        fs->preamble_counter = 0;
        fs->state = FRAMESYNC_STATE_RXHEADER;
    }
}
static void framesync_process_rxheader(framesync_t *fs, float complex sym)
{
    float complex mf_out = 0.0f;

    if(!framesync_step(fs, sym, &mf_out))
        return;

    if(!fs->header_props.pilots)
    {
        unsigned int _sym;

        modemcf_demodulate(fs->header_demod, mf_out, &_sym);

        float phase_error = modemcf_get_demodulator_phase_error(fs->header_demod);

        nco_crcf_pll_step(fs->mixer, phase_error);
    }

    fs->header_sym[fs->symbol_counter++] = mf_out;

    if(fs->symbol_counter == fs->header_sym_len)
    {
        framesync_decode_header(fs);

        if(fs->header_valid)
        {
            fs->symbol_counter = 0;
            fs->state = FRAMESYNC_STATE_RXPAYLOAD;
        }
        else
        {
            framesync_reset(fs);

            if(fs->cb)
            {
                fs->cb_stats.evm = fs->header_props.pilots ? qpilotsync_get_evm(fs->header_pilotsync) : 0.0f;
                fs->cb_stats.cfo = nco_crcf_get_frequency(fs->mixer);

                fs->cb(fs->cb_ptr, &fs->cb_stats, fs->header_dec + 6, fs->header_valid, NULL, 0, 0, fs->header_is_last);
            }
        }
    }
}
static void framesync_process_rxpayload(framesync_t *fs, float complex sym)
{
    float complex mf_out = 0.0f;

    if(!framesync_step(fs, sym, &mf_out))
        return;

    if(!fs->payload_props.pilots)
    {
        unsigned int _sym;

        modemcf_demodulate(fs->payload_demod, mf_out, &_sym);

        float phase_error = modemcf_get_demodulator_phase_error(fs->payload_demod);
        float evm = modemcf_get_demodulator_evm(fs->payload_demod);

        nco_crcf_pll_step(fs->mixer, phase_error);

        if(!fs->symbol_counter)
            fs->cb_stats.evm = evm * evm;
        else
            fs->cb_stats.evm += evm * evm;
    }

    fs->payload_sym[fs->symbol_counter++] = mf_out;

    if(fs->symbol_counter == fs->payload_sym_len)
    {
        if(fs->payload_props.pilots)
            qpilotsync_execute(fs->payload_pilotsync, fs->payload_sym, fs->payload_mod);
        else
            memmove(fs->payload_mod, fs->payload_sym, fs->payload_sym_len * sizeof(float complex));

        if(fs->payload_soft)
            fs->payload_valid = qpacketmodem_decode_soft(fs->payload_decoder, fs->payload_mod, fs->payload_dec);
        else
            fs->payload_valid = qpacketmodem_decode(fs->payload_decoder, fs->payload_mod, fs->payload_dec);

        if(fs->payload_props.pilots)
        {
            float gamma_hat = qpilotsync_get_gain(fs->payload_pilotsync);
            float dphi_hat = qpilotsync_get_dphi(fs->payload_pilotsync);
            float phi_hat = qpilotsync_get_phi(fs->payload_pilotsync);

            // nco_crcf_adjust_frequency(fs->mixer, dphi_hat);
            nco_crcf_adjust_phase(fs->mixer, phi_hat + dphi_hat * fs->payload_sym_len);

            // firpfb_crcf_set_scale(fs->mf, 0.5f / gamma_hat);
        }

        uint8_t force_relock = fs->header_is_last || !fs->payload_valid;

        if(fs->cb)
        {
            fs->cb_stats.evm = fs->payload_props.pilots ? qpilotsync_get_evm(fs->payload_pilotsync) : 10.0f * log10f(fs->cb_stats.evm / (float)fs->payload_sym_len);
            fs->cb_stats.cfo = nco_crcf_get_frequency(fs->mixer);

            force_relock |= !!fs->cb(fs->cb_ptr, &fs->cb_stats, fs->header_dec + 6, fs->header_valid, fs->payload_dec, fs->payload_dec_len, fs->payload_valid, fs->header_is_last);
        }

        if(force_relock)
        {
            framesync_reset(fs);
        }
        else
        {
            fs->preamble_counter = 0;
            fs->symbol_counter = 0;
            fs->state = FRAMESYNC_STATE_RXPREAMBLE;
        }
    }
}

static size_t framesync_process_samples_int(framesync_t *fs, float complex *buffer, size_t buffer_len)
{
    if(fs->state == FRAMESYNC_STATE_DETECTFRAME)
    {
        if(fs->detector_buf_len >= fs->detector_n_threads)
        {
            sched_yield();

            return 0;
        }

        framesync_detector_buffer_t *buf = &fs->detector_buf[fs->detector_buf_head];

        pthread_mutex_lock(&fs->detector_buf_mutex);

        if(buf->state != FRAMESYNC_BUFFER_CONSUMED)
        {
            pthread_mutex_unlock(&fs->detector_buf_mutex);

            return 0;
        }

        pthread_mutex_unlock(&fs->detector_buf_mutex);

        if(buf->len != buffer_len)
            buf->buf = (float complex *)realloc(buf->buf, buffer_len * sizeof(float complex));

        if(!buf->buf)
            return 0;

        buf->len = buffer_len;
        memmove(buf->buf, buffer, buffer_len * sizeof(float complex));

        pthread_mutex_lock(&fs->detector_buf_mutex);

        buf->state = FRAMESYNC_BUFFER_READY;

        fs->detector_buf_len++;
        fs->detector_buf_head = (fs->detector_buf_head + 1) % fs->detector_n_threads;
        pthread_cond_broadcast(&fs->detector_buf_cond);

        pthread_mutex_unlock(&fs->detector_buf_mutex);

        fprintf(stderr, "Loaded buffer %zu\n", (fs->detector_n_threads + fs->detector_buf_head - 1) % fs->detector_n_threads);

        return buffer_len;
    }

    for(size_t i = 0; i < buffer_len; i++)
    {
        switch(fs->state)
        {
            case FRAMESYNC_STATE_DETECTFRAME:
                return i;
            break;
            case FRAMESYNC_STATE_RXDELAY:
                framesync_process_rxdelay(fs, buffer[i]);
            break;
            case FRAMESYNC_STATE_RXPREAMBLE:
                framesync_process_rxpreamble(fs, buffer[i]);
            break;
            case FRAMESYNC_STATE_RXHEADER:
                framesync_process_rxheader(fs, buffer[i]);
            break;
            case FRAMESYNC_STATE_RXPAYLOAD:
                framesync_process_rxpayload(fs, buffer[i]);
            break;
        }
    }

    return buffer_len;
}
static void framesync_process_detector_out(framesync_t *fs)
{
    uint8_t f = 1;

    while(fs->detector_buf_len)
    {
        framesync_detector_buffer_t *buf = &fs->detector_buf[fs->detector_buf_tail];

        pthread_mutex_lock(&fs->detector_buf_mutex);

        if(fs->state == FRAMESYNC_STATE_DETECTFRAME && buf->state != FRAMESYNC_BUFFER_PROCESSED)
        {
            pthread_mutex_unlock(&fs->detector_buf_mutex);

            return; // We must process the buffers in order
        }

        fs->detector_buf_len--;
        fs->detector_buf_tail = (fs->detector_buf_tail + 1) % fs->detector_n_threads;

        buf->state = FRAMESYNC_BUFFER_CONSUMED;

        if(fs->state == FRAMESYNC_STATE_DETECTFRAME)
        {
            if(!buf->sync_buf) // Nothing found in this buffer
            {
                pthread_mutex_unlock(&fs->detector_buf_mutex);

                continue;
            }
        }
        else
        {
            buf->sync_buf = NULL;

            if(buf->state != FRAMESYNC_BUFFER_PROCESSING)
                pthread_cond_broadcast(&fs->detector_buf_cond);
        }

        pthread_mutex_unlock(&fs->detector_buf_mutex);

        // We can work with buf directly here without the mutex lock since we guarantee the state is DONE, hence, no one else will touch it
        if(fs->state == FRAMESYNC_STATE_DETECTFRAME)
        {
            fprintf(stderr, "Frame found in buffer %zu %hhu\n", (fs->detector_n_threads + fs->detector_buf_tail - 1) % fs->detector_n_threads, f); f = 0;

            if(buf->stats.tau_hat > 0)
            {
                fs->mf_pfb_index = (unsigned int)roundf(buf->stats.tau_hat * fs->mf_npfb) % fs->mf_npfb;
                fs->mf_counter = 0;
            }
            else
            {
                fs->mf_pfb_index = (unsigned int)roundf((1.0f + buf->stats.tau_hat) * fs->mf_npfb) % fs->mf_npfb;
                fs->mf_counter = 1;
            }

            firpfb_crcf_set_scale(fs->mf, 0.5f / buf->stats.gamma_hat);

            fs->cb_stats.rssi = 20.0f * log10f(buf->stats.gamma_hat);

            nco_crcf_set_frequency(fs->mixer, buf->stats.dphi_hat);
            nco_crcf_set_phase(fs->mixer, buf->stats.phi_hat);

            fs->state = FRAMESYNC_STATE_RXDELAY;

            fprintf(stderr, "Proc sync\n");
            framesync_process_samples_int(fs, buf->sync_buf, buf->sync_len); // Process the synced preamble

            pthread_mutex_lock(&fs->detector_buf_mutex);
            buf->sync_buf = NULL; // Signal the detector thread that it can release this buffer
            pthread_cond_broadcast(&fs->detector_buf_cond);
            pthread_mutex_unlock(&fs->detector_buf_mutex);

            if(buf->sync_pos < buf->len) // Process the remaining of this buffer
            {
                size_t _len = buf->len - buf->sync_pos;
                float complex *_buf = (float complex *)malloc(_len * sizeof(float complex));

                if(_buf)
                {
                    memmove(_buf, buf->buf + buf->sync_pos, _len * sizeof(float complex));

            fprintf(stderr, "Proc rem\n");
                    size_t proc = 0;

                    while(proc < _len)
                        proc += framesync_process_samples_int(fs, _buf + proc, _len- proc);

                    free(_buf);
                }
            }
        }
        else
        {
            fprintf(stderr, "Proc buffer %zu\n", (fs->detector_n_threads + fs->detector_buf_tail - 1) % fs->detector_n_threads);
            float complex *_buf = (float complex *)malloc(buf->len * sizeof(float complex));

            if(_buf)
            {
                memmove(_buf, buf->buf, buf->len * sizeof(float complex));

                size_t proc = 0;

                while(proc < buf->len)
                    proc += framesync_process_samples_int(fs, _buf + proc, buf->len - proc);

                free(_buf);
            }
        }
    }
}


framesync_t *framesync_create(framegenprops_t *h_props, size_t h_len, size_t n_det_threads)
{
    if(!h_props)
        return NULL;

    if(h_props->crc == LIQUID_CRC_UNKNOWN || h_props->crc >= LIQUID_CRC_NUM_SCHEMES)
        return NULL;

    if(h_props->i_fec == LIQUID_FEC_UNKNOWN || h_props->i_fec >= LIQUID_FEC_NUM_SCHEMES)
        return NULL;

    if(h_props->o_fec == LIQUID_FEC_UNKNOWN || h_props->o_fec >= LIQUID_FEC_NUM_SCHEMES)
        return NULL;

    if(h_props->mod == LIQUID_MODEM_UNKNOWN || h_props->mod >= LIQUID_MODEM_NUM_SCHEMES)
        return NULL;

    if(!n_det_threads)
        return NULL;

    framesync_t *fs = (framesync_t *)malloc(sizeof(framesync_t));

    if(!fs)
        return NULL;

    fs->cb = NULL;
    fs->cb_ptr = NULL;

    fs->preamble_pn = (float complex *)malloc(64 * sizeof(float complex));

    if(!fs->preamble_pn)
    {
        free(fs);

        return NULL;
    }

    msequence ms = msequence_create_default(7);

    for(size_t i = 0; i < 64; i++)
    {
        fs->preamble_pn[i] = (msequence_advance(ms) ? M_SQRT1_2 : -M_SQRT1_2);
        fs->preamble_pn[i] += (msequence_advance(ms) ? M_SQRT1_2 : -M_SQRT1_2) * _Complex_I;
    }

    msequence_destroy(ms);

    fs->preamble_rx = (float complex *)malloc(64 * sizeof(float complex));

    if(!fs->preamble_rx)
    {
        free(fs->preamble_pn);
        free(fs);

        return NULL;
    }

    fs->detector_n_threads = n_det_threads;
    fs->detector_thread = (pthread_t *)malloc(n_det_threads * sizeof(pthread_t));

    if(!fs->detector_thread)
    {
        free(fs->preamble_rx);
        free(fs->preamble_pn);
        free(fs);

        return NULL;
    }

    fs->detector_k = 2;
    fs->detector_m = 8;
    fs->detector_beta = 0.15f;
    fs->detector_mutex = (pthread_mutex_t *)malloc(n_det_threads * sizeof(pthread_mutex_t));

    if(!fs->detector_mutex)
    {
        free(fs->detector_thread);
        free(fs->preamble_rx);
        free(fs->preamble_pn);
        free(fs);

        return NULL;
    }

    fs->detector = (qdetector_cccf *)malloc(n_det_threads * sizeof(qdetector_cccf));

    if(!fs->detector)
    {
        free(fs->detector_mutex);
        free(fs->detector_thread);
        free(fs->preamble_rx);
        free(fs->preamble_pn);
        free(fs);

        return NULL;
    }

    pthread_mutex_init(&fs->detector_buf_mutex, NULL);
    pthread_cond_init(&fs->detector_buf_cond, NULL);

    fs->detector_buf = (framesync_detector_buffer_t *)malloc(n_det_threads * sizeof(framesync_detector_buffer_t));

    if(!fs->detector_buf)
    {
        free(fs->detector);
        free(fs->detector_mutex);
        free(fs->detector_thread);
        free(fs->preamble_rx);
        free(fs->preamble_pn);
        free(fs);

        return NULL;
    }

    fs->detector_buf_len = 0;
    fs->detector_buf_head = 0;
    fs->detector_buf_tail = 0;

    for(size_t i = 0; i < n_det_threads; i++)
    {
        pthread_mutex_init(&fs->detector_mutex[i], NULL);

        fs->detector[i] = qdetector_cccf_create_linear(fs->preamble_pn, 64, LIQUID_FIRFILT_RRC, fs->detector_k, fs->detector_m, fs->detector_beta);

        qdetector_cccf_set_threshold(fs->detector[i], 0.5f);

        fs->detector_buf[i].buf = NULL;
        fs->detector_buf[i].len = 0;
        fs->detector_buf[i].state = FRAMESYNC_BUFFER_CONSUMED;
        fs->detector_buf[i].sync_buf = NULL;
        fs->detector_buf[i].sync_len = 0;
        fs->detector_buf[i].sync_pos = 0;

        framesync_thread_data_t *data = (framesync_thread_data_t *)malloc(sizeof(framesync_thread_data_t));

        if(!data)
        {
            for(size_t j = 0; j < i; j++)
                qdetector_cccf_destroy(fs->detector[j]);

            free(fs->detector);
            free(fs->detector_mutex);
            free(fs->detector_thread);
            free(fs->preamble_rx);
            free(fs->preamble_pn);
            free(fs);

            return NULL;
        }

        data->fs = fs;
        data->idx = i;

        pthread_create(&fs->detector_thread[i], NULL, framesync_detector_thread_entry, data);
    }

    fs->mf_k = 2;
    fs->mf_m = 8;
    fs->mf_beta = 0.15f;
    fs->mf_npfb = 128;
    fs->mf = firpfb_crcf_create_rnyquist(LIQUID_FIRFILT_RRC, fs->mf_npfb, fs->mf_k, fs->mf_m, fs->mf_beta);

    fs->mixer = nco_crcf_create(LIQUID_NCO);
    nco_crcf_pll_set_bandwidth(fs->mixer, 1e-4f);

#if FRAMESYNC_ENABLE_EQ
    fs->eq = eqlms_cccf_create_lowpass(2 * fs->mf_k * 3 + 1, 0.4f);

    eqlms_cccf_set_bw(fs->eq, 0.05f);
#endif

    memmove(&fs->header_props, h_props, sizeof(framegenprops_t));

    fs->header_user_len = h_len;
    fs->header_dec_len = 6 + h_len;
    fs->header_dec = (uint8_t *)malloc(fs->header_dec_len);

    if(!fs->header_dec)
    {
        free(fs->detector_buf);
        free(fs->detector);
        free(fs->detector_mutex);
        free(fs->detector_thread);
        free(fs->preamble_rx);
        free(fs->preamble_pn);
        free(fs);

        return NULL;
    }

    fs->header_decoder = qpacketmodem_create();

    qpacketmodem_configure(fs->header_decoder, fs->header_dec_len, fs->header_props.crc, fs->header_props.i_fec, fs->header_props.o_fec, fs->header_props.mod);

    fs->header_demod = modemcf_create(fs->header_props.mod);
    fs->header_mod_len = qpacketmodem_get_frame_len(fs->header_decoder);
    fs->header_mod = (float complex *)malloc(fs->header_mod_len * sizeof(float complex));

    if(!fs->header_mod)
    {
        free(fs->header_dec);
        free(fs->detector_buf);
        free(fs->detector);
        free(fs->detector_mutex);
        free(fs->detector_thread);
        free(fs->preamble_rx);
        free(fs->preamble_pn);
        free(fs);

        return NULL;
    }

    fs->header_pilotsync = qpilotsync_create(fs->header_mod_len, 16);
    fs->header_sym_len = fs->header_props.pilots ? qpilotsync_get_frame_len(fs->header_pilotsync) : fs->header_mod_len;
    fs->header_sym = (float complex *)malloc(fs->header_sym_len * sizeof(float complex));

    if(!fs->header_sym)
    {
        free(fs->header_mod);
        free(fs->header_dec);
        free(fs->detector_buf);
        free(fs->detector);
        free(fs->detector_mutex);
        free(fs->detector_thread);
        free(fs->preamble_rx);
        free(fs->preamble_pn);
        free(fs);

        return NULL;
    }

    fs->payload_dec_len = 0;
    fs->payload_dec = NULL;
    fs->payload_decoder = qpacketmodem_create();
    fs->payload_demod = modemcf_create(LIQUID_MODEM_QPSK); // dummy modem, will be reconfigured later
    fs->payload_mod_len = 0;
    fs->payload_mod = NULL;
    fs->payload_pilotsync = qpilotsync_create(128, 64);
    fs->payload_sym_len = 0;
    fs->payload_sym = NULL;

    framesync_reset(fs);

    return fs;
}
void framesync_delete(framesync_t *fs)
{
    if(!fs)
        return;

    qpilotsync_destroy(fs->payload_pilotsync);
    modemcf_destroy(fs->payload_demod);
    qpacketmodem_destroy(fs->payload_decoder);
    qpilotsync_destroy(fs->header_pilotsync);
    modemcf_destroy(fs->header_demod);
    qpacketmodem_destroy(fs->header_decoder);
#if FRAMESYNC_ENABLE_EQ
    eqlms_cccf_destroy(fs->eq);
#endif
    nco_crcf_destroy(fs->mixer);
    firpfb_crcf_destroy(fs->mf);

    pthread_mutex_lock(&fs->detector_buf_mutex);

    while(1)
    {
        uint8_t all_done = 1;

        for(size_t i = 0; i < fs->detector_n_threads; i++)
        {
            if(fs->detector_buf[i].state == FRAMESYNC_BUFFER_PROCESSING)
            {
                all_done = 0;

                break;
            }
        }

        if(all_done)
            break;

        pthread_mutex_unlock(&fs->detector_buf_mutex);
        sched_yield();
        pthread_mutex_lock(&fs->detector_buf_mutex);
    }

    framesync_detector_buffer_t *_b = fs->detector_buf;
    fs->detector_buf = NULL;
    pthread_cond_broadcast(&fs->detector_buf_cond);

    pthread_mutex_unlock(&fs->detector_buf_mutex);

    for(size_t i = 0; i < fs->detector_n_threads; i++)
        pthread_join(fs->detector_thread[i], NULL);

    fs->detector_buf = _b;

    for(size_t i = 0; i < fs->detector_n_threads; i++)
    {
        qdetector_cccf_destroy(fs->detector[i]);

        if(fs->detector_buf[i].buf)
            free(fs->detector_buf[i].buf);
    }

    free(fs->payload_sym);
    free(fs->payload_dec);
    free(fs->header_sym);
    free(fs->header_mod);
    free(fs->header_dec);
    free(fs->detector_buf);
    free(fs->detector);
    free(fs->detector_mutex);
    free(fs->detector_thread);
    free(fs->preamble_rx);
    free(fs->preamble_pn);
    free(fs);
}
void framesync_reset(framesync_t *fs)
{
    if(!fs)
        return;

    for(size_t i = 0; i < fs->detector_n_threads; i++)
    {
        pthread_mutex_lock(&fs->detector_mutex[i]);

        qdetector_cccf_reset(fs->detector[i]);

        pthread_mutex_unlock(&fs->detector_mutex[i]);
    }

    firpfb_crcf_reset(fs->mf);
    nco_crcf_reset(fs->mixer);
#if FRAMESYNC_ENABLE_EQ
    eqlms_cccf_reset(fs->eq);
#endif

    fs->preamble_counter = 0;
    fs->symbol_counter = 0;
    fs->state = FRAMESYNC_STATE_DETECTFRAME;
}
void framesync_set_callback(framesync_t *fs, framesync_packet_received_cb_t cb, void *cb_ptr)
{
    if(!fs)
        return;

    fs->cb = cb;
    fs->cb_ptr = cb_ptr;
}
void framesync_get_header_props(framesync_t *fs, framegenprops_t *props)
{
    if(!fs || !props)
        return;

    memmove(props, &fs->header_props, sizeof(framegenprops_t));
}
uint8_t framesync_get_header_soft_demod(framesync_t *fs)
{
    if(!fs)
        return 0;

    return !!fs->header_soft;
}
void framesync_set_header_soft_demod(framesync_t *fs, uint8_t soft)
{
    if(!fs)
        return;

    fs->header_soft = !!soft;
}
uint8_t framesync_get_payload_soft_demod(framesync_t *fs)
{
    if(!fs)
        return 0;

    return !!fs->payload_soft;
}
void framesync_set_payload_soft_demod(framesync_t *fs, uint8_t soft)
{
    if(!fs)
        return;

    fs->payload_soft = !!soft;
}
size_t framesync_process_samples(framesync_t *fs, float complex *buffer, size_t buffer_len)
{
    if(!fs)
        return 0;

    if(buffer_len && !buffer)
        return 0;

    framesync_process_detector_out(fs);

    if(!buffer || !buffer_len)
        return 0;

    return framesync_process_samples_int(fs, buffer, buffer_len);
}
