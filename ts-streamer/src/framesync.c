#include "framesync.h"

static void framesync_reconfigure(framesync_t *fs)
{
    if(fs->payload_dec_len != fs->payload_dec_len_p || fs->payload_props.crc != fs->payload_props_p.crc || fs->payload_props.i_fec != fs->payload_props_p.i_fec || fs->payload_props.o_fec != fs->payload_props_p.o_fec || fs->payload_props.mod != fs->payload_props_p.mod || fs->payload_props.pilots != fs->payload_props_p.pilots)
    {
        fs->payload_dec = (uint8_t *)realloc(fs->payload_dec, fs->payload_dec_len * sizeof(uint8_t));
        fs->payload_decoder = packetizer_recreate(fs->payload_decoder, fs->payload_dec_len, fs->payload_props.crc, fs->payload_props.o_fec, fs->payload_props.i_fec);
        fs->payload_enc_len = packetizer_get_enc_msg_len(fs->payload_decoder);
        fs->payload_enc = (uint8_t *)realloc(fs->payload_enc, fs->payload_enc_len * sizeof(uint8_t));
        fs->payload_demod = modemcf_recreate(fs->payload_demod, fs->payload_props.mod);
        fs->payload_mod_len = fs->payload_props.pilots ? ceilf((float)fs->payload_enc_len * 8 / (float)modem_get_bps(fs->payload_demod)) : 0;
        fs->payload_mod = fs->payload_props.pilots ? (float complex *)realloc(fs->payload_mod, fs->payload_mod_len * sizeof(float complex)) : fs->payload_mod;
        fs->payload_pilotsync = fs->payload_props.pilots ? qpilotsync_recreate(fs->payload_pilotsync, fs->payload_mod_len, 16) : fs->payload_pilotsync;
        fs->payload_sym_len = fs->payload_props.pilots ? qpilotsync_get_frame_len(fs->payload_pilotsync) : ceilf((float)fs->payload_enc_len * 8 / (float)modem_get_bps(fs->payload_demod));
        fs->payload_sym = fs->payload_props.pilots ? (float complex *)realloc(fs->payload_sym, fs->payload_sym_len * sizeof(float complex)) : fs->payload_sym;
    }

    fs->payload_props_p.crc = fs->payload_props.crc;
    fs->payload_props_p.i_fec = fs->payload_props.i_fec;
    fs->payload_props_p.o_fec = fs->payload_props.o_fec;
    fs->payload_props_p.mod = fs->payload_props.mod;
    fs->payload_props_p.pilots = fs->payload_props.pilots;
    fs->payload_dec_len_p = fs->payload_dec_len;
}
static void framesync_decode_header(framesync_t *fs)
{
    fs->header_valid = packetizer_decode(fs->header_decoder, fs->header_enc, fs->header_dec);

    if(!fs->header_valid)
        return;

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
    uint8_t pilots = !!(fs->header_dec[5] & 0x80);

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

    fs->payload_props.mod = mod;
    fs->payload_props.crc = crc;
    fs->payload_props.i_fec = i_fec;
    fs->payload_props.o_fec = o_fec;
    fs->payload_props.pilots = pilots;
    fs->payload_dec_len = payload_dec_len;
}

static void framesync_process_seekpn(framesync_t *fs, float complex sym, unsigned int demod_sym)
{
    // Process PN sequence and try to align to it
    fs->symbol_counter++;

    size_t bps = modem_get_bps(fs->preamble_demod);
    float len = bsequence_get_length(fs->preamble_pn);

    float best_r = 0.8f; // Threshold
    int best_i = -1;

    for(size_t i = 0; i < 4; i++)
    {
        if(i > 0)
        {
            sym *= cexpf(M_PI / 2.0f * _Complex_I); // Rotate 90 degrees

            modemcf_demodulate(fs->preamble_demod, sym, &demod_sym); // Re-demodulate
        }

        for(size_t j = 0; j < bps; j++)
            bsequence_push(fs->preamble_rx[i], (demod_sym >> (bps - j - 1)) & 0x01);

        float rxy = bsequence_correlate(fs->preamble_pn, fs->preamble_rx[i]);
        float r = 2.0f * rxy / len - 1.0f; // > 0 if not inverted, < 0 if inverted

        if(r > best_r) // Check actual value, since we do not want to lock on an inverted phase
        {
            best_r = r;
            best_i = i;
        }
    }

    if(best_i < 0)
    {
        if(fs->symbol_counter >= 10 * fs->preamble_sym_len)
        {
            fs->symbol_counter = 0;

            nco_crcf_reset(fs->mixer); // Prevent runaway NCO
        }

        return;
    }

    // Adjust mixer phase to de-rotate the remaining bits
    nco_crcf_adjust_phase(fs->mixer, M_PI / 2.0f * best_i);

    // Reset counters and move on
    fs->symbol_counter = 0;
    fs->byte_counter = 0;
    fs->bit_counter = 0;
    fs->state = FRAMESYNC_STATE_HEADER;
}
static void framesync_process_pn(framesync_t *fs, float complex sym, unsigned int demod_sym)
{
    // Count the number of preamble symbols received
    (void)sym;
    (void)demod_sym;

    fs->symbol_counter++;

    if(fs->symbol_counter == fs->preamble_sym_len)
    {
        // Reset counters and move on
        fs->symbol_counter = 0;
        fs->byte_counter = 0;
        fs->bit_counter = 0;
        fs->state = FRAMESYNC_STATE_HEADER;
    }
}
static void framesync_process_header(framesync_t *fs, float complex sym, unsigned int demod_sym)
{
    // Process header symbols TODO: Support soft demodulation when pilots are off
    if(fs->header_props.pilots)
    {
        // Pilots are enabled, we need to store all symbols before decoding
        fs->header_sym[fs->symbol_counter++] = sym;

        if(fs->symbol_counter < fs->header_sym_len)
            return;

        // All here, decode
        qpilotsync_execute(fs->header_pilotsync, fs->header_sym, fs->header_mod);

        for(size_t i = 0; i < fs->header_mod_len; i++)
        {
            modemcf_demodulate(fs->header_demod, fs->header_mod[i], &demod_sym);

            size_t bps = modem_get_bps(fs->header_demod);

            for(size_t j = 0; j < bps; j++)
            {
                if(!fs->bit_counter)
                    fs->header_enc[fs->byte_counter] = 0x00;

                uint8_t bit = (demod_sym >> (bps - j - 1)) & 0x01;

                fs->header_enc[fs->byte_counter] |= bit << (7 - fs->bit_counter++);

                if(fs->bit_counter == 8)
                {
                    fs->bit_counter = 0;
                    fs->byte_counter++;
                }
            }
        }
    }
    else
    {
        // No pilots, we can start decoding as we receive symbols
        float evm = modemcf_get_demodulator_evm(fs->header_demod);

        if(!fs->symbol_counter)
            fs->cb_stats.evm = evm * evm;
        else
            fs->cb_stats.evm += evm * evm;

        size_t bps = modem_get_bps(fs->header_demod);

        for(size_t i = 0; i < bps; i++)
        {
            if(!fs->bit_counter)
                fs->header_enc[fs->byte_counter] = 0x00;

            uint8_t bit = (demod_sym >> (bps - i - 1)) & 0x01;

            fs->header_enc[fs->byte_counter] |= bit << (7 - fs->bit_counter++);

            if(fs->bit_counter == 8)
            {
                fs->bit_counter = 0;
                fs->byte_counter++;
            }
        }

        fs->symbol_counter++;
    }

    if(fs->byte_counter == fs->header_enc_len)
    {
        framesync_decode_header(fs);
        framesync_reconfigure(fs);

        if(fs->header_valid)
        {
            // Reset counters and move on
            fs->symbol_counter = 0;
            fs->byte_counter = 0;
            fs->bit_counter = 0;
            fs->state = FRAMESYNC_STATE_PAYLOAD;
        }
        else
        {
            if(fs->cb)
            {
                fs->cb_stats.evm = fs->header_props.pilots ? qpilotsync_get_evm(fs->header_pilotsync) : 10.0f * log10f(fs->cb_stats.evm / (float)fs->symbol_counter);
                fs->cb_stats.rssi = 20.0f * log10f(1.f / agc_crcf_get_gain(fs->agc));
                fs->cb_stats.cfo = nco_crcf_get_frequency(fs->mixer);

                fs->cb(fs->cb_ptr, &fs->cb_stats, fs->header_dec + 6, fs->header_valid, NULL, 0, 0);
            }

            // Reset counters and move on
            fs->symbol_counter = 0;
            fs->byte_counter = 0;
            fs->bit_counter = 0;
            fs->state = FRAMESYNC_STATE_PN_SEEK;
        }
    }
}
static void framesync_process_payload(framesync_t *fs, float complex sym, unsigned int demod_sym)
{
    // Process payload symbols TODO: Support soft demodulation when pilots are off
    if(fs->payload_props.pilots)
    {
        // Pilots are enabled, we need to store all symbols before decoding
        fs->payload_sym[fs->symbol_counter++] = sym;

        if(fs->symbol_counter < fs->payload_sym_len)
            return;

        // All here, decode
        qpilotsync_execute(fs->payload_pilotsync, fs->payload_sym, fs->payload_mod);

        size_t bps = modem_get_bps(fs->payload_demod);

        for(size_t i = 0; i < fs->payload_mod_len; i++)
        {
            modemcf_demodulate(fs->payload_demod, fs->payload_mod[i], &demod_sym);

            for(size_t j = 0; j < bps; j++)
            {
                if(!fs->bit_counter)
                    fs->payload_enc[fs->byte_counter] = 0x00;

                uint8_t bit = (demod_sym >> (bps - j - 1)) & 0x01;

                fs->payload_enc[fs->byte_counter] |= bit << (7 - fs->bit_counter++);

                if(fs->bit_counter == 8)
                {
                    fs->bit_counter = 0;
                    fs->byte_counter++;
                }
            }
        }
    }
    else
    {
        // No pilots, we can start decoding as we receive symbols
        float evm = modemcf_get_demodulator_evm(fs->payload_demod);

        if(!fs->symbol_counter)
            fs->cb_stats.evm = evm * evm;
        else
            fs->cb_stats.evm += evm * evm;

        size_t bps = modem_get_bps(fs->payload_demod);

        for(size_t i = 0; i < bps; i++)
        {
            if(!fs->bit_counter)
                fs->payload_enc[fs->byte_counter] = 0x00;

            uint8_t bit = (demod_sym >> (bps - i - 1)) & 0x01;

            fs->payload_enc[fs->byte_counter] |= bit << (7 - fs->bit_counter++);

            if(fs->bit_counter == 8)
            {
                fs->bit_counter = 0;
                fs->byte_counter++;
            }
        }

        fs->symbol_counter++;
    }

    if(fs->byte_counter == fs->payload_enc_len)
    {
        fs->payload_valid = packetizer_decode(fs->payload_decoder, fs->payload_enc, fs->payload_dec);

        uint8_t force_relock = !fs->payload_valid;

        if(fs->cb)
        {
            fs->cb_stats.evm = fs->payload_props.pilots ? qpilotsync_get_evm(fs->payload_pilotsync) : 10.0f * log10f(fs->cb_stats.evm / (float)fs->symbol_counter);
            fs->cb_stats.rssi = 20.0f * log10f(1.f / agc_crcf_get_gain(fs->agc));
            fs->cb_stats.cfo = nco_crcf_get_frequency(fs->mixer);

            force_relock |= !!fs->cb(fs->cb_ptr, &fs->cb_stats, fs->header_dec + 6, fs->header_valid, fs->payload_dec, fs->payload_dec_len, fs->payload_valid);
        }

        // Reset counters and move on
        fs->symbol_counter = 0;
        fs->byte_counter = 0;
        fs->bit_counter = 0;
        fs->state = force_relock ? FRAMESYNC_STATE_PN_SEEK : FRAMESYNC_STATE_PN_RX_DELAY;
    }
}


framesync_t *framesync_create(framegenprops_t *h_props, size_t h_len)
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

    framesync_t *fs = (framesync_t *)malloc(sizeof(framesync_t));

    if(!fs)
        return NULL;

    // callback
    fs->cb = NULL;
    fs->cb_ptr = NULL;

    // carrier, timing recovery and eq objects
    float bw = 0.9f;

    fs->agc = agc_crcf_create();
    agc_crcf_set_bandwidth(fs->agc, 0.02f * bw);

    fs->mixer = nco_crcf_create(LIQUID_NCO);
    nco_crcf_pll_set_bandwidth(fs->mixer, 0.001f * bw);

    fs->symsync = symsync_crcf_create_rnyquist(LIQUID_FIRFILT_RRC, 2, 15, 0.2f, 32);
    symsync_crcf_set_lf_bw(fs->symsync, 0.001f * bw);
    symsync_crcf_set_output_rate(fs->symsync, 2);
    fs->symsync_n = 0;

    fs->eq = eqlms_cccf_create_lowpass(2 * 4 + 1, 0.45f);
    eqlms_cccf_set_bw(fs->eq, 0.02f * bw);
    fs->eq_strat = FRAMESYNC_EQ_STRAT_OFF;

    // preamble
    msequence pn_seq = msequence_create_default(6);

    fs->preamble_demod = modemcf_create(LIQUID_MODEM_QPSK); // Check the generator for more info

    fs->preamble_sym_len = ceilf((float)msequence_get_length(pn_seq) / (float)modem_get_bps(fs->preamble_demod));
    size_t pn_len = fs->preamble_sym_len * modem_get_bps(fs->preamble_demod);

    fs->preamble_pn = bsequence_create(pn_len);

    for(size_t i = 0; i < pn_len; i++)
        bsequence_push(fs->preamble_pn, msequence_advance(pn_seq));

    msequence_destroy(pn_seq);

    for(size_t i = 0; i < 4; i++)
        fs->preamble_rx[i] = bsequence_create(pn_len);

    // header
    memmove(&fs->header_props, h_props, sizeof(framegenprops_t));

    fs->header_user_len = h_len;
    fs->header_dec_len = 6 + h_len;
    fs->header_dec = (uint8_t *)malloc(fs->header_dec_len);
    fs->header_decoder = packetizer_create(fs->header_dec_len, fs->header_props.crc, fs->header_props.o_fec, fs->header_props.i_fec);
    fs->header_enc_len = packetizer_get_enc_msg_len(fs->header_decoder);
    fs->header_enc = (uint8_t *)malloc(fs->header_enc_len);
    fs->header_demod = modemcf_create(fs->header_props.mod);
    fs->header_mod_len = fs->header_props.pilots ? ceilf((float)fs->header_enc_len * 8 / (float)modem_get_bps(fs->header_demod)) : 0;
    fs->header_mod = fs->header_props.pilots ? (float complex *)malloc(fs->header_mod_len * sizeof(float complex)) : NULL;
    fs->header_pilotsync = fs->header_props.pilots ? qpilotsync_create(fs->header_mod_len, 16) : NULL;
    fs->header_sym_len = fs->header_props.pilots ? qpilotsync_get_frame_len(fs->header_pilotsync) : ceilf((float)fs->header_enc_len * 8 / (float)modem_get_bps(fs->header_demod));
    fs->header_sym = fs->header_props.pilots ? (float complex *)malloc(fs->header_sym_len * sizeof(float complex)) : NULL;
    fs->header_soft = 0;
    fs->header_valid = 0;

    // payload
    fs->payload_props_p.crc = LIQUID_CRC_NONE;
    fs->payload_props_p.i_fec = LIQUID_FEC_NONE;
    fs->payload_props_p.o_fec = LIQUID_FEC_NONE;
    fs->payload_props_p.mod = LIQUID_MODEM_QPSK;
    fs->payload_props_p.pilots = 0;
    fs->payload_props.crc = LIQUID_CRC_NONE;
    fs->payload_props.i_fec = LIQUID_FEC_NONE;
    fs->payload_props.o_fec = LIQUID_FEC_NONE;
    fs->payload_props.mod = LIQUID_MODEM_QPSK;
    fs->payload_props.pilots = 0;
    fs->payload_dec_len_p = 0;
    fs->payload_dec_len = 0;
    fs->payload_dec = NULL;
    fs->payload_decoder = packetizer_create(1, LIQUID_CRC_NONE, LIQUID_FEC_NONE, LIQUID_FEC_NONE);
    fs->payload_enc_len = 0;
    fs->payload_enc = NULL;
    fs->payload_demod = modemcf_create(LIQUID_MODEM_QPSK);
    fs->payload_mod_len = 0;
    fs->payload_mod = NULL;
    fs->payload_pilotsync = qpilotsync_create(128, 64);
    fs->payload_sym_len = 0;
    fs->payload_sym = NULL;
    fs->payload_soft = 0;
    fs->payload_valid = 0;

    framesync_reset(fs);

    return fs;
}
void framesync_delete(framesync_t *fs)
{
    if(!fs)
        return;

    // payload
    if(fs->payload_sym)
        free(fs->payload_sym);

    qpilotsync_destroy(fs->payload_pilotsync);

    if(fs->payload_mod)
        free(fs->payload_mod);

    modemcf_destroy(fs->payload_demod);

    if(fs->payload_enc)
        free(fs->payload_enc);

    packetizer_destroy(fs->payload_decoder);

    if(fs->payload_dec)
        free(fs->payload_dec);

    // header
    if(fs->header_sym)
        free(fs->header_sym);

    if(fs->header_pilotsync)
        qpilotsync_destroy(fs->header_pilotsync);

    if(fs->header_mod)
        free(fs->header_mod);

    modemcf_destroy(fs->header_demod);
    free(fs->header_enc);
    packetizer_destroy(fs->header_decoder);
    free(fs->header_dec);

    // preamble
    for(size_t i = 0; i < 4; i++)
        bsequence_destroy(fs->preamble_rx[i]);

    bsequence_destroy(fs->preamble_pn);
    modemcf_destroy(fs->preamble_demod);

    // carrier, timing recovery and eq objects
    eqlms_cccf_destroy(fs->eq);
    symsync_crcf_destroy(fs->symsync);
    nco_crcf_destroy(fs->mixer);
    agc_crcf_destroy(fs->agc);

    free(fs);
}
void framesync_reset(framesync_t *fs)
{
    if(!fs)
        return;

    agc_crcf_reset(fs->agc);
    nco_crcf_reset(fs->mixer);
    symsync_crcf_reset(fs->symsync);
    eqlms_cccf_reset(fs->eq);
    modemcf_reset(fs->preamble_demod);

    for(size_t i = 0; i < 4; i++)
        bsequence_reset(fs->preamble_rx[i]);

    modemcf_reset(fs->header_demod);

    if(fs->header_pilotsync)
        qpilotsync_reset(fs->header_pilotsync);

    modemcf_reset(fs->payload_demod);
    qpilotsync_reset(fs->payload_pilotsync);

    fs->symsync_n = 0;
    fs->symbol_counter = 0;
    fs->byte_counter = 0;
    fs->bit_counter = 0;
    fs->state = FRAMESYNC_STATE_PN_SEEK;
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
uint8_t framesync_get_eq_strategy(framesync_t *fs)
{
    if(!fs)
        return FRAMESYNC_EQ_STRAT_OFF;

    return fs->eq_strat;
}
void framesync_set_eq_strategy(framesync_t *fs, uint8_t strategy)
{
    if(!fs)
        return;

    switch(strategy)
    {
        case FRAMESYNC_EQ_STRAT_OFF:
        case FRAMESYNC_EQ_STRAT_CONST_MOD:
        case FRAMESYNC_EQ_STRAT_DEC_DIR:
        break;
        default:
            return;
    }

    fs->eq_strat = strategy;
}
void framesync_process_samples(framesync_t *fs, float complex *buffer, size_t buffer_len)
{
    if(!fs || !buffer || !buffer_len)
        return;

    for(size_t i = 0; i < buffer_len; i++)
    {
        float complex sym[8];
        unsigned int n = 0;

        agc_crcf_execute(fs->agc, buffer[i], &sym[0]);

        symsync_crcf_execute(fs->symsync, sym, 1, sym, &n);

        for(size_t j = 0; j < n; j++)
        {
            float complex _sym = sym[j];

            nco_crcf_step(fs->mixer);
            nco_crcf_mix_down(fs->mixer, _sym, &_sym);

            eqlms_cccf_push(fs->eq, _sym);

            fs->symsync_n++;
            if(!(fs->symsync_n & 1))
                continue;

            eqlms_cccf_execute(fs->eq, &_sym);

            // Select appropriate modem given the current state
            modemcf dem = NULL;

            switch(fs->state)
            {
                case FRAMESYNC_STATE_PN_SEEK:
                case FRAMESYNC_STATE_PN_RX_DELAY:
                    dem = fs->preamble_demod;
                break;
                case FRAMESYNC_STATE_HEADER:
                    dem = fs->header_demod;
                break;
                case FRAMESYNC_STATE_PAYLOAD:
                    dem = fs->payload_demod;
                break;
            }

            //demodulate and get phase error
            unsigned int demod_sym = 0;
            float phi_hat = 0.0f;

            if(dem)
            {
                modemcf_demodulate(dem, _sym, &demod_sym);
                phi_hat = modemcf_get_demodulator_phase_error(dem);
            }

            nco_crcf_pll_step(fs->mixer, phi_hat);

            if(fs->eq_strat != FRAMESYNC_EQ_STRAT_OFF) // TODO Support Equalizer, after first PN alignment
            {
                float complex _d = 0.0f;

                switch(fs->eq_strat)
                {
                    case FRAMESYNC_EQ_STRAT_CONST_MOD:
                    {
                        _d = _sym / cabsf(_sym);
                    }
                    break;
                    case FRAMESYNC_EQ_STRAT_DEC_DIR:
                    {
                        if(dem)
                            modemcf_get_demodulator_sample(dem, &_d);
                    }
                    break;
                }

                eqlms_cccf_step(fs->eq, _d, _sym);
            }

            switch(fs->state)
            {
                case FRAMESYNC_STATE_PN_SEEK:
                    framesync_process_seekpn(fs, _sym, demod_sym);
                break;
                case FRAMESYNC_STATE_PN_RX_DELAY:
                    framesync_process_pn(fs, _sym, demod_sym);
                break;
                case FRAMESYNC_STATE_HEADER:
                    framesync_process_header(fs, _sym, demod_sym);
                break;
                case FRAMESYNC_STATE_PAYLOAD:
                    framesync_process_payload(fs, _sym, demod_sym);
                break;
            }
        }
    }
}
