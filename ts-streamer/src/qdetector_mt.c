#include "qdetector_mt.h"

int qdetector_mt_cccf_execute_seek(qdetector_mt_cccf_t *_q, float complex _x);
int qdetector_mt_cccf_execute_align(qdetector_mt_cccf_t *_q, float complex _x);

qdetector_mt_cccf_t * qdetector_mt_cccf_create(float complex *_s, unsigned int _s_len)
{
    // validate input
    if(_s_len == 0)
        return NULL;

    // allocate memory for main object and set internal properties
    qdetector_mt_cccf_t * q = (qdetector_mt_cccf_t *) malloc(sizeof(qdetector_mt_cccf_t));
    q->s_len = _s_len;

    // allocate memory and copy sequence
    q->s = (float complex *) malloc(q->s_len * sizeof(float complex));
    memmove(q->s, _s, q->s_len*sizeof(float complex));
    q->s_bar = (float complex *) malloc(q->s_len * sizeof(float complex));

    for(unsigned int i=0; i<q->s_len; i++)
        q->s_bar[i] = conjf(q->s[i]);

    q->s2_sum = liquid_sumsqcf(q->s, q->s_len); // compute sum{ s^2 }
    q->s_rms  = sqrtf(q->s2_sum);               // compute sqrt{ sum{ s^2 } }

    // prepare transforms
    q->nfft       = 1 << liquid_nextpow2( (unsigned int)( 2 * q->s_len ) ); // NOTE: must be even
    q->buf_time_0 = (float complex *) fftwf_malloc(q->nfft * sizeof(float complex));
    q->buf_freq_0 = (float complex *) fftwf_malloc(q->nfft * sizeof(float complex));
    q->buf_freq_1 = (float complex *) fftwf_malloc(q->nfft * sizeof(float complex));
    q->buf_time_1 = (float complex *) fftwf_malloc(q->nfft * sizeof(float complex));

    q->fft  = fftwf_plan_dft_1d(q->nfft, q->buf_time_0, q->buf_freq_0, FFTW_FORWARD,  0);
    q->ifft = fftwf_plan_dft_1d(q->nfft, q->buf_freq_1, q->buf_time_1, FFTW_BACKWARD, 0);

    // create frequency-domain template by taking nfft-point transform on 's', storing in 'S'
    q->S_bar = (float complex *) malloc(q->nfft * sizeof(float complex));
    memset(q->buf_time_0, 0x00, q->nfft*sizeof(float complex));
    memmove(q->buf_time_0, q->s, q->s_len*sizeof(float complex));
    fftwf_execute(q->fft);

    for(unsigned int i=0; i<q->nfft; i++)
        q->S_bar[i] = conjf(q->buf_freq_0[i]);

    q->g0_1 = sqrtf((float)q->s_len / (float)(q->nfft / 2));
    q->g0_2 = sqrtf((float)q->s_len / (float)q->nfft);

    // reset state variables
    q->counter        = q->nfft/2;
    q->x2_sum_0       = 0.0f;
    q->x2_sum_1       = 0.0f;
    q->state          = QDETECTOR_STATE_SEEK;
    q->frame_detected = 0;
    memset(q->buf_time_0, 0x00, q->nfft*sizeof(float complex));

    // reset estimates
    q->rxy       = 0.0f;
    q->tau_hat   = 0.0f;
    q->gamma_hat = 0.0f;
    q->dphi_hat  = 0.0f;
    q->phi_hat   = 0.0f;

    // thread data
    q->t_size     = 8; // TODO: Configure
    q->t          = (pthread_t *)malloc(q->t_size * sizeof(pthread_t));
    q->t_start    = (int *)malloc(q->t_size * sizeof(int));
    q->t_end      = (int *)malloc(q->t_size * sizeof(int));
    q->t_buf_freq = (float complex **)malloc(q->t_size * sizeof(float complex *));
    q->t_buf_time = (float complex **)malloc(q->t_size * sizeof(float complex *));
    q->t_ifft     = (fftwf_plan *)malloc(q->t_size * sizeof(fftwf_plan));
    q->t_rxy_peak = (float *)malloc(q->t_size * sizeof(float));
    q->t_rxy_index = (unsigned int *)malloc(q->t_size * sizeof(unsigned int));
    q->t_rxy_offset = (int *)malloc(q->t_size * sizeof(int));

    for(size_t i=0; i<q->t_size; i++)
    {
        q->t_start[i]      = 0;
        q->t_end[i]        = 0;
        q->t_buf_freq[i]   = (float complex *) fftwf_malloc(q->nfft * sizeof(float complex));
        q->t_buf_time[i]   = (float complex *) fftwf_malloc(q->nfft * sizeof(float complex));
        q->t_ifft[i]       = fftwf_plan_dft_1d(q->nfft, q->t_buf_freq[i], q->t_buf_time[i], FFTW_BACKWARD, 0);
    }

    qdetector_mt_cccf_set_threshold(q,0.5f);
    qdetector_mt_cccf_set_range    (q,0.3f); // set initial range for higher detection

    // return object
    return q;
}
qdetector_mt_cccf_t * qdetector_mt_cccf_create_linear(float complex *_sequence, unsigned int _sequence_len, int _ftype, unsigned int _k, unsigned int _m, float _beta)
{
    // validate input
    if(_sequence_len == 0)
        return NULL;
    if(_k < 2 || _k > 80)
        return NULL;
    if(_m < 1 || _m > 100)
        return NULL;
    if(_beta < 0.0f || _beta > 1.0f)
        return NULL;

    // create time-domain template
    unsigned int    s_len = _k * (_sequence_len + 2*_m);
    float complex * s     = (float complex *) malloc(s_len * sizeof(float complex));
    firinterp_crcf interp = firinterp_crcf_create_prototype(_ftype, _k, _m, _beta, 0);
    unsigned int i;
    for(i=0; i<_sequence_len + 2*_m; i++)
        firinterp_crcf_execute(interp, i < _sequence_len ? _sequence[i] : 0, &s[_k*i]);
    firinterp_crcf_destroy(interp);

    // create main object
    qdetector_mt_cccf_t * q = qdetector_mt_cccf_create(s, s_len);

    // free allocated temporary array
    free(s);

    // return object
    return q;
}
qdetector_mt_cccf_t * qdetector_mt_cccf_create_gmsk(unsigned char *_sequence, unsigned int _sequence_len, unsigned int _k, unsigned int _m, float _beta)
{
    // validate input
    if(_sequence_len == 0)
        return NULL;
    if(_k < 2 || _k > 80)
        return NULL;
    if(_m < 1 || _m > 100)
        return NULL;
    if(_beta < 0.0f || _beta > 1.0f)
        return NULL;

    // create time-domain template using GMSK modem
    unsigned int    s_len = _k * (_sequence_len + 2*_m);
    float complex * s     = (float complex *) malloc(s_len * sizeof(float complex));
    gmskmod mod = gmskmod_create(_k, _m, _beta);
    unsigned int i;
    for(i=0; i<_sequence_len + 2*_m; i++)
        gmskmod_modulate(mod, i < _sequence_len ? _sequence[i] : 0, &s[_k*i]);
    gmskmod_destroy(mod);

    // create main object
    qdetector_mt_cccf_t * q = qdetector_mt_cccf_create(s, s_len);

    // free allocated temporary array
    free(s);

    // return object
    return q;
}
qdetector_mt_cccf_t * qdetector_mt_cccf_create_cpfsk(unsigned char *_sequence, unsigned int _sequence_len, unsigned int _bps, float _h, unsigned int _k, unsigned int _m, float _beta, int _type)
{
    // validate input
    if(_sequence_len == 0)
        return NULL;
    if(_k < 2 || _k > 80)
        return NULL;
    if(_m < 1 || _m > 100)
        return NULL;
    if(_beta < 0.0f || _beta > 1.0f)
        return NULL;

    // create time-domain template using GMSK modem
    unsigned int s_len = _k * (_sequence_len + 2*_m);
    float complex *         s     = (float complex *) malloc(s_len * sizeof(float complex));
    cpfskmod mod = cpfskmod_create(_bps, _h, _k, _m, _beta, _type);
    unsigned int i;
    for(i=0; i<_sequence_len + 2*_m; i++)
        cpfskmod_modulate(mod, i < _sequence_len ? _sequence[i] : 0, &s[_k*i]);
    cpfskmod_destroy(mod);

    // create main object
    qdetector_mt_cccf_t * q = qdetector_mt_cccf_create(s, s_len);

    // free allocated temporary array
    free(s);

    // return object
    return q;
}
qdetector_mt_cccf_t * qdetector_mt_cccf_copy(qdetector_mt_cccf_t * q_orig)
{
    // validate input
    if(q_orig == NULL)
        return NULL;

    // create new object from internal sequence
    qdetector_mt_cccf_t * q_copy = qdetector_mt_cccf_create(q_orig->s, q_orig->s_len);

    // copy buffer contents
    memmove(q_copy->buf_time_0, q_orig->buf_time_0, q_orig->nfft*sizeof(float complex));
    memmove(q_copy->buf_freq_0, q_orig->buf_freq_0, q_orig->nfft*sizeof(float complex));
    memmove(q_copy->buf_time_1, q_orig->buf_time_1, q_orig->nfft*sizeof(float complex));
    memmove(q_copy->buf_freq_1, q_orig->buf_freq_1, q_orig->nfft*sizeof(float complex));

    // copy internal state
    q_copy->counter         = q_orig->counter;
    q_copy->threshold       = q_orig->threshold;
    q_copy->dphi_max        = q_orig->dphi_max;
    q_copy->range           = q_orig->range;
    // buffer power magnitude
    q_copy->x2_sum_0        = q_orig->x2_sum_0;
    q_copy->x2_sum_1        = q_orig->x2_sum_1;
    // state variables
    q_copy->state           = q_orig->state;
    q_copy->frame_detected  = q_orig->frame_detected;

    // return new object
    return q_copy;
}
int qdetector_mt_cccf_destroy(qdetector_mt_cccf_t *_q)
{
    // free allocated arrays
    free(_q->s);
    free(_q->s_bar);
    free(_q->S_bar);
    fftwf_free(_q->buf_time_0);
    fftwf_free(_q->buf_freq_0);
    fftwf_free(_q->buf_freq_1);
    fftwf_free(_q->buf_time_1);

    // destroy objects
    fftwf_destroy_plan(_q->fft);
    fftwf_destroy_plan(_q->ifft);

    // free thread data
    for(size_t i=0; i<_q->t_size; i++)
    {
        fftwf_free(_q->t_buf_freq[i]);
        fftwf_free(_q->t_buf_time[i]);
        fftwf_destroy_plan(_q->t_ifft[i]);
    }

    free(_q->t);
    free(_q->t_start);
    free(_q->t_end);
    free(_q->t_buf_freq);
    free(_q->t_buf_time);
    free(_q->t_ifft);
    free(_q->t_rxy_peak);
    free(_q->t_rxy_index);
    free(_q->t_rxy_offset);

    // free main object memory
    free(_q);
    return LIQUID_OK;
}
int qdetector_mt_cccf_reset(qdetector_mt_cccf_t *_q)
{
    return LIQUID_OK;
}
void * qdetector_mt_cccf_execute(qdetector_mt_cccf_t *_q, float complex _x)
{
    switch (_q->state) {
    case QDETECTOR_STATE_SEEK:
        // seek signal
        qdetector_mt_cccf_execute_seek(_q, _x);
        break;

    case QDETECTOR_STATE_ALIGN:
        // align signal
        qdetector_mt_cccf_execute_align(_q, _x);
        break;
    }

    // check if frame was detected
    if(_q->frame_detected) {
        // clear flag
        _q->frame_detected = 0;

        // return pointer to internal buffer of saved samples
        return (void*)(_q->buf_time_1);
    }

    // frame not yet ready
    return NULL;
}
float qdetector_mt_cccf_get_threshold(qdetector_mt_cccf_t *_q)
{
    return _q->threshold;
}
int qdetector_mt_cccf_set_threshold(qdetector_mt_cccf_t *_q, float _threshold)
{
    if(_threshold <= 0.0f || _threshold > 2.0f)
        return -LIQUID_EICONFIG;

    // set internal threshold value
    _q->threshold = _threshold;
    return LIQUID_OK;
}
float qdetector_mt_cccf_get_range(qdetector_mt_cccf_t *_q)
{
    return _q->dphi_max;
}
int qdetector_mt_cccf_set_range(qdetector_mt_cccf_t *_q, float _dphi_max)
{
    if(_dphi_max < 0.0f || _dphi_max > 0.5f)
        return -LIQUID_EICONFIG;

    // set internal search range
    _q->dphi_max = _dphi_max;
    _q->range    = (int)(_q->dphi_max *_q->nfft / (2*M_PI));
    _q->range    = _q->range < 0 ? 0 : _q->range;
    //printf("range: %d / %u\n", _q->range, _q->nfft);

    size_t n_iter = 2 * _q->range + 1;
    size_t t_range = (n_iter + _q->t_size - 1) / _q->t_size;
    int iter = -_q->range;

    for(size_t i=0; i<_q->t_size; i++)
    {
        size_t _n_iter = t_range < n_iter ? t_range : n_iter;

        _q->t_start[i] = iter;
        _q->t_end[i]   = iter + _n_iter - 1;

        n_iter -= _n_iter;
        iter += _n_iter;
    }

    return LIQUID_OK;
}
unsigned int qdetector_mt_cccf_get_seq_len(qdetector_mt_cccf_t *_q)
{
    return _q->s_len;
}
const void * qdetector_mt_cccf_get_sequence(qdetector_mt_cccf_t *_q)
{
    return (const void*) _q->s;
}
unsigned int qdetector_mt_cccf_get_buf_len(qdetector_mt_cccf_t *_q)
{
    return _q->nfft;
}
float qdetector_mt_cccf_get_rxy(qdetector_mt_cccf_t *_q)
{
    return _q->rxy;
}
float qdetector_mt_cccf_get_tau(qdetector_mt_cccf_t *_q)
{
    return _q->tau_hat;
}
float qdetector_mt_cccf_get_gamma(qdetector_mt_cccf_t *_q)
{
    return _q->gamma_hat;
}
float qdetector_mt_cccf_get_dphi(qdetector_mt_cccf_t *_q)
{
    return _q->dphi_hat;
}
float qdetector_mt_cccf_get_phi(qdetector_mt_cccf_t *_q)
{
    return _q->phi_hat;
}

typedef struct {
    size_t i;
    qdetector_mt_cccf_t *q;
    float g;
} t_arg;

void *qdetector_mt_cccf_execute_seek_t(void *p)
{
    t_arg *arg = (t_arg *)p;
    size_t t_i = arg->i;
    qdetector_mt_cccf_t *q = arg->q;
    float g = arg->g;

    q->t_rxy_peak[t_i] = 0.0f;
    q->t_rxy_index[t_i] = 0;
    q->t_rxy_offset[t_i] = 0;

    for(int offset = q->t_start[t_i]; offset <= q->t_end[t_i]; offset++)
    {
        for(unsigned int i = 0; i < q->nfft; i++)
        {
            unsigned int j = (i + q->nfft - offset) % q->nfft;

            q->t_buf_freq[t_i][i] = q->buf_freq_0[i] * q->S_bar[j];
        }

        fftwf_execute(q->t_ifft[t_i]);

        for(unsigned int i = 0; i < q->nfft; i++)
        {
            float rxy_abs = cabsf(q->t_buf_time[t_i][i]) * g;

            if(rxy_abs > q->t_rxy_peak[t_i])
            {
                q->t_rxy_peak[t_i] = rxy_abs;
                q->t_rxy_index[t_i] = i;
                q->t_rxy_offset[t_i] = offset;
            }
        }
    }

    return NULL;
}
int qdetector_mt_cccf_execute_seek(qdetector_mt_cccf_t *_q, float complex _x)
{
    // write sample to buffer and increment counter
    _q->buf_time_0[_q->counter++] = _x;

    // accumulate signal magnitude
    _q->x2_sum_1 += crealf(_x)*crealf(_x) + cimagf(_x)*cimagf(_x);

    if(_q->counter < _q->nfft)
        return LIQUID_OK;

    // reset counter (last half of time buffer)
    _q->counter = _q->nfft/2;

    // run forward transform
    fftwf_execute(_q->fft);

    // compute scaling factor (TODO: use median rather than mean signal level)
    float g0;

    if(_q->x2_sum_0 == 0.f)
        g0 = sqrtf(_q->x2_sum_1) * _q->g0_1;
    else
        g0 = sqrtf(_q->x2_sum_0 + _q->x2_sum_1) * _q->g0_2;

    if(g0 < 1e-10) {
        memmove(_q->buf_time_0, _q->buf_time_0 + _q->nfft / 2, (_q->nfft / 2) * sizeof(float complex));

        // swap accumulated signal levels
        _q->x2_sum_0 = _q->x2_sum_1;
        _q->x2_sum_1 = 0.0f;
        return LIQUID_OK;
    }

    float g = 1.0f / ((float)_q->nfft * g0 * _q->s_rms);

    // t_arg arg[_q->t_size];

    // for(size_t i=0; i<_q->t_size; i++)
    // {
    //     if(_q->t_start[i] > _q->t_end[i])
    //         break;

    //     arg[i].i = i;
    //     arg[i].q = _q;
    //     arg[i].g = g;

    //     pthread_create(&_q->t[i], NULL, qdetector_mt_cccf_execute_seek_t, (void *)&arg[i]);
    // }

    float        rxy_peak   = 0.0f;
    unsigned int rxy_index  = 0;
    int          rxy_offset = 0;

    // for(size_t i=0; i<_q->t_size; i++)
    // {
    //     if(_q->t_start[i] > _q->t_end[i])
    //         break;

    //     pthread_join(_q->t[i], NULL);

    //     if(_q->t_rxy_peak[i] > rxy_peak)
    //     {
    //         rxy_peak   = _q->t_rxy_peak[i];
    //         rxy_index  = _q->t_rxy_index[i];
    //         rxy_offset = _q->t_rxy_offset[i];
    //     }
    // }

    for(int offset = -_q->range; offset <= _q->range; offset++)
    {
        for(unsigned int i = 0; i < _q->nfft; i++)
        {
            unsigned int j = (i + _q->nfft - offset) % _q->nfft;

            _q->buf_freq_1[i] = _q->buf_freq_0[i] * _q->S_bar[j];
        }

        fftwf_execute(_q->ifft);

        for(unsigned int i = 0; i < _q->nfft; i++)
        {
            float rxy_abs = cabsf(_q->buf_time_1[i]) * g;

            if(rxy_abs > rxy_peak)
            {
                rxy_peak   = rxy_abs;
                rxy_index  = i;
                rxy_offset = offset;
            }
        }
    }

    if(rxy_peak > _q->threshold && rxy_index < _q->nfft - _q->s_len) {
        // update state, reset counter, copy buffer appropriately
        _q->state = QDETECTOR_STATE_ALIGN;
        _q->offset = rxy_offset;
        _q->rxy    = rxy_peak; // note that this is a coarse estimate
        // TODO: check for edge case where rxy_index is zero (signal already aligned)

        // copy last part of fft input buffer to front
        memmove(_q->buf_time_0, _q->buf_time_0 + rxy_index, (_q->nfft - rxy_index)*sizeof(float complex));
        _q->counter = _q->nfft - rxy_index;

        return LIQUID_OK;
    }

    // copy last half of fft input buffer to front
    memmove(_q->buf_time_0, _q->buf_time_0 + _q->nfft/2, (_q->nfft/2)*sizeof(float complex));

    // swap accumulated signal levels
    _q->x2_sum_0 = _q->x2_sum_1;
    _q->x2_sum_1 = 0.0f;
    return LIQUID_OK;
}
int qdetector_mt_cccf_execute_align(qdetector_mt_cccf_t *_q, float complex _x)
{
    // write sample to buffer and increment counter
    _q->buf_time_0[_q->counter++] = _x;

    if(_q->counter < _q->nfft)
        return LIQUID_OK;

    //printf("signal is aligned!\n");

    // estimate timing offset
    fftwf_execute(_q->fft);
    // cross-multiply frequency-domain components, aligning appropriately with
    // estimated FFT offset index due to carrier frequency offset in received signal
    unsigned int i;
    for(i=0; i<_q->nfft; i++) {
        // shifted index
        unsigned int j = (i + _q->nfft - _q->offset) % _q->nfft;
        _q->buf_freq_1[i] = _q->buf_freq_0[i] * _q->S_bar[j];
    }
    fftwf_execute(_q->ifft);
    // time aligned to index 0
    // NOTE: taking the sqrt removes bias in the timing estimate, but messes up gamma estimate
    float yneg = cabsf(_q->buf_time_1[_q->nfft-1]);  yneg = sqrtf(yneg);
    float y0   = cabsf(_q->buf_time_1[         0]);  y0   = sqrtf(y0  );
    float ypos = cabsf(_q->buf_time_1[         1]);  ypos = sqrtf(ypos);
    // compute timing offset estimate from quadratic polynomial fit
    //  y = a x^2 + b x + c, [xneg = -1, x0 = 0, xpos = +1]
    float a     =  0.5f*(ypos + yneg) - y0;
    float b     =  0.5f*(ypos - yneg);
    float c     =  y0;
    _q->tau_hat = -b / (2.0f*a); //-0.5f*(ypos - yneg) / (ypos + yneg - 2*y0);
    float g_hat   = (a*_q->tau_hat*_q->tau_hat + b*_q->tau_hat + c);
    _q->gamma_hat = g_hat * g_hat / ((float)(_q->nfft) *_q->s2_sum); // g_hat^2 because of sqrt for yneg/y0/ypos
    // TODO: revise estimate of rxy here

    // copy buffer to preserve data integrity
    memmove(_q->buf_time_1, _q->buf_time_0, _q->nfft*sizeof(float complex));

    // estimate carrier frequency offset
    for(i=0; i<_q->nfft; i++)
        _q->buf_time_0[i] *= i < _q->s_len ? _q->s_bar[i] : 0.0f;
    fftwf_execute(_q->fft);
    // search for peak (NOTE: should be at: _q->offset)
    // TODO: don't search for peak but just use internal offset
    float        v0 = 0.0f;
    unsigned int i0 = 0;
    for(i=0; i<_q->nfft; i++) {
        float v_abs = cabsf(_q->buf_freq_0[i]);
        if(v_abs > v0) {
            v0 = v_abs;
            i0 = i;
        }
    }
    // interpolate using quadratic polynomial for carrier frequency estimate
    unsigned int ineg = (i0 + _q->nfft - 1)%_q->nfft;
    unsigned int ipos = (i0            + 1)%_q->nfft;
    float        vneg = cabsf(_q->buf_freq_0[ineg]);
    float        vpos = cabsf(_q->buf_freq_0[ipos]);
    a            =  0.5f*(vpos + vneg) - v0;
    b            =  0.5f*(vpos - vneg);
    //c            =  v0;
    float idx    = -b / (2.0f*a); //-0.5f*(vpos - vneg) / (vpos + vneg - 2*v0);
    float index  = (float)i0 + idx;
    _q->dphi_hat = (i0 > _q->nfft/2 ? index-(float)_q->nfft : index) * 2*M_PI / (float)(_q->nfft);

    // estimate carrier phase offset
#if 0
    // METHOD 1: linear interpolation of phase in FFT output buffer
    float p0     = cargf(_q->buf_freq_0[ idx < 0 ? ineg : i0   ]);
    float p1     = cargf(_q->buf_freq_0[ idx < 0 ? i0   : ipos ]);
    float xp     = idx < 0 ? 1+idx : idx;
    _q->phi_hat  = (p1-p0)*xp + p0;
    //printf("v0 = %12.8f, v1 = %12.8f, xp = %12.8f\n", v0, v1, xp);
#else
    // METHOD 2: compute metric by de-rotating signal and measuring resulting phase
    // NOTE: this is possibly more accurate than the above method but might also
    //       be more computationally complex
    float complex metric = 0;
    for(i=0; i<_q->s_len; i++)
        metric += _q->buf_time_0[i] * cexpf(-_Complex_I*_q->dphi_hat*i);
    //printf("metric : %12.8f <%12.8f>\n", cabsf(metric), cargf(metric));
    _q->phi_hat = cargf(metric);
#endif

    // set flag
    _q->frame_detected = 1;

    // reset state
    // copy saved buffer state (last half of buf_time_1 to front half of buf_time_0)
    memmove(_q->buf_time_0, _q->buf_time_1 + _q->nfft/2, (_q->nfft/2)*sizeof(float complex));
    _q->state = QDETECTOR_STATE_SEEK;
    _q->x2_sum_0 = liquid_sumsqcf(_q->buf_time_0, _q->nfft/2);
    _q->x2_sum_1 = 0;
    _q->counter = _q->nfft/2;
    return LIQUID_OK;
}