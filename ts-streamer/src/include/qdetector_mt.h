#ifndef __QDETECTOR_MT_H__
#define __QDETECTOR_MT_H__

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <pthread.h>
#include <signal.h>
#include <fftw3.h>
#include <liquid/liquid.h>

// Dectector states
#define QDETECTOR_STATE_SEEK 0
#define QDETECTOR_STATE_ALIGN 1

typedef struct qdetector_mt_cccf_t qdetector_mt_cccf_t;

struct qdetector_mt_cccf_t
{
    unsigned int        s_len;          // template (time) length: k * (sequence_len + 2 * m)
    float complex *     s;              // template (time), [size: s_len x 1]
    float complex *     s_bar;          // template (time), conj, [size: s_len x 1]
    float complex *     S_bar;          // template (freq), conj, [size: nfft x 1]
    float               s2_sum;         // sum{ s^2 }
    float               s_rms;          // sqrt{ sum{ s^2 } }

    float complex *     buf_time_0;     // time-domain buffer (FFT)
    float complex *     buf_freq_0;     // frequence-domain buffer (FFT)
    float complex *     buf_freq_1;     // frequence-domain buffer (IFFT)
    float complex *     buf_time_1;     // time-domain buffer (IFFT)
    unsigned int        nfft;           // fft size
    fftwf_plan          fft;            // FFT object:  buf_time_0 > buf_freq_0
    fftwf_plan          ifft;           // IFFT object: buf_freq_1 > buf_freq_1

    float               g0_1;           // sqrt(s_len/(nfft/2))
    float               g0_2;           // sqrt(s_len/nfft)

    unsigned int        counter;        // sample counter for determining when to compute FFTs
    float               threshold;      // detection threshold
    float               dphi_max;       // carrier offset search range (radians/sample)
    int                 range;          // carrier offset search range (subcarriers)

    float               x2_sum_0;       // sum{ |x|^2 } of first half of buffer
    float               x2_sum_1;       // sum{ |x|^2 } of second half of buffer

    float               rxy;            // peak correlation output
    int                 offset;         // FFT offset index for peak correlation (coarse carrier estimate)
    float               tau_hat;        // timing offset estimate
    float               gamma_hat;      // signal level estimate (channel gain)
    float               dphi_hat;       // carrier frequency offset estimate
    float               phi_hat;        // carrier phase offset estimate

    size_t              state;          // execution state
    int                 frame_detected; // frame detected?

    pthread_t *            t;            // thread objects
    size_t                 t_size;       // number of threads
    int *                  t_start;      // start index for each thread
    int *                  t_end;        // end index for each thread
    float complex **       t_buf_freq;   // frequence-domain buffer (IFFT) (1 per thread)
    float complex **       t_buf_time;   // time-domain buffer (IFFT) (1 per thread)
    fftwf_plan *           t_ifft;       // IFFT object: t_buf_freq > t_buf_freq (1 per thread)
    volatile sig_atomic_t *t_stat;       // thread status
    float *                t_rxy_peak;   // peak correlation output (per thread)
    unsigned int *         t_rxy_index;  // peak correlation index (per thread)
    int *                  t_rxy_offset; // FFT offset index for peak correlation (coarse carrier estimate) (per thread)
};

qdetector_mt_cccf_t * qdetector_mt_cccf_create(float complex *_s, unsigned int _s_len);
qdetector_mt_cccf_t * qdetector_mt_cccf_create_linear(float complex *_sequence, unsigned int _sequence_len, int _ftype, unsigned int _k, unsigned int _m, float _beta);
qdetector_mt_cccf_t * qdetector_mt_cccf_create_gmsk(unsigned char *_sequence, unsigned int _sequence_len, unsigned int _k, unsigned int _m, float _beta);
qdetector_mt_cccf_t * qdetector_mt_cccf_create_cpfsk(unsigned char *_sequence, unsigned int _sequence_len, unsigned int _bps, float _h, unsigned int _k, unsigned int _m, float _beta, int _type);
qdetector_mt_cccf_t * qdetector_mt_cccf_copy(qdetector_mt_cccf_t * q_orig);
int qdetector_mt_cccf_destroy(qdetector_mt_cccf_t *_q);
int qdetector_mt_cccf_reset(qdetector_mt_cccf_t *_q);
void * qdetector_mt_cccf_execute(qdetector_mt_cccf_t *_q, float complex _x);
float qdetector_mt_cccf_get_threshold(qdetector_mt_cccf_t *_q);
int qdetector_mt_cccf_set_threshold(qdetector_mt_cccf_t *_q, float _threshold);
float qdetector_mt_cccf_get_range(qdetector_mt_cccf_t *_q);
int qdetector_mt_cccf_set_range(qdetector_mt_cccf_t *_q, float _dphi_max);
unsigned int qdetector_mt_cccf_get_seq_len(qdetector_mt_cccf_t *_q);
const void * qdetector_mt_cccf_get_sequence(qdetector_mt_cccf_t *_q);
unsigned int qdetector_mt_cccf_get_buf_len(qdetector_mt_cccf_t *_q);
float qdetector_mt_cccf_get_rxy(qdetector_mt_cccf_t *_q);
float qdetector_mt_cccf_get_tau(qdetector_mt_cccf_t *_q);
float qdetector_mt_cccf_get_gamma(qdetector_mt_cccf_t *_q);
float qdetector_mt_cccf_get_dphi(qdetector_mt_cccf_t *_q);
float qdetector_mt_cccf_get_phi(qdetector_mt_cccf_t *_q);

#endif