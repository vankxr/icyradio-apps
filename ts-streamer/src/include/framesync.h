#ifndef __FRAMESYNC_H__
#define __FRAMESYNC_H__

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <liquid/liquid.h>
#include "framegen.h"

#define FRAMESYNC_ENABLE_EQ 0

// Synchronizer states
#define FRAMESYNC_STATE_DETECTFRAME 0
#define FRAMESYNC_STATE_RXDELAY 1
#define FRAMESYNC_STATE_RXPREAMBLE 2
#define FRAMESYNC_STATE_RXHEADER 3
#define FRAMESYNC_STATE_RXPAYLOAD 4

// Detector buffer states
#define FRAMESYNC_BUFFER_READY      0
#define FRAMESYNC_BUFFER_PROCESSING 1
#define FRAMESYNC_BUFFER_PROCESSED  2
#define FRAMESYNC_BUFFER_CONSUMED   3

typedef struct framesync_stats_t framesync_stats_t;
typedef struct framesync_detector_stats_t framesync_detector_stats_t;
typedef struct framesync_detector_buffer_t framesync_detector_buffer_t;
typedef struct framesync_t framesync_t;

typedef uint8_t (* framesync_packet_received_cb_t)(void *, const framesync_stats_t *, const uint8_t *, const uint8_t, const uint8_t *, const size_t, const uint8_t, const uint8_t);

struct framesync_stats_t
{
    float evm;      // error vector magnitude [dB]
    float rssi;     // received signal strength indicator [dB]
    float cfo;      // carrier frequency offset (f/Fs)
};
struct framesync_detector_stats_t
{
    float tau_hat;   // timing offset estimate
    float gamma_hat; // gain estimate
    float dphi_hat;  // frequency offset estimate
    float phi_hat;   // phase offset estimate
};
struct framesync_detector_buffer_t
{
    float complex *             buf;      // buffer
    size_t                      len;      // length of buffer
    uint8_t                     state;    // buffer state
    float complex *             sync_buf; // synchronized buffer
    size_t                      sync_len; // length of synchronized buffer
    size_t                      sync_pos; // index where to continue reading in the main buffer
    framesync_detector_stats_t  stats;    // detector statistics
};
struct framesync_t
{
    // callback
    framesync_packet_received_cb_t cb;         // user-defined callback function
    void *                         cb_ptr;     // userdata pointer passed to callback
    framesync_stats_t              cb_stats;   // frame statistic object (synchronizer)

    // preamble
    float complex *                preamble_pn;        // known 64-symbol p/n sequence
    float complex *                preamble_rx;        // received p/n symbols

    // frame detector objects
    size_t                         detector_n_threads; // detector thread count
    pthread_t *                    detector_thread;    // detector threads
    unsigned int                   detector_k;         // filter samples/symbol (fixed at 2)
    unsigned int                   detector_m;         // filter delay (symbols)
    float                          detector_beta;      // filter excess bandwidth factor
    pthread_mutex_t *              detector_mutex;     // mutex for each detector
    qdetector_cccf *               detector;           // pre-demod detector array (one per thread)
    pthread_mutex_t                detector_buf_mutex; // detector buffers mutex (protects the below five variables)
    pthread_cond_t                 detector_buf_cond;  // detector buffers condition variable
    framesync_detector_buffer_t *  detector_buf;       // detector buffers
    size_t                         detector_buf_len;   // detector buffer number of valid elements
    size_t                         detector_buf_head;  // detector buffer head index
    size_t                         detector_buf_tail;  // detector buffer tail index

    // timing recovery objects, states
    unsigned int                   mf_k;               // matched filter samples/symbol (fixed at 2)
    unsigned int                   mf_m;               // matched filter delay (symbols)
    float                          mf_beta;            // excess bandwidth factor
    unsigned int                   mf_npfb;            // number of filters in symsync
    firpfb_crcf                    mf;                 // matched filter decimator
    int                            mf_counter;         // matched filter output timer
    unsigned int                   mf_pfb_index;       // filterbank index

    // carrier recovery and equalization
    nco_crcf                       mixer;              // carrier frequency recovery
#if FRAMESYNC_ENABLE_EQ
    eqlms_cccf                     eq;          // equalizer (trained on p/n sequence)
#endif

    // header
    framegenprops_t                header_props;       // header properties
    size_t                         header_user_len;    // length of user-defined array
    size_t                         header_dec_len;     // length of header (decoded)
    uint8_t *                      header_dec;         // header bytes (decoded)
    qpacketmodem                   header_decoder;     // header demodulator/decoder
    modemcf                        header_demod;       // header demod (for phase recovery only)
    size_t                         header_mod_len;     // header symbols (length)
    float complex *                header_mod;         // header symbols (received)
    qpilotsync                     header_pilotsync;   // header demodulator/decoder
    size_t                         header_sym_len;     // header symbols with pilots (length)
    float complex *                header_sym;         // header symbols with pilots (received)
    uint8_t                        header_soft;        // header performs soft demod (0 : hard, 1 : soft)
    uint8_t                        header_valid;       // Is the header valid?
    uint8_t                        header_is_last;     // Is this the last frame?

    // payload
    framegenprops_t                payload_props;      // payload properties
    size_t                         payload_dec_len;    // length of payload (decoded)
    uint8_t *                      payload_dec;        // payload bytes (decoded)
    qpacketmodem                   payload_decoder;    // payload demodulator/decoder
    modemcf                        payload_demod;      // payload demod (for phase recovery only)
    size_t                         payload_mod_len;    // payload symbols (length)
    float complex *                payload_mod;        // payload symbols (received)
    qpilotsync                     payload_pilotsync;  // payload demodulator/decoder
    size_t                         payload_sym_len;    // payload symbols (length)
    float complex *                payload_sym;        // payload symbols (received)
    uint8_t                        payload_soft;       // payload performs soft demod (0 : hard, 1 : soft)
    uint8_t                        payload_valid;      // Is the payload valid?

    // status variables
    size_t                         preamble_counter;   // counter: num of p/n syms received
    size_t                         symbol_counter;     // counter: num of symbols received
    uint8_t                        state;              // receiver state
};

framesync_t *framesync_create(framegenprops_t *h_props, size_t h_len, size_t n_det_threads);
void framesync_delete(framesync_t *fs);
void framesync_reset(framesync_t *fs);
void framesync_set_callback(framesync_t *fs, framesync_packet_received_cb_t cb, void *cb_ptr);
void framesync_get_header_props(framesync_t *fs, framegenprops_t *props);
uint8_t framesync_get_header_soft_demod(framesync_t *fs);
void framesync_set_header_soft_demod(framesync_t *fs, uint8_t soft);
uint8_t framesync_get_payload_soft_demod(framesync_t *fs);
void framesync_set_payload_soft_demod(framesync_t *fs, uint8_t soft);
size_t framesync_process_samples(framesync_t *fs, float complex *buffer, size_t buffer_len);

#endif