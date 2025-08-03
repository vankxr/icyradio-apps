#ifndef __FRAMESYNC_H__
#define __FRAMESYNC_H__

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <stdio.h>
#include <stdlib.h>
#include <liquid/liquid.h>
#include "framegen.h"

// Debug mode
#define FRAMESYNC_DEBUG 100 // Every 100 frames, write debug data to file

// Synchronizer states
#define FRAMESYNC_STATE_PN_SEEK     0   // preamble search and align
#define FRAMESYNC_STATE_PN_RX_DELAY 1   // preamble rx delay
#define FRAMESYNC_STATE_HEADER      2   // header reception
#define FRAMESYNC_STATE_PAYLOAD     3   // payload reception

// Equalizer strategies
#define FRAMESYNC_EQ_STRAT_OFF          0   // equalizer disabled
#define FRAMESYNC_EQ_STRAT_CONST_MOD    1   // constant modulus
#define FRAMESYNC_EQ_STRAT_DEC_DIR      2   // decision-directed

typedef struct framesync_stats_t framesync_stats_t;
typedef struct framesync_t framesync_t;

typedef void (* framesync_signal_detected_cb_t)(void *, const framesync_stats_t *);
typedef void (* framesync_signal_lost_cb_t)(void *, const framesync_stats_t *);
typedef void (* framesync_sync_cb_t)(void *, const framesync_stats_t *);
typedef uint8_t (* framesync_packet_received_cb_t)(void *, const framesync_stats_t *, const uint8_t *, const uint8_t, const uint8_t *, const size_t, const uint8_t);

struct framesync_stats_t
{
    float evm;      // error vector magnitude [dB]
    float rssi;     // received signal strength indicator [dB]
    float cfo;      // carrier frequency offset (f/Fs)
    float phi;      // constellation rotation angle [radians]
    float rxy;      // preamble correlation peak value
};
struct framesync_t
{
    // callback
    framesync_signal_detected_cb_t sig_det_cb;         // user-defined callback function
    framesync_signal_lost_cb_t     sig_lost_cb;        // user-defined callback function
    framesync_sync_cb_t            sync_cb;            // user-defined callback function
    framesync_packet_received_cb_t pkt_rx_cb;          // user-defined callback function
    void *                         cb_ptr;             // userdata pointer passed to callback
    framesync_stats_t              cb_stats;           // frame statistic object (synchronizer)

    // carrier, timing recovery and eq objects
    agc_crcf                       agc;                // automatic gain control
    nco_crcf                       mixer;              // carrier frequency recovery
    symsync_crcf                   symsync;            // symbol synchronizer
    size_t                         symsync_n;          // symbol synchronizer decimation counter
    eqlms_cccf                     eq;                 // equalizer
    uint8_t                        eq_strat;           // equalizer strategy

    // preamble
    modemcf                        preamble_demod;     // preamble demodulator
    size_t                         preamble_sym_len;   // preamble length (symbols)
    bsequence                      preamble_pn;        // known p/n sequence
    bsequence                      preamble_rx[4];     // received p/n sequence (one for each quadrant)

    // header
    framegenprops_t                header_props;       // header properties
    size_t                         header_user_len;    // header user section length
    size_t                         header_dec_len;     // header length (decoded)
    uint8_t *                      header_dec;         // header data (decoded)
    packetizer                     header_decoder;     // header decoder
    size_t                         header_enc_len;     // header length (encoded)
    uint8_t *                      header_enc;         // header bytes (encoded)
    modemcf                        header_demod;       // header demod
    size_t                         header_mod_len;     // header length (symbols)
    float complex *                header_mod;         // header symbols
    qpilotsync                     header_pilotsync;   // header pilot synchronizer
    size_t                         header_sym_len;     // header length with pilots (symbols)
    float complex *                header_sym;         // header symbols with pilots
    uint8_t                        header_soft;        // header performs soft demod (0 : hard, 1 : soft)
    uint8_t                        header_valid;       // Is the header valid?

    // payload
    framegenprops_t                payload_props_p;    // payload properties (previous frame)
    framegenprops_t                payload_props;      // payload properties
    size_t                         payload_dec_len_p;  // payload length (decoded) (previous frame)
    size_t                         payload_dec_len;    // payload length (decoded)
    uint8_t *                      payload_dec;        // payload bytes (decoded)
    packetizer                     payload_decoder;    // payload decoder
    size_t                         payload_enc_len;    // payload length (encoded)
    uint8_t *                      payload_enc;        // payload bytes (encoded)
    modemcf                        payload_demod;      // payload demod
    size_t                         payload_mod_len;    // payload length (symbols)
    float complex *                payload_mod;        // payload symbols
    qpilotsync                     payload_pilotsync;  // payload pilot synchronizer
    size_t                         payload_sym_len;    // payload length with pilots (symbols)
    float complex *                payload_sym;        // payload symbols with pilots
    uint8_t                        payload_soft;       // payload performs soft demod (0 : hard, 1 : soft)
    uint8_t                        payload_valid;      // Is the payload valid?

    // status variables
    size_t                         symbol_counter;     // counter: num of symbols received
    size_t                         byte_counter;       // counter: num of bytes received
    size_t                         bit_counter;        // counter: num of bits received
    uint8_t                        state;              // receiver state

    // debug
#ifdef FRAMESYNC_DEBUG
    FILE *                         debug_fd;           // debug file descriptor
    size_t                         debug_counter;      // debug file counter
    size_t                         debug_fr_counter;   // debug frame counter
#endif
};

framesync_t *framesync_create(framegenprops_t *h_props, size_t h_len);
void framesync_delete(framesync_t *fs);
void framesync_reset(framesync_t *fs);
void framesync_set_callback(framesync_t *fs, framesync_signal_detected_cb_t sig_det_cb, framesync_signal_lost_cb_t sig_lost_cb, framesync_sync_cb_t sync_cb, framesync_packet_received_cb_t pkt_rx_cb, void *cb_ptr);
void framesync_get_header_props(framesync_t *fs, framegenprops_t *props);
uint8_t framesync_get_header_soft_demod(framesync_t *fs);
void framesync_set_header_soft_demod(framesync_t *fs, uint8_t soft);
uint8_t framesync_get_payload_soft_demod(framesync_t *fs);
void framesync_set_payload_soft_demod(framesync_t *fs, uint8_t soft);
uint8_t framesync_get_eq_strategy(framesync_t *fs);
void framesync_set_eq_strategy(framesync_t *fs, uint8_t strategy);
void framesync_process_samples(framesync_t *fs, float complex *buffer, size_t buffer_len);

#endif