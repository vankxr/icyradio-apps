#ifndef __FRAMEGEN_H__
#define __FRAMEGEN_H__

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <liquid/liquid.h>

// Generator states
#define FRAMEGEN_STATE_PREAMBLE 0
#define FRAMEGEN_STATE_HEADER 1
#define FRAMEGEN_STATE_PAYLOAD 2
#define FRAMEGEN_STATE_TAIL 3

typedef struct framegenprops_t framegenprops_t;
typedef struct framegen_t framegen_t;

struct framegenprops_t
{
    crc_scheme        crc;    // data validity check
    fec_scheme        i_fec;  // forward error-correction scheme (inner)
    fec_scheme        o_fec;  // forward error-correction scheme (outer)
    modulation_scheme mod;    // modulation scheme
    uint8_t           pilots; // use pilots (1) or not (0)
};
struct framegen_t
{
    // interpolator
    unsigned int    interp_k;           // interp samples/symbol (fixed at 2)
    unsigned int    interp_m;           // interp filter delay (symbols)
    float           interp_beta;        // excess bandwidth factor
    firinterp_crcf  interp;             // interpolator object
    float complex * interp_buf;         // output interpolator buffer

    // preamble
    float complex * preamble_pn;        // p/n sequence

    // header
    framegenprops_t header_props;       // header properties
    size_t          header_user_len;    // header user section length
    size_t          header_dec_len;     // header length (decoded)
    uint8_t *       header_dec;         // header data (decoded)
    qpacketmodem    header_encoder;     // header encoder/modulator
    size_t          header_mod_len;     // header length (encoded/modulated)
    float complex * header_mod;         // header symbols (encoded/modulated)
    qpilotgen       header_pilotgen;    // header pilot symbol generator
    size_t          header_sym_len;     // header length (pilots added)
    float complex * header_sym;         // header symbols (pilots added)

    // payload
    framegenprops_t payload_props;      // payload properties
    size_t          payload_dec_len;    // payload length (decoded)
    qpacketmodem    payload_encoder;    // payload encoder/modulator
    size_t          payload_mod_len;    // payload length (encoded/modulated)
    float complex * payload_mod;        // payload symbols (encoded/modulated)
    qpilotgen       payload_pilotgen;   // payload pilot symbol generator
    size_t          payload_sym_len;    // payload length (pilots added)
    float complex * payload_sym;        // payload symbols (pilots added)

    // counters/states
    size_t          symbol_counter;     // output symbol number
    size_t          sample_counter;     // output sample number
    uint8_t         is_last;            // whether this is the last frame or not (send tail symbols or not)
    uint8_t         frame_assembled;    // frame assembled flag
    uint8_t         frame_complete;     // frame completed flag
    uint8_t         state;              // generator state
};

framegen_t *framegen_create(framegenprops_t *h_props, size_t h_len);
void framegen_delete(framegen_t *fg);
void framegen_reset(framegen_t *fg);
uint8_t framegen_is_assembled(framegen_t *fg);
void framegen_get_header_props(framegen_t *fg, framegenprops_t *props);
void framegen_get_payload_props(framegen_t *fg, framegenprops_t *props);
void framegen_set_payload_props(framegen_t *fg, framegenprops_t *props);
size_t framegen_get_symbol_count(framegen_t *fg);
void framegen_assemble(framegen_t *fg, uint8_t *header_user, uint8_t *payload, size_t payload_len, uint8_t is_last);
size_t framegen_write_samples(framegen_t *fg, float complex *buffer, size_t buffer_len);

#endif