#include "framegen.h"

static void framegen_reconfigure(framegen_t *fg)
{
    if(fg->payload_dec_len != fg->payload_dec_len_p || fg->payload_props.crc != fg->payload_props_p.crc || fg->payload_props.i_fec != fg->payload_props_p.i_fec || fg->payload_props.o_fec != fg->payload_props_p.o_fec || fg->payload_props.mod != fg->payload_props_p.mod)
    {
        qpacketmodem_configure(fg->payload_encoder, fg->payload_dec_len, fg->payload_props.crc, fg->payload_props.o_fec, fg->payload_props.i_fec, fg->payload_props.mod);

        fg->payload_mod_len = qpacketmodem_get_frame_len(fg->payload_encoder);
        fg->payload_mod = (float complex *)realloc(fg->payload_mod, fg->payload_mod_len * sizeof(float complex));
    }

    if(fg->payload_dec_len != fg->payload_dec_len_p || fg->payload_props.pilots != fg->payload_props_p.pilots)
    {

        if(fg->payload_props.pilots)
        {
            qpilotgen_recreate(fg->payload_pilotgen, fg->payload_mod_len, 16);

            fg->payload_sym_len = qpilotgen_get_frame_len(fg->payload_pilotgen);
        }
        else
        {
            fg->payload_sym_len = fg->payload_mod_len;
        }

        fg->payload_sym = (float complex *)realloc(fg->payload_sym, fg->payload_sym_len * sizeof(float complex));
    }

    fg->payload_props_p.crc = fg->payload_props.crc;
    fg->payload_props_p.i_fec = fg->payload_props.i_fec;
    fg->payload_props_p.o_fec = fg->payload_props.o_fec;
    fg->payload_props_p.mod = fg->payload_props.mod;
    fg->payload_props_p.pilots = fg->payload_props.pilots;
    fg->payload_dec_len_p = fg->payload_dec_len;
}

static float complex framegen_generate_preamble(framegen_t *fg)
{
    float complex sym = fg->preamble_pn[fg->symbol_counter++];

    if(fg->symbol_counter == fg->preamble_pn_len)
    {
        fg->symbol_counter = 0;
        fg->state = FRAMEGEN_STATE_HEADER;
    }

    return sym;
}
static float complex framegen_generate_header(framegen_t *fg)
{
    float complex sym = fg->header_sym[fg->symbol_counter++];

    if(fg->symbol_counter == fg->header_sym_len)
    {
        fg->symbol_counter = 0;
        fg->state = FRAMEGEN_STATE_PAYLOAD;
    }

    return sym;
}
static float complex framegen_generate_payload(framegen_t *fg)
{
    float complex sym = fg->payload_sym[fg->symbol_counter++];

    // check state
    if(fg->symbol_counter == fg->payload_sym_len)
    {
        fg->symbol_counter = 0;
        fg->frame_complete = 1;
        fg->frame_assembled = 0;
    }

    return sym;
}
static float complex framegen_generate_symbol(framegen_t *fg)
{
    if(!fg->frame_assembled)
        return 0.0f;

    switch(fg->state)
    {
        case FRAMEGEN_STATE_PREAMBLE:
            return framegen_generate_preamble(fg);
        case FRAMEGEN_STATE_HEADER:
            return framegen_generate_header(fg);
        case FRAMEGEN_STATE_PAYLOAD:
            return framegen_generate_payload(fg);
    }

    return 0.0f;
}


framegen_t *framegen_create(framegenprops_t *h_props, size_t h_len)
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

    framegen_t *fg = (framegen_t *)malloc(sizeof(framegen_t));

    if(!fg)
        return NULL;

    // interpolator
    fg->interp_k = 2;
    fg->interp = firinterp_crcf_create_prototype(LIQUID_FIRFILT_RRC, fg->interp_k, 15, 0.2f, 0);
    firinterp_crcf_set_scale(fg->interp, 0.5f);
    fg->interp_buf = (float complex *)malloc(fg->interp_k * sizeof(float complex));

    // preamble
    msequence pn_seq = msequence_create_default(6);
    modemcf pn_mod = modemcf_create(LIQUID_MODEM_QPSK); // Can be any modem, however QPSK is the lowest MI that allows detecting a rotated constellation

    fg->preamble_pn_len = ceilf((float)msequence_get_length(pn_seq) / (float)modem_get_bps(pn_mod));
    fg->preamble_pn = (float complex *)malloc(fg->preamble_pn_len * sizeof(float complex));

    for(size_t i = 0; i < fg->preamble_pn_len; i++)
        modemcf_modulate(pn_mod, msequence_generate_symbol(pn_seq, modem_get_bps(pn_mod)), &fg->preamble_pn[i]);

    modemcf_destroy(pn_mod);
    msequence_destroy(pn_seq);

    // header
    memmove(&fg->header_props, h_props, sizeof(framegenprops_t));

    fg->header_user_len = h_len;
    fg->header_dec_len = 6 + h_len;
    fg->header_dec = (uint8_t *)malloc(fg->header_dec_len);
    fg->header_encoder = qpacketmodem_create();

    qpacketmodem_configure(fg->header_encoder, fg->header_dec_len, fg->header_props.crc, fg->header_props.o_fec, fg->header_props.i_fec, fg->header_props.mod);

    fg->header_mod_len = fg->header_props.pilots ? qpacketmodem_get_frame_len(fg->header_encoder) : 0;
    fg->header_mod = fg->header_props.pilots ? (float complex *)malloc(fg->header_mod_len * sizeof(float complex)) : NULL;
    fg->header_pilotgen = fg->header_props.pilots ? qpilotgen_create(fg->header_mod_len, 16) : NULL;
    fg->header_sym_len = fg->header_props.pilots ? qpilotgen_get_frame_len(fg->header_pilotgen) : qpacketmodem_get_frame_len(fg->header_encoder);
    fg->header_sym = (float complex *)malloc(fg->header_sym_len * sizeof(float complex));

    // payload
    fg->payload_props_p.crc = LIQUID_CRC_NONE;
    fg->payload_props_p.i_fec = LIQUID_FEC_NONE;
    fg->payload_props_p.o_fec = LIQUID_FEC_NONE;
    fg->payload_props_p.mod = LIQUID_MODEM_QPSK;
    fg->payload_props_p.pilots = 0;
    fg->payload_props.crc = LIQUID_CRC_NONE;
    fg->payload_props.i_fec = LIQUID_FEC_NONE;
    fg->payload_props.o_fec = LIQUID_FEC_NONE;
    fg->payload_props.mod = LIQUID_MODEM_QPSK;
    fg->payload_props.pilots = 0;
    fg->payload_dec_len_p = 0;
    fg->payload_dec_len = 0;
    fg->payload_encoder = qpacketmodem_create();
    fg->payload_mod_len = 0;
    fg->payload_mod = NULL;
    fg->payload_pilotgen = qpilotgen_create(128, 64);
    fg->payload_sym_len = 0;
    fg->payload_sym = NULL;

    framegen_reset(fg);

    return fg;
}
void framegen_delete(framegen_t *fg)
{
    if(!fg)
        return;

    // payload
    if(fg->payload_sym)
        free(fg->payload_sym);

    qpilotgen_destroy(fg->payload_pilotgen);

    if(fg->payload_mod)
        free(fg->payload_mod);

    qpacketmodem_destroy(fg->payload_encoder);

    // header
    free(fg->header_sym);

    if(fg->header_pilotgen)
        qpilotgen_destroy(fg->header_pilotgen);

    if(fg->header_mod)
        free(fg->header_mod);

    qpacketmodem_destroy(fg->header_encoder);
    free(fg->header_dec);

    // preamble
    free(fg->preamble_pn);

    // interpolator
    free(fg->interp_buf);
    firinterp_crcf_destroy(fg->interp);

    free(fg);
}
void framegen_reset(framegen_t *fg)
{
    if(!fg)
        return;

    fg->symbol_counter = 0;
    fg->sample_counter = 0;
    fg->frame_assembled = 0;
    fg->frame_complete = 0;

    fg->state = FRAMEGEN_STATE_PREAMBLE;
}
uint8_t framegen_is_assembled(framegen_t *fg)
{
    if(!fg)
        return 0;

    return fg->frame_assembled;
}
void framegen_get_header_props(framegen_t *fg, framegenprops_t *props)
{
    if(!fg || !props)
        return;

    memmove(props, &fg->header_props, sizeof(framegenprops_t));
}
void framegen_get_payload_props(framegen_t *fg, framegenprops_t *props)
{
    if(!fg || !props)
        return;

    memmove(props, &fg->payload_props, sizeof(framegenprops_t));
}
void framegen_set_payload_props(framegen_t *fg, framegenprops_t *props)
{
    if(!fg || !props)
        return;

    if(fg->frame_assembled)
        return;

    if(props->crc == LIQUID_CRC_UNKNOWN || props->crc >= LIQUID_CRC_NUM_SCHEMES)
        return;

    if(props->i_fec == LIQUID_FEC_UNKNOWN || props->i_fec >= LIQUID_FEC_NUM_SCHEMES)
        return;

    if(props->o_fec == LIQUID_FEC_UNKNOWN || props->o_fec >= LIQUID_FEC_NUM_SCHEMES)
        return;

    if(props->mod == LIQUID_MODEM_UNKNOWN || props->mod >= LIQUID_MODEM_NUM_SCHEMES)
        return;

    memmove(&fg->payload_props, props, sizeof(framegenprops_t));
}
size_t framegen_get_symbol_count(framegen_t *fg)
{
    if(!fg)
        return 0;

    return fg->preamble_pn_len + fg->header_sym_len + fg->payload_sym_len;
}
size_t framegen_get_sample_count(framegen_t *fg)
{
    if(!fg)
        return 0;

    return fg->interp_k * framegen_get_symbol_count(fg);
}
void framegen_assemble(framegen_t *fg, uint8_t *header_user, uint8_t *payload, size_t payload_len)
{
    if(!fg)
        return;

    if(fg->frame_assembled)
        return;

    if(fg->header_user_len && !header_user)
        return;

    if(payload_len && !payload)
        return;

    framegen_reset(fg);

    fg->payload_dec_len = payload_len;

    fg->header_dec[0] = 0x5A;
    fg->header_dec[1] = (fg->payload_dec_len >> 8) & 0xFF;
    fg->header_dec[2] = (fg->payload_dec_len >> 0) & 0xFF;
    fg->header_dec[3] = (uint8_t)fg->payload_props.mod;
    fg->header_dec[4] = ((uint8_t)fg->payload_props.crc & 0x07) << 5;
    fg->header_dec[4] |= ((uint8_t)fg->payload_props.i_fec) & 0x1F;
    fg->header_dec[5] = fg->payload_props.pilots ? 0x80 : 0x00;
    fg->header_dec[5] |= ((uint8_t)fg->payload_props.o_fec) & 0x1F;

    if(fg->header_user_len)
        memmove(fg->header_dec + 6, header_user, fg->header_user_len);

    qpacketmodem_encode(fg->header_encoder, fg->header_dec, fg->header_props.pilots ? fg->header_mod : fg->header_sym);

    if(fg->header_props.pilots)
        qpilotgen_execute(fg->header_pilotgen, fg->header_mod, fg->header_sym);

    framegen_reconfigure(fg);

    qpacketmodem_encode(fg->payload_encoder, payload, fg->payload_props.pilots ? fg->payload_mod : fg->payload_sym);

    if(fg->payload_props.pilots)
        qpilotgen_execute(fg->payload_pilotgen, fg->payload_mod, fg->payload_sym);

    fg->frame_assembled = 1;
}
size_t framegen_write_samples(framegen_t *fg, float complex *buffer, size_t buffer_len)
{
    if(!fg || !buffer)
        return 0;

    if(!fg->frame_assembled)
        return 0;

    for(size_t i = 0; i < buffer_len; i++)
    {
        if(fg->sample_counter == 0)
        {
            if(fg->frame_complete)
                return i;

            firinterp_crcf_execute(fg->interp, framegen_generate_symbol(fg), fg->interp_buf);
        }

        buffer[i] = fg->interp_buf[fg->sample_counter];

        fg->sample_counter = (fg->sample_counter + 1) % fg->interp_k;
    }

    return buffer_len;
}
