#include "framegen.h"

static void framegen_reconfigure(framegen_t *fg)
{
    qpacketmodem_configure(fg->payload_encoder, fg->payload_dec_len, fg->payload_props.crc, fg->payload_props.o_fec, fg->payload_props.i_fec, fg->payload_props.mod);

    fg->payload_mod_len = qpacketmodem_get_frame_len(fg->payload_encoder);
    fg->payload_mod = (float complex *)realloc(fg->payload_mod, fg->payload_mod_len * sizeof(float complex));

    if(fg->payload_props.pilots)
        qpilotgen_recreate(fg->payload_pilotgen, fg->payload_mod_len, 16);

    fg->payload_sym_len = fg->payload_props.pilots ? qpilotgen_get_frame_len(fg->payload_pilotgen) : fg->payload_mod_len;
    fg->payload_sym = (float complex *)realloc(fg->payload_sym, fg->payload_sym_len * sizeof(float complex));
}

static float complex framegen_generate_preamble(framegen_t *fg)
{
    float complex sym = fg->preamble_pn[fg->symbol_counter++];

    if(fg->symbol_counter == 64)
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

        if(fg->is_last)
        {
            fg->state = FRAMEGEN_STATE_TAIL;
        }
        else
        {
            fg->frame_complete = 1;
            fg->frame_assembled = 0;
        }
    }

    return sym;
}
static float complex framegen_generate_tail(framegen_t *fg)
{
    fg->symbol_counter++;

    if(fg->symbol_counter == 2 * fg->interp_m)
    {
        fg->symbol_counter = 0;
        fg->frame_complete = 1;
        fg->frame_assembled = 0;
    }

    return 0.0f;
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
        case FRAMEGEN_STATE_TAIL:
            return framegen_generate_tail(fg);
    }

    return 0.0f;
}


framegen_t * framegen_create(framegenprops_t *h_props, size_t h_len)
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

    fg->interp_k = 2;
    fg->interp_m = 7;
    fg->interp_beta = 0.25f;
    fg->interp = firinterp_crcf_create_prototype(LIQUID_FIRFILT_RRC, fg->interp_k, fg->interp_m, fg->interp_beta, 0);
    fg->interp_buf = (float complex *)malloc(fg->interp_k * sizeof(float complex));

    if(!fg->interp_buf)
    {
        firinterp_crcf_destroy(fg->interp);
        free(fg);

        return NULL;
    }

    fg->preamble_pn = (float complex *)malloc(64 * sizeof(float complex));

    if(!fg->preamble_pn)
    {
        free(fg->interp_buf);
        firinterp_crcf_destroy(fg->interp);
        free(fg);

        return NULL;
    }

    msequence ms = msequence_create_default(7);

    for(size_t i = 0; i < 64; i++)
    {
        fg->preamble_pn[i] = (msequence_advance(ms) ? M_SQRT1_2 : -M_SQRT1_2);
        fg->preamble_pn[i] += (msequence_advance(ms) ? M_SQRT1_2 : -M_SQRT1_2) * _Complex_I;
    }

    msequence_destroy(ms);

    memmove(&fg->header_props, h_props, sizeof(framegenprops_t));

    fg->header_user_len = h_len;
    fg->header_dec_len = 6 + h_len;
    fg->header_dec = (uint8_t *)malloc(fg->header_dec_len);

    if(!fg->header_dec)
    {
        free(fg->preamble_pn);
        free(fg->interp_buf);
        firinterp_crcf_destroy(fg->interp);
        free(fg);

        return NULL;
    }

    fg->header_encoder = qpacketmodem_create();

    qpacketmodem_configure(fg->header_encoder, fg->header_dec_len, fg->header_props.crc, fg->header_props.o_fec, fg->header_props.i_fec, fg->header_props.mod);

    fg->header_mod_len = qpacketmodem_get_frame_len(fg->header_encoder);
    fg->header_mod = (float complex *)malloc(fg->header_mod_len * sizeof(float complex));

    if(!fg->header_mod)
    {
        qpacketmodem_destroy(fg->header_encoder);
        free(fg->header_dec);
        free(fg->preamble_pn);
        free(fg->interp_buf);
        firinterp_crcf_destroy(fg->interp);
        free(fg);

        return NULL;
    }

    fg->header_pilotgen = qpilotgen_create(fg->header_mod_len, 16);
    fg->header_sym_len = fg->header_props.pilots ? qpilotgen_get_frame_len(fg->header_pilotgen) : fg->header_mod_len;
    fg->header_sym = (float complex *)malloc(fg->header_sym_len * sizeof(float complex));

    if(!fg->header_sym)
    {
        qpilotgen_destroy(fg->header_pilotgen);
        free(fg->header_mod);
        qpacketmodem_destroy(fg->header_encoder);
        free(fg->header_dec);
        free(fg->preamble_pn);
        free(fg->interp_buf);
        firinterp_crcf_destroy(fg->interp);
        free(fg);

        return NULL;
    }

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

    qpilotgen_destroy(fg->payload_pilotgen);
    qpacketmodem_destroy(fg->payload_encoder);
    qpilotgen_destroy(fg->header_pilotgen);
    qpacketmodem_destroy(fg->header_encoder);
    firinterp_crcf_destroy(fg->interp);

    free(fg->payload_sym);
    free(fg->header_sym);
    free(fg->header_mod);
    free(fg->header_dec);
    free(fg->preamble_pn);
    free(fg->interp_buf);
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

    return fg->interp_k * (64 + fg->header_sym_len + fg->payload_sym_len + (fg->is_last ? (2 * fg->interp_m) : 0));
}
void framegen_assemble(framegen_t *fg, uint8_t *header_user, uint8_t *payload, size_t payload_len, uint8_t is_last)
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
    fg->header_dec[5] = is_last ? 0x80 : 0x00;
    fg->header_dec[5] |= fg->payload_props.pilots ? 0x40 : 0x00;
    fg->header_dec[5] |= ((uint8_t)fg->payload_props.o_fec) & 0x1F;

    if(fg->header_user_len)
        memmove(fg->header_dec + 6, header_user, fg->header_user_len);

    qpacketmodem_encode(fg->header_encoder, fg->header_dec, fg->header_mod);

    if(fg->header_props.pilots)
        qpilotgen_execute(fg->header_pilotgen, fg->header_mod, fg->header_sym);
    else
        memmove(fg->header_sym, fg->header_mod, fg->header_mod_len * sizeof(float complex));

    framegen_reconfigure(fg);

    qpacketmodem_encode(fg->payload_encoder, payload, fg->payload_mod);

    if(fg->payload_props.pilots)
        qpilotgen_execute(fg->payload_pilotgen, fg->payload_mod, fg->payload_sym);
    else
        memmove(fg->payload_sym, fg->payload_mod, fg->payload_mod_len * sizeof(float complex));

    fg->is_last = is_last;
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
