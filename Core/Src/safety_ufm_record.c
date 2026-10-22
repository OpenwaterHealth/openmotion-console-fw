/*
 * safety_ufm_record.c - decode and validate the per-unit safety record.
 * Pure C (no HAL), so it is also built by host_tests/test_safety_ufm_record.c.
 */
#include "safety_ufm_record.h"
#include "safety_params.h"

#include <string.h>

uint16_t safety_crc16_ccitt(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static bool all_bytes(const uint8_t *raw, size_t len, uint8_t v)
{
    for (size_t i = 0; i < len; i++) {
        if (raw[i] != v) {
            return false;
        }
    }
    return true;
}

/* Every limit against the hard bounds of safety_params.h. */
static bool limits_in_bounds(const safety_ufm_limits_t *l, uint8_t device)
{
    /* The drive-current limit register has a different meaning per device:
     * EE compares the TA drive current (mA), OPT the optical monitor. */
    uint16_t drive_cl_max = (device == 3U) ? (uint16_t)SAFETY_OPT_DRIVE_CL_MAX
                                           : (uint16_t)SAFETY_EE_OPT_DRIVE_CL_MA;
    if (l->pulse_width_ul_us > SAFETY_EE_OPT_PULSE_WIDTH_UL_US) { return false; }
    if (l->rate_ll_us < SAFETY_EE_OPT_RATE_LL_MIN_US)           { return false; }
    if (l->drive_cl_ma == 0U || l->drive_cl_ma > drive_cl_max)  { return false; }
    if (l->pwm_current_ma > SAFETY_EE_OPT_PWM_CURRENT_MA)       { return false; }
    if (l->cw_current_ma > SAFETY_EE_OPT_CW_CURRENT_MA)         { return false; }
    if (l->ta_current_max_ma > SAFETY_TA_CURRENT_DRV_MAX_MA)    { return false; }
    if (l->ta_pulse_max_us > SAFETY_TA_PULSE_WIDTH_MAX_US)      { return false; }
    return true;
}

safety_ufm_status_t safety_ufm_record_parse(const uint8_t *raw, size_t len,
                                            uint8_t device, safety_ufm_limits_t *out)
{
    if (raw == NULL || len < SAFETY_UFM_RECORD_SIZE) {
        return SAFETY_UFM_BAD_MAGIC;
    }
    if (all_bytes(raw, SAFETY_UFM_RECORD_SIZE, 0x00u) || all_bytes(raw, SAFETY_UFM_RECORD_SIZE, 0xFFu)) {
        return SAFETY_UFM_BLANK;
    }
    if (rd32(&raw[0]) != SAFETY_UFM_RECORD_MAGIC) {
        return SAFETY_UFM_BAD_MAGIC;
    }
    if (raw[4] != SAFETY_UFM_RECORD_LAYOUT) {
        return SAFETY_UFM_BAD_LAYOUT;
    }
    /* Checksum before the field checks: a corrupt record is reported as
     * corrupt, not as out of bounds. */
    if (safety_crc16_ccitt(0xFFFFu, raw, 30U) != rd16(&raw[30])) {
        return SAFETY_UFM_BAD_CHECKSUM;
    }
    if (raw[5] != device) {
        return SAFETY_UFM_BAD_DEVICE;
    }
    if (!all_bytes(&raw[22], 8U, 0x00u)) {
        return SAFETY_UFM_BAD_RESERVED;
    }

    safety_ufm_limits_t l;
    l.pulse_width_ul_us = rd16(&raw[6]);
    l.rate_ll_us        = rd32(&raw[8]);
    l.drive_cl_ma       = rd16(&raw[12]);
    l.pwm_current_ma    = rd16(&raw[14]);
    l.cw_current_ma     = rd16(&raw[16]);
    l.ta_current_max_ma = rd16(&raw[18]);
    l.ta_pulse_max_us   = rd16(&raw[20]);
    if (!limits_in_bounds(&l, device)) {
        return SAFETY_UFM_OUT_OF_BOUNDS;
    }
    if (out != NULL) {
        *out = l;
    }
    return SAFETY_UFM_VALID;
}

const char *safety_ufm_status_name(safety_ufm_status_t s)
{
    switch (s) {
    case SAFETY_UFM_BLANK:         return "blank";
    case SAFETY_UFM_VALID:         return "valid";
    case SAFETY_UFM_BAD_MAGIC:     return "bad magic";
    case SAFETY_UFM_BAD_LAYOUT:    return "unknown layout";
    case SAFETY_UFM_BAD_DEVICE:    return "wrong device";
    case SAFETY_UFM_BAD_CHECKSUM:  return "bad checksum";
    case SAFETY_UFM_BAD_RESERVED:  return "reserved bytes set";
    case SAFETY_UFM_OUT_OF_BOUNDS: return "limit out of bounds";
    default:                       return "unknown";
    }
}

void safety_ufm_record_encode(const safety_ufm_limits_t *in, uint8_t device,
                              uint8_t out[SAFETY_UFM_RECORD_SIZE])
{
    memset(out, 0, SAFETY_UFM_RECORD_SIZE);
    wr32(&out[0], SAFETY_UFM_RECORD_MAGIC);
    out[4] = SAFETY_UFM_RECORD_LAYOUT;
    out[5] = device;
    wr16(&out[6], in->pulse_width_ul_us);
    wr32(&out[8], in->rate_ll_us);
    wr16(&out[12], in->drive_cl_ma);
    wr16(&out[14], in->pwm_current_ma);
    wr16(&out[16], in->cw_current_ma);
    wr16(&out[18], in->ta_current_max_ma);
    wr16(&out[20], in->ta_pulse_max_us);
    wr16(&out[30], safety_crc16_ccitt(0xFFFFu, out, 30U));
}
