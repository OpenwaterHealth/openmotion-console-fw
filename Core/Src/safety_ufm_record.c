/*
 * safety_ufm_record.c - decode and validate the per-unit safety record.
 * Pure C (no HAL), so it is also built by host_tests/test_safety_ufm_record.c.
 */
#include "safety_ufm_record.h"
#include "safety_envelope.h"

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

uint16_t safety_ufm_checksum(const uint8_t *raw)
{
    uint16_t sum = 0U;
    for (size_t i = 0; i < 30U; i++) {
        sum = (uint16_t)(sum + raw[i]);
    }
    return sum;
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

bool safety_ufm_limits_in_envelope(const safety_ufm_limits_t *l, uint8_t device)
{
    /* The drive-current limit register has a different meaning per device:
     * EE compares the TA drive current (mA), OPT the optical monitor. */
    double drive_cl_max = (device == SAFETY_UFM_DEVICE_OPT) ? (double)SAFETY_OPT_DRIVE_CL_MAX
                                                            : (double)SAFETY_EE_DRIVE_CL_MAX_MA;
    if (l->pulse_width_ll > l->pulse_width_ul) { return false; }
    if ((double)l->pulse_width_ul * SAFETY_FPGA_TIME_LSB_US > (double)SAFETY_EE_OPT_PULSE_WIDTH_UL_MAX_US) { return false; }
    if ((double)l->rate_ll * SAFETY_FPGA_TIME_LSB_US < (double)SAFETY_EE_OPT_RATE_LL_MIN_US)               { return false; }
    if (l->drive_cl == 0U || (double)l->drive_cl * SAFETY_FPGA_DRIVE_LSB > drive_cl_max)                   { return false; }
    if ((double)l->pwm_current * SAFETY_FPGA_CURRENT_LSB_MA > (double)SAFETY_EE_OPT_PWM_CURRENT_MAX_MA)    { return false; }
    if ((double)l->cw_current * SAFETY_FPGA_CURRENT_LSB_MA > (double)SAFETY_EE_OPT_CW_CURRENT_MAX_MA)      { return false; }
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
    if (raw[0] != SAFETY_UFM_RECORD_MAGIC0 || raw[1] != SAFETY_UFM_RECORD_MAGIC1) {
        return SAFETY_UFM_BAD_MAGIC;
    }
    /* Checksum before the field checks: a corrupt record is reported as
     * corrupt, not as out of bounds. */
    if (safety_ufm_checksum(raw) != rd16(&raw[30])) {
        return SAFETY_UFM_BAD_CHECKSUM;
    }
    if (raw[2] != SAFETY_UFM_RECORD_LAYOUT) {
        return SAFETY_UFM_BAD_LAYOUT;
    }
    if (raw[3] != 0x00u || !all_bytes(&raw[22], 8U, 0x00u)) {
        return SAFETY_UFM_BAD_RESERVED;
    }

    safety_ufm_limits_t l;
    l.pulse_width_ll = rd32(&raw[4]);
    l.pulse_width_ul = rd32(&raw[8]);
    l.rate_ll        = rd32(&raw[12]);
    l.drive_cl       = rd16(&raw[16]);
    l.pwm_current    = rd16(&raw[18]);
    l.cw_current     = rd16(&raw[20]);
    if (!safety_ufm_limits_in_envelope(&l, device)) {
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
    case SAFETY_UFM_BLANK:           return "blank";
    case SAFETY_UFM_VALID:           return "valid";
    case SAFETY_UFM_BAD_MAGIC:       return "bad magic";
    case SAFETY_UFM_BAD_LAYOUT:      return "unknown layout";
    case SAFETY_UFM_LOADER_FALLBACK: return "FPGA loader on defaults";
    case SAFETY_UFM_BAD_CHECKSUM:    return "bad checksum";
    case SAFETY_UFM_BAD_RESERVED:    return "reserved bytes set";
    case SAFETY_UFM_OUT_OF_BOUNDS:   return "limit outside envelope";
    default:                         return "unknown";
    }
}

bool safety_ufm_status_is_fault(safety_ufm_status_t s)
{
#if defined(OW_FACTORY_PROGRAMMING)
    if (s == SAFETY_UFM_BLANK) {
        return false;
    }
#endif
    return s != SAFETY_UFM_VALID;
}

void safety_ufm_record_encode(const safety_ufm_limits_t *in, uint8_t out[SAFETY_UFM_RECORD_SIZE])
{
    memset(out, 0, SAFETY_UFM_RECORD_SIZE);
    out[0] = SAFETY_UFM_RECORD_MAGIC0;
    out[1] = SAFETY_UFM_RECORD_MAGIC1;
    out[2] = SAFETY_UFM_RECORD_LAYOUT;
    wr32(&out[4], in->pulse_width_ll);
    wr32(&out[8], in->pulse_width_ul);
    wr32(&out[12], in->rate_ll);
    wr16(&out[16], in->drive_cl);
    wr16(&out[18], in->pwm_current);
    wr16(&out[20], in->cw_current);
    wr16(&out[30], safety_ufm_checksum(out));
}
