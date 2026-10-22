/* Safety FPGA UFM calibration record (Core/Src/safety_ufm_record.c):
 * good record, bad checksum, out-of-range limit, blank UFM, wrong device,
 * unknown layout, reserved bytes; and the firmware-owned load table
 * (Core/Src/safety_params.c) with and without per-unit records. */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "safety_params.h"
#include "safety_ufm_record.h"
#include "fpga_register_map.h"

static safety_ufm_limits_t good(void)
{
    safety_ufm_limits_t l;
    memset(&l, 0, sizeof l);
    l.pulse_width_ul_us = 550;     /* 500 us x 1.1, WI-00015 */
    l.rate_ll_us = 23125;
    l.drive_cl_ma = 4287;          /* a measured x 1.1 value seen on a fleet unit */
    l.pwm_current_ma = 100;
    l.cw_current_ma = 100;
    l.ta_current_max_ma = 4500;
    l.ta_pulse_max_us = 600;
    return l;
}

static const safety_reg_load_t *find_load(const safety_reg_load_t *loads, size_t n, const char *reg)
{
    for (size_t i = 0; i < n; i++) {
        if (strcmp(loads[i].reg, reg) == 0) {
            return &loads[i];
        }
    }
    return NULL;
}

int main(void)
{
    uint8_t raw[SAFETY_UFM_RECORD_SIZE];
    safety_ufm_limits_t out;

    /* good record for EE (device 2) */
    safety_ufm_limits_t g = good();
    safety_ufm_record_encode(&g, 2, raw);
    assert(safety_ufm_record_parse(raw, sizeof raw, 2, &out) == SAFETY_UFM_VALID);
    assert(out.drive_cl_ma == 4287 && out.pulse_width_ul_us == 550 && out.rate_ll_us == 23125);
    assert(out.ta_current_max_ma == 4500 && out.ta_pulse_max_us == 600);

    /* wrong device: the EE record read from the OPT device */
    assert(safety_ufm_record_parse(raw, sizeof raw, 3, &out) == SAFETY_UFM_BAD_DEVICE);

    /* bad checksum: one flipped bit in a limit */
    {
        uint8_t bad[SAFETY_UFM_RECORD_SIZE];
        memcpy(bad, raw, sizeof bad);
        bad[12] ^= 0x01;
        assert(safety_ufm_record_parse(bad, sizeof bad, 2, &out) == SAFETY_UFM_BAD_CHECKSUM);
        /* and the checksum byte itself */
        memcpy(bad, raw, sizeof bad);
        bad[31] ^= 0x80;
        assert(safety_ufm_record_parse(bad, sizeof bad, 2, &out) == SAFETY_UFM_BAD_CHECKSUM);
    }

    /* out-of-range limits, each with a valid checksum */
    {
        safety_ufm_limits_t l = good();
        l.drive_cl_ma = SAFETY_EE_OPT_DRIVE_CL_MA + 1;
        safety_ufm_record_encode(&l, 2, raw);
        assert(safety_ufm_record_parse(raw, sizeof raw, 2, &out) == SAFETY_UFM_OUT_OF_BOUNDS);
        /* the OPT bound is wider: the same value is valid for device 3 */
        safety_ufm_record_encode(&l, 3, raw);
        assert(safety_ufm_record_parse(raw, sizeof raw, 3, &out) == SAFETY_UFM_VALID);
        l = good();
        l.drive_cl_ma = 0;
        safety_ufm_record_encode(&l, 2, raw);
        assert(safety_ufm_record_parse(raw, sizeof raw, 2, &out) == SAFETY_UFM_OUT_OF_BOUNDS);
        l = good();
        l.pulse_width_ul_us = SAFETY_EE_OPT_PULSE_WIDTH_UL_US + 1;
        safety_ufm_record_encode(&l, 2, raw);
        assert(safety_ufm_record_parse(raw, sizeof raw, 2, &out) == SAFETY_UFM_OUT_OF_BOUNDS);
        l = good();
        l.rate_ll_us = SAFETY_EE_OPT_RATE_LL_MIN_US - 1;   /* would allow a higher rate */
        safety_ufm_record_encode(&l, 2, raw);
        assert(safety_ufm_record_parse(raw, sizeof raw, 2, &out) == SAFETY_UFM_OUT_OF_BOUNDS);
        l = good();
        l.ta_current_max_ma = SAFETY_TA_CURRENT_DRV_MAX_MA + 1;
        safety_ufm_record_encode(&l, 2, raw);
        assert(safety_ufm_record_parse(raw, sizeof raw, 2, &out) == SAFETY_UFM_OUT_OF_BOUNDS);
        l = good();
        l.pwm_current_ma = SAFETY_EE_OPT_PWM_CURRENT_MA + 1;
        safety_ufm_record_encode(&l, 2, raw);
        assert(safety_ufm_record_parse(raw, sizeof raw, 2, &out) == SAFETY_UFM_OUT_OF_BOUNDS);
    }

    /* blank UFM: erased (0x00) or never programmed (0xFF) */
    memset(raw, 0x00, sizeof raw);
    assert(safety_ufm_record_parse(raw, sizeof raw, 2, &out) == SAFETY_UFM_BLANK);
    memset(raw, 0xFF, sizeof raw);
    assert(safety_ufm_record_parse(raw, sizeof raw, 3, &out) == SAFETY_UFM_BLANK);

    /* junk: not blank, not a record */
    memset(raw, 0x5A, sizeof raw);
    assert(safety_ufm_record_parse(raw, sizeof raw, 2, &out) == SAFETY_UFM_BAD_MAGIC);

    /* unknown layout and reserved bytes */
    g = good();
    safety_ufm_record_encode(&g, 2, raw);
    raw[4] = 2;
    raw[30] = 0; raw[31] = 0;
    {
        uint16_t c = safety_crc16_ccitt(0xFFFFu, raw, 30);
        raw[30] = (uint8_t)c; raw[31] = (uint8_t)(c >> 8);
    }
    assert(safety_ufm_record_parse(raw, sizeof raw, 2, &out) == SAFETY_UFM_BAD_LAYOUT);
    safety_ufm_record_encode(&g, 2, raw);
    raw[25] = 1;
    {
        uint16_t c = safety_crc16_ccitt(0xFFFFu, raw, 30);
        raw[30] = (uint8_t)c; raw[31] = (uint8_t)(c >> 8);
    }
    assert(safety_ufm_record_parse(raw, sizeof raw, 2, &out) == SAFETY_UFM_BAD_RESERVED);

    /* short buffer */
    assert(safety_ufm_record_parse(raw, 16, 2, &out) == SAFETY_UFM_BAD_MAGIC);

    /* ---- the load table: ratings when no record, per-unit when valid ---- */
    safety_reg_load_t loads[SAFETY_REG_LOADS_MAX];
    size_t n = safety_reg_loads(NULL, NULL, loads, SAFETY_REG_LOADS_MAX);
    assert(n == 18);  /* 6 EE + 6 OPT + 6 design-reset registers */
    const safety_reg_load_t *l;
    l = find_load(loads, n, "EE_DRIVE_CL");
    assert(l != NULL && l->raw == 0x0a80 && !l->per_unit);      /* 5000 mA = registers.v reset */
    l = find_load(loads, n, "OPT_PWM_CURRENT");
    assert(l != NULL && l->raw == 0x036b);                       /* 140 mA = registers.v reset */
    l = find_load(loads, n, "EE_PULSE_WIDTH_UL");
    assert(l != NULL && l->raw == 2063);                         /* 660 us / 0.32 */
    l = find_load(loads, n, "EE_RATE_LL");
    assert(l != NULL && l->raw == 72266);                        /* 23125 us / 0.32 */
    l = find_load(loads, n, "TA_CURRENT_LIMIT");
    assert(l != NULL && l->raw == 0x8647);
    l = find_load(loads, n, "TA_PERIOD");
    assert(l != NULL && l->raw == 0x006e00);
    l = find_load(loads, n, "SEED_CW_CL");
    assert(l != NULL && l->raw == 0x0800);
    /* never written by the firmware: not a design register of the released seed design */
    assert(find_load(loads, n, "SEED_ADC_DDS_CL") == NULL);
    /* the operating point is never in the safety loads */
    assert(find_load(loads, n, "TA_CURRENT_DRV") == NULL);
    assert(find_load(loads, n, "SEED_CW_GAIN") == NULL);

    g = good();
    n = safety_reg_loads(&g, NULL, loads, SAFETY_REG_LOADS_MAX);
    l = find_load(loads, n, "EE_DRIVE_CL");
    assert(l != NULL && l->per_unit && l->raw == (uint32_t)(4287.0 / 1.86 + 0.5));
    l = find_load(loads, n, "EE_PULSE_WIDTH_UL");
    assert(l != NULL && l->per_unit && l->raw == (uint32_t)(550.0 / 0.32 + 0.5));
    l = find_load(loads, n, "OPT_DRIVE_CL");
    assert(l != NULL && !l->per_unit && l->raw == 0x0a80);       /* OPT has no record: rating */
    assert(safety_ta_current_drv_max_ma(&g, NULL) == 4500);
    assert(safety_ta_current_drv_max_ma(NULL, NULL) == SAFETY_TA_CURRENT_DRV_MAX_MA);
    assert(safety_ta_pulse_width_max_us(&g, NULL) == 600);
    g.ta_current_max_ma = 0;
    assert(safety_ta_current_drv_max_ma(&g, NULL) == SAFETY_TA_CURRENT_DRV_MAX_MA);

    printf("safety_ufm_record host tests OK\n");
    return 0;
}
