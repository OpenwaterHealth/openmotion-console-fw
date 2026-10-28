/* Safety FPGA UFM safety record (Core/Src/safety_ufm_record.c), in the layout
 * the safety FPGA's own loader reads (openmotion-safety-fpga ufm_config.v):
 * good record, the loader's reference record, bad checksum, limits outside
 * the envelope, blank UFM, unknown layout, reserved bytes; the
 * firmware-owned load table and the operating-point ceilings
 * (Core/Src/safety_envelope.c) with and without per-unit records; and the
 * envelope LSBs against the register map. */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "safety_envelope.h"
#include "safety_ufm_record.h"
#include "fpga_register_map.h"

#define EE  SAFETY_UFM_DEVICE_EE
#define OPT SAFETY_UFM_DEVICE_OPT

static safety_ufm_limits_t good(void)
{
    safety_ufm_limits_t l;
    memset(&l, 0, sizeof l);
    l.pulse_width_ll = 0;
    l.pulse_width_ul = 1718;       /* 549.76 us: 500 us x 1.1, WI-00015, rounded down */
    l.rate_ll = 72266;             /* 23125.12 us */
    l.drive_cl = 2304;             /* 4285.4 mA: a measured x 1.1 value seen on a fleet unit */
    l.pwm_current = 875;           /* 140 mA, registers.v reset */
    l.cw_current = 875;
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

static void reseal(uint8_t *raw)
{
    uint16_t c = safety_ufm_checksum(raw);
    raw[30] = (uint8_t)c;
    raw[31] = (uint8_t)(c >> 8);
}

static safety_ufm_status_t parse_limits(const safety_ufm_limits_t *l, uint8_t dev)
{
    uint8_t raw[SAFETY_UFM_RECORD_SIZE];
    safety_ufm_limits_t out;
    safety_ufm_record_encode(l, raw);
    return safety_ufm_record_parse(raw, sizeof raw, dev, &out);
}

static void lsb_matches(const char *reg, double lsb)
{
    const FpgaRegister *r = fpga_register_lookup(reg);
    assert(r != NULL && (double)r->scale > lsb - 1e-6 && (double)r->scale < lsb + 1e-6);
}

int main(void)
{
    uint8_t raw[SAFETY_UFM_RECORD_SIZE];
    safety_ufm_limits_t out;

    /* The reference record of openmotion-safety-fpga tools/make_ufm_record.py
     * with its defaults (the registers.v reset values) - byte for byte the
     * layout ufm_config.v reads. Its rate_lower_limit 0x112a9 (22500 us) is
     * below the firmware envelope, so the firmware refuses it: the record a
     * unit carries must come from WI-00015, not from the tool defaults. */
    static const uint8_t ref[SAFETY_UFM_RECORD_SIZE] = {
        0x4F, 0x57, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5C, 0x03, 0x00, 0x00, 0xA9, 0x12, 0x01, 0x00,
        0x80, 0x0A, 0x6B, 0x03, 0x6B, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    memcpy(raw, ref, sizeof raw);
    reseal(raw);
    assert(raw[30] == 0x28 && raw[31] == 0x03);    /* sum of bytes 0..29 = 0x0328, as the FPGA tool computes */
    assert(safety_ufm_record_parse(raw, sizeof raw, EE, &out) == SAFETY_UFM_OUT_OF_BOUNDS);
    {
        safety_ufm_limits_t r = { 0, 0x35C, 0x112A9, 0x0A80, 0x036B, 0x036B };
        uint8_t enc[SAFETY_UFM_RECORD_SIZE];
        safety_ufm_record_encode(&r, enc);
        assert(memcmp(enc, raw, sizeof enc) == 0);  /* encode == the FPGA tool's bytes */
        r.rate_ll = 72266;                           /* the same record at the WI-00015 rate limit */
        assert(parse_limits(&r, EE) == SAFETY_UFM_VALID);
    }

    /* good record */
    safety_ufm_limits_t g = good();
    safety_ufm_record_encode(&g, raw);
    assert(safety_ufm_record_parse(raw, sizeof raw, EE, &out) == SAFETY_UFM_VALID);
    assert(out.drive_cl == 2304 && out.pulse_width_ul == 1718 && out.rate_ll == 72266);
    assert(out.pulse_width_ll == 0 && out.pwm_current == 875 && out.cw_current == 875);
    assert(safety_ufm_record_parse(raw, sizeof raw, OPT, &out) == SAFETY_UFM_VALID);

    /* bad checksum: one flipped bit in a limit, and in the checksum itself */
    {
        uint8_t bad[SAFETY_UFM_RECORD_SIZE];
        memcpy(bad, raw, sizeof bad);
        bad[16] ^= 0x01;
        assert(safety_ufm_record_parse(bad, sizeof bad, EE, &out) == SAFETY_UFM_BAD_CHECKSUM);
        memcpy(bad, raw, sizeof bad);
        bad[31] ^= 0x80;
        assert(safety_ufm_record_parse(bad, sizeof bad, EE, &out) == SAFETY_UFM_BAD_CHECKSUM);
    }

    /* limits outside the envelope, each with a valid checksum */
    {
        safety_ufm_limits_t l = good();
        l.drive_cl = 2689;                              /* 5001.5 mA > EE 5000 */
        assert(parse_limits(&l, EE) == SAFETY_UFM_OUT_OF_BOUNDS);
        assert(parse_limits(&l, OPT) == SAFETY_UFM_VALID); /* the OPT bound is wider */
        l.drive_cl = 3871;                              /* 7200.06 > OPT 7200 */
        assert(parse_limits(&l, OPT) == SAFETY_UFM_OUT_OF_BOUNDS);
        l = good(); l.drive_cl = 0;
        assert(parse_limits(&l, EE) == SAFETY_UFM_OUT_OF_BOUNDS);
        l = good(); l.pulse_width_ul = 2063;            /* 660.16 us > 660 */
        assert(parse_limits(&l, EE) == SAFETY_UFM_OUT_OF_BOUNDS);
        l.pulse_width_ul = 2062;                        /* 659.84 us: inside */
        assert(parse_limits(&l, EE) == SAFETY_UFM_VALID);
        l = good(); l.rate_ll = 72265;                  /* 23124.8 us: would allow a higher rate */
        assert(parse_limits(&l, EE) == SAFETY_UFM_OUT_OF_BOUNDS);
        l = good(); l.pwm_current = 876;                /* 140.16 mA */
        assert(parse_limits(&l, EE) == SAFETY_UFM_OUT_OF_BOUNDS);
        l = good(); l.cw_current = 876;
        assert(parse_limits(&l, OPT) == SAFETY_UFM_OUT_OF_BOUNDS);
        l = good(); l.pulse_width_ll = l.pulse_width_ul + 1U;   /* LL above UL */
        assert(parse_limits(&l, EE) == SAFETY_UFM_OUT_OF_BOUNDS);
        /* tighter than the envelope is always accepted */
        l = good(); l.drive_cl = 1; l.pulse_width_ul = 1; l.rate_ll = 0xFFFFFFFFu; l.pwm_current = 0; l.cw_current = 0;
        assert(parse_limits(&l, EE) == SAFETY_UFM_VALID);
    }

    /* blank UFM: erased (0x00) or never programmed (0xFF) */
    memset(raw, 0x00, sizeof raw);
    assert(safety_ufm_record_parse(raw, sizeof raw, EE, &out) == SAFETY_UFM_BLANK);
    memset(raw, 0xFF, sizeof raw);
    assert(safety_ufm_record_parse(raw, sizeof raw, OPT, &out) == SAFETY_UFM_BLANK);

    /* junk: not blank, not a record */
    memset(raw, 0x5A, sizeof raw);
    assert(safety_ufm_record_parse(raw, sizeof raw, EE, &out) == SAFETY_UFM_BAD_MAGIC);
    /* a record of the earlier firmware-only layout ("OWSR", CRC-16) starts
     * with the same two magic bytes; it fails the byte-sum checksum */
    memcpy(raw, "OWSR", 4);
    assert(safety_ufm_record_parse(raw, sizeof raw, EE, &out) == SAFETY_UFM_BAD_CHECKSUM);

    /* unknown layout and reserved bytes (byte 3 and 22..29) */
    safety_ufm_record_encode(&g, raw);
    raw[2] = 2;
    reseal(raw);
    assert(safety_ufm_record_parse(raw, sizeof raw, EE, &out) == SAFETY_UFM_BAD_LAYOUT);
    safety_ufm_record_encode(&g, raw);
    raw[25] = 1;
    reseal(raw);
    assert(safety_ufm_record_parse(raw, sizeof raw, EE, &out) == SAFETY_UFM_BAD_RESERVED);
    safety_ufm_record_encode(&g, raw);
    raw[3] = 1;
    reseal(raw);
    assert(safety_ufm_record_parse(raw, sizeof raw, EE, &out) == SAFETY_UFM_BAD_RESERVED);

    /* short buffer; *out untouched on failure */
    memset(&out, 0xA5, sizeof out);
    assert(safety_ufm_record_parse(raw, 16, EE, &out) == SAFETY_UFM_BAD_MAGIC);
    assert(out.drive_cl == 0xA5A5);

    /* ---- envelope LSBs = register map scales (the record is raw) ---- */
    lsb_matches("EE_PULSE_WIDTH_UL", SAFETY_FPGA_TIME_LSB_US);
    lsb_matches("OPT_RATE_LL", SAFETY_FPGA_TIME_LSB_US);
    lsb_matches("EE_DRIVE_CL", SAFETY_FPGA_DRIVE_LSB);
    lsb_matches("OPT_DRIVE_CL", SAFETY_FPGA_DRIVE_LSB);
    lsb_matches("EE_PWM_CURRENT", SAFETY_FPGA_CURRENT_LSB_MA);
    lsb_matches("OPT_CW_CURRENT", SAFETY_FPGA_CURRENT_LSB_MA);
    {
        const FpgaRegister *r = fpga_register_lookup("EE_UFM_STATUS");
        assert(r != NULL && r->offset == 0x31 && r->direction == FPGA_DIR_RD);
        r = fpga_register_lookup("OPT_UFM_STATUS");
        assert(r != NULL && r->offset == 0x31 && r->channel == 7);
    }

    /* ---- the load table ---- */
    safety_reg_load_t loads[SAFETY_REG_LOADS_MAX];
    const safety_reg_load_t *l;
    /* no records: no EE/OPT limit loads at all (design reset values stay,
     * the attestation interlock keeps the laser off); the fixed clamps only */
    size_t n = safety_reg_loads(NULL, NULL, loads, SAFETY_REG_LOADS_MAX);
    assert(n == 6);
    assert(find_load(loads, n, "EE_DRIVE_CL") == NULL && find_load(loads, n, "OPT_PULSE_WIDTH_UL") == NULL);
    l = find_load(loads, n, "TA_CURRENT_LIMIT");
    assert(l != NULL && l->raw == 0x8647 && !l->per_unit);
    l = find_load(loads, n, "TA_PERIOD");
    assert(l != NULL && l->raw == 0x006e00);
    l = find_load(loads, n, "SEED_CW_CL");
    assert(l != NULL && l->raw == 0x0800);
    /* never written by the firmware: not a design register of the released seed design */
    assert(find_load(loads, n, "SEED_ADC_DDS_CL") == NULL);
    /* the operating point is never in the safety loads */
    assert(find_load(loads, n, "TA_CURRENT_DRV") == NULL);
    assert(find_load(loads, n, "SEED_CW_GAIN") == NULL);

    /* EE record only: the six EE registers verbatim from the record */
    g = good();
    n = safety_reg_loads(&g, NULL, loads, SAFETY_REG_LOADS_MAX);
    assert(n == 12);
    l = find_load(loads, n, "EE_DRIVE_CL");
    assert(l != NULL && l->per_unit && l->raw == 2304);
    l = find_load(loads, n, "EE_PULSE_WIDTH_UL");
    assert(l != NULL && l->per_unit && l->raw == 1718);
    l = find_load(loads, n, "EE_RATE_LL");
    assert(l != NULL && l->raw == 72266);
    l = find_load(loads, n, "EE_PULSE_WIDTH_LL");
    assert(l != NULL && l->raw == 0);
    assert(find_load(loads, n, "OPT_DRIVE_CL") == NULL);
    n = safety_reg_loads(&g, &g, loads, SAFETY_REG_LOADS_MAX);
    assert(n == 18);

    /* ---- operating-point ceilings: the record can only lower them ---- */
    assert(safety_ta_pulse_width_max_us(NULL, NULL) == (double)SAFETY_TA_PULSE_WIDTH_MAX_US);
    {
        double p = safety_ta_pulse_width_max_us(&g, NULL);     /* 1718 x 0.32 = 549.76 */
        assert(p > 549.7 && p < 549.8);
        safety_ufm_limits_t o = good();
        o.pulse_width_ul = 1500;                               /* 480 us: the smaller wins */
        p = safety_ta_pulse_width_max_us(&g, &o);
        assert(p > 479.9 && p < 480.1);
        o.pulse_width_ul = 2062;                               /* 659.84 us: never above 600 */
        assert(safety_ta_pulse_width_max_us(&o, &o) == (double)SAFETY_TA_PULSE_WIDTH_MAX_US);
    }

    printf("safety_ufm_record host tests OK\n");
    return 0;
}
