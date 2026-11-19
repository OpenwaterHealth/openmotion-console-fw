/* Configuration key policy (Core/Src/motion_cfg_keys.c):
 *  - a host write naming a safety key is refused (Release/Debug) or
 *    accepted within bounds (Factory);
 *  - an operational key out of range, a non-numeric value and an unknown
 *    key are refused;
 *  - the opaque calibration block is accepted untouched;
 *  - a v2 record (the WI-00015 ten-key set plus the calibration block) is
 *    migrated by dropping the safety keys and keeping the rest verbatim;
 *  - the pass-through filter refuses safety-owned registers in Release and
 *    range-checks operational ones. */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "jsmn.h"
#include "motion_cfg_keys.h"
#include "fpga_register_map.h"
#include "safety_envelope.h"

#define NTOK 256

static int tokenize(const char *js, jsmntok_t *t)
{
    jsmn_parser p;
    jsmn_init(&p, NULL);
    return jsmn_parse(&p, js, strlen(js), t, NTOK, NULL);
}

static cfg_check_t check(const char *js, bool factory, char *why)
{
    jsmntok_t t[NTOK];
    int r = tokenize(js, t);
    assert(r > 0);
    return motion_cfg_check_tokens(js, t, r, factory, why, 72);
}

/* The record a fleet console held before this change (PTR-2026-1-5 §5.2.3
 * screenshot): the WI-00015 ten keys plus the application's calibration block. */
static const char V2_RECORD[] =
    "{\"TA_PULSE_WIDTH\":500,\"TA_CURRENT_DRV\":4200,\"SEED_CW_GAIN\":140,"
    "\"EE_PULSE_WIDTH_UL\":550,\"EE_RATE_LL\":23125,\"EE_DRIVE_CL\":4287,"
    "\"OPT_PULSE_WIDTH_UL\":550,\"OPT_RATE_LL\":23125,\"OPT_DRIVE_CL\":699,"
    "\"calibration\":{\"C_min\":[[0.36,0.0],[0.0,0.0]],\"C_max\":[[0.47,1.0]],\"I_min\":[1.2,3.4],\"I_max\":[141.3,321.0]},"
    "\"TEC_TRIP\":40.0}";

int main(void)
{
    char why[72];

    /* classification */
    assert(motion_cfg_key_rule("TA_CURRENT_DRV")->cls == CFG_KEY_OPERATIONAL);
    assert(motion_cfg_key_rule("TEC_TRIP")->cls == CFG_KEY_SAFETY);
    assert(motion_cfg_key_rule("EE_DRIVE_CL")->cls == CFG_KEY_SAFETY);
    assert(motion_cfg_key_rule("OPT_THRESH")->cls == CFG_KEY_SAFETY);
    assert(motion_cfg_key_rule("calibration")->cls == CFG_KEY_OPAQUE);
    assert(motion_cfg_key_rule("EE_STATUS") == NULL);
    assert(motion_cfg_key_rule("EE_DYNAMIC_CTRL") == NULL);
    assert(motion_cfg_key_rule("SEED_ADC_DDS_CL") == NULL);
    assert(motion_cfg_key_rule("bogus") == NULL);
#if defined(OW_FACTORY_PROGRAMMING)
    assert(motion_cfg_safety_keys_writable());
#else
    assert(!motion_cfg_safety_keys_writable());
#endif

    /* a Release host write */
    assert(check("{\"TA_CURRENT_DRV\":4200,\"TA_PULSE_WIDTH\":500,\"SEED_CW_GAIN\":140}", false, why) == CFG_CHECK_OK);
    assert(check("{}", false, why) == CFG_CHECK_OK);
    assert(check("{\"calibration\":{\"C_min\":[[1,2],[3,4]],\"x\":null}}", false, why) == CFG_CHECK_OK);
    assert(check("{\"TA_CURRENT_DRV\":4200,\"TEC_TRIP\":40}", false, why) == CFG_CHECK_SAFETY_KEY);
    assert(strcmp(why, "TEC_TRIP: safety parameter is firmware-owned") == 0);
    assert(check("{\"EE_DRIVE_CL\":4287}", false, why) == CFG_CHECK_SAFETY_KEY);
    assert(check("{\"OPT_THRESH\":7143,\"OPT_GAIN\":1.86}", false, why) == CFG_CHECK_SAFETY_KEY);
    assert(check("{\"EE_PULSE_WIDTH_UL\":550}", false, why) == CFG_CHECK_SAFETY_KEY);
    assert(check("{\"TA_CURRENT_DRV\":5001}", false, why) == CFG_CHECK_BAD_VALUE);
    assert(strcmp(why, "TA_CURRENT_DRV: out of range") == 0);
    assert(check("{\"TA_CURRENT_DRV\":-1}", false, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"TA_PULSE_WIDTH\":601}", false, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"TA_PULSE_WIDTH\":0}", false, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"SEED_CW_GAIN\":161}", false, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"SEED_DDS_GAIN\":161}", false, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"SEED_DDS_GAIN\":100}", false, why) == CFG_CHECK_OK);
    assert(check("{\"TA_CURRENT_DRV\":\"4200\"}", false, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"TA_CURRENT_DRV\":true}", false, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"TA_CURRENT_DRV\":[4200]}", false, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"EE_STATUS\":0}", false, why) == CFG_CHECK_UNKNOWN_KEY);
    assert(check("{\"bogus\":1}", false, why) == CFG_CHECK_UNKNOWN_KEY);
    assert(check("{\"calibration\":{},\"bogus\":1}", false, why) == CFG_CHECK_UNKNOWN_KEY);
    /* the first problem wins: unknown key after a valid nested block */
    assert(check("[1,2]", false, why) == CFG_CHECK_NOT_OBJECT);

    /* a Factory host write: safety keys within the hard bounds */
    assert(check("{\"TEC_TRIP\":40,\"EE_DRIVE_CL\":4287,\"EE_PULSE_WIDTH_UL\":550}", true, why) == CFG_CHECK_OK);
    assert(check("{\"TEC_TRIP\":61}", true, why) == CFG_CHECK_BAD_VALUE);
    /* the envelope can only be tightened: the trip may only move down */
    assert(check("{\"TEC_TRIP\":40.5}", true, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"TEC_TRIP\":35}", true, why) == CFG_CHECK_OK);
    assert(check("{\"TEC_TRIP\":19}", true, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"EE_DRIVE_CL\":5001}", true, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"EE_PULSE_WIDTH_UL\":661}", true, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"EE_RATE_LL\":23124}", true, why) == CFG_CHECK_BAD_VALUE);
    assert(check("{\"EE_RATE_LL\":30000}", true, why) == CFG_CHECK_OK);
    assert(check("{\"bogus\":1}", true, why) == CFG_CHECK_UNKNOWN_KEY);

    /* migration of the v2 fleet record, Release policy */
    {
        jsmntok_t t[NTOK];
        char out[2030];
        int r = tokenize(V2_RECORD, t);
        assert(r > 0);
        int dropped = motion_cfg_filter_json(V2_RECORD, t, r, false, out, sizeof out);
        assert(dropped == 7);  /* EE_PULSE_WIDTH_UL, EE_RATE_LL, EE_DRIVE_CL, OPT_*, TEC_TRIP */
        assert(strcmp(out,
            "{\"TA_PULSE_WIDTH\":500,\"TA_CURRENT_DRV\":4200,\"SEED_CW_GAIN\":140,"
            "\"calibration\":{\"C_min\":[[0.36,0.0],[0.0,0.0]],\"C_max\":[[0.47,1.0]],\"I_min\":[1.2,3.4],\"I_max\":[141.3,321.0]}}") == 0);
        /* the migrated text passes the write policy */
        assert(check(out, false, why) == CFG_CHECK_OK);
        /* Factory policy keeps the safety keys */
        dropped = motion_cfg_filter_json(V2_RECORD, t, r, true, out, sizeof out);
        assert(dropped == 0);
        assert(strcmp(out, V2_RECORD) == 0);
        /* an out-of-range operational value is dropped, not clamped */
        const char *bad = "{\"TA_CURRENT_DRV\":9000,\"TA_PULSE_WIDTH\":500,\"TEC_TRIP\":40}";
        r = tokenize(bad, t);
        dropped = motion_cfg_filter_json(bad, t, r, false, out, sizeof out);
        assert(dropped == 2 && strcmp(out, "{\"TA_PULSE_WIDTH\":500}") == 0);
        /* the old test-app config with register keys the firmware no longer accepts */
        const char *old = "{\"TEC_TRIP\": 40, \"OPT_THRESH\": 7143, \"EE_THRESH\": 5000, \"EE_GAIN\": 1.86, \"OPT_GAIN\": 1.86, "
                          "\"TA_PULSE_WIDTH\": 500, \"TA_PERIOD\": 1000, \"TA_CURRENT_DRV\": 4050, \"TA_CURRENT_LIMIT\": 4095, "
                          "\"SEED_DDS_GAIN\": 100, \"SEED_CW_GAIN\": 140, \"SEED_ADC_DDS_CL\": 100, \"EE_RATE_UL\": 30000}";
        r = tokenize(old, t);
        dropped = motion_cfg_filter_json(old, t, r, false, out, sizeof out);
        assert(dropped == 9);
        assert(strcmp(out, "{\"TA_PULSE_WIDTH\":500,\"TA_CURRENT_DRV\":4050,\"SEED_DDS_GAIN\":100,\"SEED_CW_GAIN\":140}") == 0);
        /* too small an output buffer is an error, never a truncated object */
        char tiny[40];
        assert(motion_cfg_filter_json(V2_RECORD, t, tokenize(V2_RECORD, t), false, tiny, sizeof tiny) == -1);
    }

    /* skip helper walks nested values */
    {
        jsmntok_t t[NTOK];
        const char *js = "{\"a\":{\"b\":[1,{\"c\":2}],\"d\":3},\"e\":4}";
        int r = tokenize(js, t);
        int after_a = motion_cfg_skip_token(t, r, 2);  /* value of "a" starts at token 2 */
        assert(after_a < r && t[after_a].type == JSMN_STRING);
        assert(strncmp(js + t[after_a].start, "e", 1) == 0);
        assert(motion_cfg_skip_token(t, r, r) == r);
    }

    /* pass-through filter (OW_CTRL_I2C_WR) */
    {
        uint8_t v4200[2] = { 0x8A, 0x66 };   /* 4200 mA / 0.160 = 26250 = 0x668A */
        uint8_t v5001[2] = { 0x1C, 0x7A };   /* 5001 / 0.160 = 31256 = 0x7A18 -> use 31260 */
        v5001[0] = 0x1C; v5001[1] = 0x7A;
        uint8_t one[1] = { 1 };
        uint8_t two[2] = { 1, 0 };   /* raw 1: inside every rule's range */
        /* TA_CURRENT_DRV: mux 1 ch 4 addr 0x41 off 0x06, 2 bytes */
        assert(fpga_passthrough_write_problem(1, 4, 0x41, 0x06, 2, v4200, false) == NULL);
        assert(fpga_passthrough_write_problem(1, 4, 0x41, 0x06, 2, v5001, false) != NULL);
        assert(strcmp(fpga_passthrough_write_problem(1, 4, 0x41, 0x06, 2, v5001, false), "value out of range") == 0);
        /* TA_CURRENT_LIMIT (0x08) and EE_DRIVE_CL (ch 6, 0x10): firmware-owned */
        const char *w = fpga_passthrough_write_problem(1, 4, 0x41, 0x08, 2, two, false);
        const char *e = fpga_passthrough_write_problem(1, 6, 0x41, 0x10, 2, two, false);
        const char *s = fpga_passthrough_write_problem(1, 6, 0x41, 0x20, 2, two, false); /* EE_STATIC_CTRL */
#if defined(OW_FACTORY_PROGRAMMING)
        assert(w == NULL && e == NULL && s == NULL);
#else
        assert(w != NULL && strcmp(w, "firmware-owned safety register") == 0);
        assert(e != NULL && strcmp(e, "firmware-owned safety register") == 0);
        assert(s != NULL);
#endif
        /* the fault-clear control word stays usable with the trigger stopped */
        assert(fpga_passthrough_write_problem(1, 6, 0x41, 0x22, 1, one, false) == NULL);
        assert(fpga_passthrough_write_problem(1, 6, 0x41, 0x22, 1, one, true) != NULL);
        /* read-only and status registers, unmapped offsets, other channels */
        assert(fpga_passthrough_write_problem(1, 6, 0x41, 0x24, 1, one, false) != NULL);
        assert(fpga_passthrough_write_problem(1, 4, 0x41, 0x14, 1, one, false) != NULL);
        assert(fpga_passthrough_write_problem(1, 4, 0x41, 0x05, 1, one, false) != NULL);
        assert(fpga_passthrough_write_problem(1, 1, 0x49, 0x05, 1, one, false) == NULL);
    }

    /* per-unit ceilings from the safety record lower the operating point's
     * range for host writes and the pass-through, and never raise it */
    {
        uint8_t v4200[2] = { 0x8A, 0x66 };   /* 4200 mA */
        assert(check("{\"TA_CURRENT_DRV\":4200,\"TA_PULSE_WIDTH\":500}", false, why) == CFG_CHECK_OK);
        motion_cfg_set_unit_ceiling("TA_CURRENT_DRV", 4100.0);
        motion_cfg_set_unit_ceiling("TA_PULSE_WIDTH", 480.0);
        assert(check("{\"TA_CURRENT_DRV\":4200}", false, why) == CFG_CHECK_BAD_VALUE);
        assert(strcmp(why, "TA_CURRENT_DRV: out of range") == 0);
        assert(check("{\"TA_PULSE_WIDTH\":500}", false, why) == CFG_CHECK_BAD_VALUE);
        assert(check("{\"TA_CURRENT_DRV\":4100,\"TA_PULSE_WIDTH\":480}", false, why) == CFG_CHECK_OK);
        assert(strcmp(fpga_passthrough_write_problem(1, 4, 0x41, 0x06, 2, v4200, false), "value out of range") == 0);
        /* a ceiling above the envelope bound leaves the bound */
        motion_cfg_set_unit_ceiling("TA_CURRENT_DRV", 9000.0);
        assert(motion_cfg_effective_max(motion_cfg_key_rule("TA_CURRENT_DRV")) == (double)SAFETY_TA_CURRENT_DRV_MAX_MA);
        assert(check("{\"TA_CURRENT_DRV\":5001}", false, why) == CFG_CHECK_BAD_VALUE);
        assert(fpga_passthrough_write_problem(1, 4, 0x41, 0x06, 2, v4200, false) == NULL);
        /* only operational keys take a ceiling */
        motion_cfg_set_unit_ceiling("EE_DRIVE_CL", 1.0);
        assert(motion_cfg_effective_max(motion_cfg_key_rule("EE_DRIVE_CL")) == (double)SAFETY_EE_DRIVE_CL_MAX_MA);
        motion_cfg_set_unit_ceiling("TA_PULSE_WIDTH", (double)SAFETY_TA_PULSE_WIDTH_MAX_US);
    }

    printf("motion_cfg_keys host tests OK\n");
    return 0;
}
