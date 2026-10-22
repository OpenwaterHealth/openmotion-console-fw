/*
 * safety_params.c - the firmware-owned register loads.
 *
 * Builds the ordered list of laser-driver and safety-FPGA registers the
 * firmware writes and reads back at boot and after every configuration
 * apply (motion_cfg_apply_settings, main.c), from the constants in
 * safety_params.h and, where a safety FPGA holds a valid per-unit UFM record,
 * from that record. Pure C: built by the host tests too.
 *
 * Raw register values: the same conversion apply_fpga_register_setting()
 * uses for host values (engineering value / scale, rounded), except that the
 * design-reset registers carry their raw reset value verbatim so a rounding
 * difference can never produce a read-back mismatch.
 */
#include "safety_params.h"
#include "safety_ufm_record.h"
#include "fpga_register_map.h"

#include <string.h>

uint32_t safety_eng_to_raw(const char *reg, double eng)
{
    const FpgaRegister *r = fpga_register_lookup(reg);
    double scale = (r != NULL && r->scale != 0.0f) ? (double)r->scale : 1.0;
    double scaled = eng / scale + 0.5;
    if (scaled < 0.0) {
        return 0U;
    }
    if (scaled > 4294967295.0) {
        return 0xFFFFFFFFu;
    }
    return (uint32_t)scaled;
}

/* One safety FPGA (EE = "EE_", OPT = "OPT_"), per-unit record or ratings. */
static size_t safety_fpga_loads(const char *prefix, const safety_ufm_limits_t *unit,
                                safety_reg_load_t *out, size_t max, size_t n)
{
    struct { const char *suffix; double eng; } rows[6];
    rows[0].suffix = "PULSE_WIDTH_LL"; rows[0].eng = (double)SAFETY_EE_OPT_PULSE_WIDTH_LL_US;
    rows[1].suffix = "PULSE_WIDTH_UL"; rows[1].eng = unit ? (double)unit->pulse_width_ul_us : (double)SAFETY_EE_OPT_PULSE_WIDTH_UL_US;
    rows[2].suffix = "RATE_LL";        rows[2].eng = unit ? (double)unit->rate_ll_us        : (double)SAFETY_EE_OPT_RATE_LL_US;
    rows[3].suffix = "DRIVE_CL";       rows[3].eng = unit ? (double)unit->drive_cl_ma       : (double)SAFETY_EE_OPT_DRIVE_CL_MA;
    rows[4].suffix = "PWM_CURRENT";    rows[4].eng = unit ? (double)unit->pwm_current_ma    : (double)SAFETY_EE_OPT_PWM_CURRENT_MA;
    rows[5].suffix = "CW_CURRENT";     rows[5].eng = unit ? (double)unit->cw_current_ma     : (double)SAFETY_EE_OPT_CW_CURRENT_MA;

    for (size_t i = 0; i < 6U && n < max; i++) {
        char name[32];
        (void)strcpy(name, prefix);
        (void)strcat(name, rows[i].suffix);
        const FpgaRegister *r = fpga_register_lookup(name);
        if (r == NULL) {
            continue;
        }
        out[n].reg = NULL; /* set below from the table's own string */
        /* fpga_register_lookup() returns the register; the table owns the
         * name string, so find it again by pointer-stable lookup. */
        out[n].reg = fpga_register_name(r);
        out[n].raw = safety_eng_to_raw(name, rows[i].eng);
        out[n].per_unit = (unit != NULL) && (i >= 1U && i <= 5U);
        n++;
    }
    return n;
}

size_t safety_reg_loads(const safety_ufm_limits_t *ee, const safety_ufm_limits_t *opt,
                        safety_reg_load_t *out, size_t max)
{
    size_t n = 0;
    /* Load order: the limits of the two safety monitors first, then the
     * driver clamps. The operational point (TA current, pulse width, seed
     * gain) is loaded afterwards by the configuration apply. */
    n = safety_fpga_loads("EE_",  ee,  out, max, n);
    n = safety_fpga_loads("OPT_", opt, out, max, n);

    static const struct { const char *reg; uint32_t raw; } design[] = {
        { "TA_CURRENT_LIMIT", 0x8647u  },  /* openmotion-ta-fpga registers.v reset: 5500 mA */
        { "TA_PWM_MON_CL",    0x00b0u  },  /* registers.v reset */
        { "TA_CW_MON_CL",     0x00a0u  },  /* registers.v reset */
        { "TA_PERIOD",        0x006e00u}, /* registers.v reset */
        { "SEED_DDS_CL",      0x03dcu  },  /* openmotion-seed-fpga registers.v reset: 80 mA */
        { "SEED_CW_CL",       0x0800u  },  /* registers.v reset: 160 mA */
    };
    for (size_t i = 0; i < sizeof(design) / sizeof(design[0]) && n < max; i++) {
        const FpgaRegister *r = fpga_register_lookup(design[i].reg);
        if (r == NULL) {
            continue;
        }
        out[n].reg = fpga_register_name(r);
        out[n].raw = design[i].raw;
        out[n].per_unit = false;
        n++;
    }
    return n;
}

uint32_t safety_ta_current_drv_max_ma(const safety_ufm_limits_t *ee, const safety_ufm_limits_t *opt)
{
    uint32_t m = SAFETY_TA_CURRENT_DRV_MAX_MA;
    if (ee != NULL && ee->ta_current_max_ma != 0U && ee->ta_current_max_ma < m) {
        m = ee->ta_current_max_ma;
    }
    if (opt != NULL && opt->ta_current_max_ma != 0U && opt->ta_current_max_ma < m) {
        m = opt->ta_current_max_ma;
    }
    return m;
}

uint32_t safety_ta_pulse_width_max_us(const safety_ufm_limits_t *ee, const safety_ufm_limits_t *opt)
{
    uint32_t m = SAFETY_TA_PULSE_WIDTH_MAX_US;
    if (ee != NULL && ee->ta_pulse_max_us != 0U && ee->ta_pulse_max_us < m) {
        m = ee->ta_pulse_max_us;
    }
    if (opt != NULL && opt->ta_pulse_max_us != 0U && opt->ta_pulse_max_us < m) {
        m = opt->ta_pulse_max_us;
    }
    return m;
}
