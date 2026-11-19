/*
 * safety_envelope.c - the firmware-owned register loads.
 *
 * Builds the ordered list of laser-driver and safety-FPGA registers the
 * firmware writes and reads back at boot and after every configuration
 * apply (motion_cfg_apply_settings, main.c): the EE/OPT limits from a valid
 * per-unit UFM record, and the fixed driver clamps of safety_envelope.h.
 * Pure C: built by the host tests too.
 *
 * Raw register values: record values and the design-reset clamps are
 * written verbatim, so a rounding difference can never produce a read-back
 * mismatch. safety_eng_to_raw() is the conversion apply_fpga_register_setting()
 * uses for host values (engineering value / scale, rounded).
 */
#include "safety_envelope.h"
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

static size_t add_load(const char *name, uint32_t raw, bool per_unit,
                       safety_reg_load_t *out, size_t max, size_t n)
{
    const FpgaRegister *r = fpga_register_lookup(name);
    if (r == NULL || n >= max) {
        return n;
    }
    /* The table owns the name string (static lifetime). */
    out[n].reg = fpga_register_name(r);
    out[n].raw = raw;
    out[n].per_unit = per_unit;
    return n + 1U;
}

/* One safety FPGA (EE = "EE_", OPT = "OPT_") from its per-unit record. */
static size_t safety_fpga_loads(const char *prefix, const safety_ufm_limits_t *unit,
                                safety_reg_load_t *out, size_t max, size_t n)
{
    if (unit == NULL) {
        return n;
    }
    const struct { const char *suffix; uint32_t raw; } rows[] = {
        { "PULSE_WIDTH_LL", unit->pulse_width_ll },
        { "PULSE_WIDTH_UL", unit->pulse_width_ul },
        { "RATE_LL",        unit->rate_ll },
        { "DRIVE_CL",       unit->drive_cl },
        { "PWM_CURRENT",    unit->pwm_current },
        { "CW_CURRENT",     unit->cw_current },
    };
    for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        char name[32];
        (void)strcpy(name, prefix);
        (void)strcat(name, rows[i].suffix);
        n = add_load(name, rows[i].raw, true, out, max, n);
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
    for (size_t i = 0; i < sizeof(design) / sizeof(design[0]); i++) {
        n = add_load(design[i].reg, design[i].raw, false, out, max, n);
    }
    return n;
}

double safety_ta_pulse_width_max_us(const safety_ufm_limits_t *ee, const safety_ufm_limits_t *opt)
{
    double m = (double)SAFETY_TA_PULSE_WIDTH_MAX_US;
    const safety_ufm_limits_t *units[2] = { ee, opt };
    for (size_t i = 0; i < 2U; i++) {
        if (units[i] != NULL) {
            double unit = (double)units[i]->pulse_width_ul * SAFETY_FPGA_TIME_LSB_US;
            if (unit < m) {
                m = unit;
            }
        }
    }
    return m;
}
