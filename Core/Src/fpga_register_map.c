#include "fpga_register_map.h"
#include <string.h>

/* Flat register map keyed by friendlyName.
   muxIdx, channel, and i2cAddr are placeholders – update as needed. */

typedef struct {
    const char   *name;
    FpgaRegister  reg;
} FpgaRegisterEntry;

static const FpgaRegisterEntry fpga_register_table[] = {

    /* ── TA ──────────────────────────────────────────────────────────────── */
    { "TA_PULSE_WIDTH",    { 1, 4, 3, 0x41, 0x00, FPGA_DIR_RW, 0.320f } },
    { "TA_PERIOD",         { 1, 4, 3, 0x41, 0x03, FPGA_DIR_RW, 0.320f } },
    { "TA_CURRENT_DRV",    { 1, 4, 2, 0x41, 0x06, FPGA_DIR_RW, 0.160f } },
    { "TA_CURRENT_LIMIT",  { 1, 4, 2, 0x41, 0x08, FPGA_DIR_RW, 0.160f } },
    { "TA_PWM_MON_CL",     { 1, 4, 2, 0x41, 0x0A, FPGA_DIR_RW, 0.500f } },
    { "TA_CW_MON_CL",      { 1, 4, 2, 0x41, 0x0C, FPGA_DIR_RW, 0.500f } },
    { "TA_TEMP_SENSOR",    { 1, 4, 2, 0x41, 0x0E, FPGA_DIR_RW, 1.0f   } },
    { "TA_TRIGGER_COUNT",  { 1, 4, 4, 0x41, 0x10, FPGA_DIR_RD, 1.0f   } },
    { "TA_REVISION",       { 1, 4, 1, 0x41, 0x14, FPGA_DIR_RD, 1.0f   } },
    { "TA_MINOR",          { 1, 4, 1, 0x41, 0x15, FPGA_DIR_RD, 1.0f   } },
    { "TA_MAJOR",          { 1, 4, 1, 0x41, 0x16, FPGA_DIR_RD, 1.0f   } },
    { "TA_ID",             { 1, 4, 1, 0x41, 0x17, FPGA_DIR_RD, 1.0f   } },
    { "TA_STATIC_CTL",     { 1, 4, 2, 0x41, 0x20, FPGA_DIR_RW, 1.0f   } },
    { "TA_DYNAMIC_CTL",    { 1, 4, 2, 0x41, 0x22, FPGA_DIR_RW, 1.0f   } },
    { "TA_STATUS",         { 1, 4, 1, 0x41, 0x24, FPGA_DIR_RW, 1.0f   } },

    /* ── Seed ────────────────────────────────────────────────────────────── */
    { "SEED_DDS_CTRL",     { 1, 5, 2, 0x41, 0x00, FPGA_DIR_RW, 1.0f    } },
    { "SEED_DDS_GAIN",     { 1, 5, 2, 0x41, 0x02, FPGA_DIR_RW, 0.064f  } },
    { "SEED_CW_GAIN",      { 1, 5, 2, 0x41, 0x04, FPGA_DIR_RW, 0.0688f } },
    { "SEED_DDS_CL",       { 1, 5, 2, 0x41, 0x06, FPGA_DIR_RW, 0.081f  } },
    { "SEED_CW_CL",        { 1, 5, 2, 0x41, 0x08, FPGA_DIR_RW, 0.079f  } },
    { "SEED_ADC_DDS_CL",   { 1, 5, 2, 0x41, 0x0A, FPGA_DIR_RW, 0.500f  } },
    { "SEED_ADC_CW_CL",    { 1, 5, 2, 0x41, 0x0C, FPGA_DIR_RW, 0.500f  } },
    { "SEED_ADC_CD",       { 1, 5, 2, 0x41, 0x0E, FPGA_DIR_RD, 0.500f  } },
    { "SEED_ADC_VD",       { 1, 5, 2, 0x41, 0x10, FPGA_DIR_RD, 0.500f  } },
    { "SEED_STATUS",       { 1, 5, 1, 0x41, 0x12, FPGA_DIR_RD, 1.0f    } },
    { "SEED_REVISION",     { 1, 5, 1, 0x41, 0x13, FPGA_DIR_RD, 1.0f    } },
    { "SEED_MINOR",        { 1, 5, 1, 0x41, 0x14, FPGA_DIR_RD, 1.0f    } },
    { "SEED_MAJOR",        { 1, 5, 1, 0x41, 0x15, FPGA_DIR_RD, 1.0f    } },
    { "SEED_ID",           { 1, 5, 1, 0x41, 0x16, FPGA_DIR_RD, 1.0f    } },
    { "SEED_STATIC_CTRL",  { 1, 5, 2, 0x41, 0x20, FPGA_DIR_RW, 1.0f    } },
    { "SEED_DYNAMIC_CTRL", { 1, 5, 2, 0x41, 0x22, FPGA_DIR_WR, 1.0f    } },
    /* ── Safety EE ───────────────────────────────────────────────────────── */
    { "EE_PULSE_WIDTH_LL", { 1, 6, 4, 0x41, 0x00, FPGA_DIR_RW, 0.320f  } },
    { "EE_PULSE_WIDTH_UL", { 1, 6, 4, 0x41, 0x04, FPGA_DIR_RW, 0.320f  } },
    { "EE_RATE_LL",        { 1, 6, 4, 0x41, 0x08, FPGA_DIR_RW, 0.320f  } },
    { "EE_RATE_UL",        { 1, 6, 4, 0x41, 0x0C, FPGA_DIR_RW, 0.320f  } },
    { "EE_DRIVE_CL",       { 1, 6, 2, 0x41, 0x10, FPGA_DIR_RW, 1.86f   } },
    { "EE_THRESH",         { 1, 6, 2, 0x41, 0x10, FPGA_DIR_RW, 1.86f   } },
    { "EE_PWM_CURRENT",    { 1, 6, 2, 0x41, 0x12, FPGA_DIR_RW, 0.160f  } },
    { "EE_CW_CURRENT",     { 1, 6, 2, 0x41, 0x14, FPGA_DIR_RW, 0.160f  } },
    { "EE_PWM_MONITOR_CL", { 1, 6, 2, 0x41, 0x16, FPGA_DIR_RW, 0.025f  } },
    { "EE_CW_MONITOR_CL",  { 1, 6, 2, 0x41, 0x18, FPGA_DIR_RW, 0.025f  } },
    { "EE_TEMP_SENSOR",    { 1, 6, 2, 0x41, 0x1A, FPGA_DIR_RW, 1.0f    } },
    { "EE_ADC_DATA",       { 1, 6, 2, 0x41, 0x1C, FPGA_DIR_RD, 2.500f  } },
    { "EE_STATIC_CTRL",    { 1, 6, 2, 0x41, 0x20, FPGA_DIR_RW, 1.0f    } },
    { "EE_DYNAMIC_CTRL",   { 1, 6, 2, 0x41, 0x22, FPGA_DIR_WR, 1.0f    } },
    { "EE_STATUS",         { 1, 6, 1, 0x41, 0x24, FPGA_DIR_RW, 1.0f    } },
    { "EE_REVISION",       { 1, 6, 1, 0x41, 0x25, FPGA_DIR_RD, 1.0f    } },
    { "EE_MINOR",          { 1, 6, 1, 0x41, 0x26, FPGA_DIR_RD, 1.0f    } },
    { "EE_MAJOR",          { 1, 6, 1, 0x41, 0x27, FPGA_DIR_RD, 1.0f    } },
    { "EE_ID",             { 1, 6, 1, 0x41, 0x28, FPGA_DIR_RD, 1.0f    } },

    /* ── Safety OPT ──────────────────────────────────────────────────────── */
    { "OPT_PULSE_WIDTH_LL", { 1, 7, 4, 0x41, 0x00, FPGA_DIR_RW, 0.320f  } },
    { "OPT_PULSE_WIDTH_UL", { 1, 7, 4, 0x41, 0x04, FPGA_DIR_RW, 0.320f  } },
    { "OPT_RATE_LL",        { 1, 7, 4, 0x41, 0x08, FPGA_DIR_RW, 0.320f  } },
    { "OPT_RATE_UL",        { 1, 7, 4, 0x41, 0x0C, FPGA_DIR_RW, 0.320f  } },
    { "OPT_DRIVE_CL",       { 1, 7, 2, 0x41, 0x10, FPGA_DIR_RW, 1.86f   } },
    { "OPT_THRESH",         { 1, 7, 2, 0x41, 0x10, FPGA_DIR_RW, 1.86f   } },
    { "OPT_PWM_CURRENT",    { 1, 7, 2, 0x41, 0x12, FPGA_DIR_RW, 0.160f  } },
    { "OPT_CW_CURRENT",     { 1, 7, 2, 0x41, 0x14, FPGA_DIR_RW, 0.160f  } },
    { "OPT_PWM_MONITOR_CL", { 1, 7, 2, 0x41, 0x16, FPGA_DIR_RW, 0.025f  } },
    { "OPT_CW_MONITOR_CL",  { 1, 7, 2, 0x41, 0x18, FPGA_DIR_RW, 0.025f  } },
    { "OPT_TEMP_SENSOR",    { 1, 7, 2, 0x41, 0x1A, FPGA_DIR_RW, 1.0f    } },
    { "OPT_ADC_DATA",       { 1, 7, 2, 0x41, 0x1C, FPGA_DIR_RD, 2.500f  } },
    { "OPT_STATIC_CTRL",    { 1, 7, 2, 0x41, 0x20, FPGA_DIR_RW, 1.0f    } },
    { "OPT_DYNAMIC_CTRL",   { 1, 7, 2, 0x41, 0x22, FPGA_DIR_WR, 1.0f    } },
    { "OPT_STATUS",         { 1, 7, 1, 0x41, 0x24, FPGA_DIR_RW, 1.0f    } },
    { "OPT_REVISION",       { 1, 7, 1, 0x41, 0x25, FPGA_DIR_RD, 1.0f    } },
    { "OPT_MINOR",          { 1, 7, 1, 0x41, 0x26, FPGA_DIR_RD, 1.0f    } },
    { "OPT_MAJOR",          { 1, 7, 1, 0x41, 0x27, FPGA_DIR_RD, 1.0f    } },
    { "OPT_ID",             { 1, 7, 1, 0x41, 0x28, FPGA_DIR_RD, 1.0f    } },
};

#define FPGA_REGISTER_TABLE_SIZE \
    (sizeof(fpga_register_table) / sizeof(fpga_register_table[0]))

/* True when any mapped register lives on this mux channel. */
static bool fpga_channel_is_mapped(int mux, int channel)
{
    for (size_t i = 0; i < FPGA_REGISTER_TABLE_SIZE; i++) {
        if (fpga_register_table[i].reg.muxIdx == mux &&
            fpga_register_table[i].reg.channel == channel) {
            return true;
        }
    }
    return false;
}

/* The mapped register at this location, or NULL. Aliases (EE/OPT_THRESH share
   DRIVE_CL's offset) have the same layout, so the first match is used. */
static const FpgaRegisterEntry *fpga_register_at(int mux, int channel,
                                                 uint8_t i2c_addr, uint8_t offset)
{
    for (size_t i = 0; i < FPGA_REGISTER_TABLE_SIZE; i++) {
        const FpgaRegister *r = &fpga_register_table[i].reg;
        if (r->muxIdx == mux && r->channel == channel &&
            r->i2cAddr == i2c_addr && r->offset == offset) {
            return &fpga_register_table[i];
        }
    }
    return NULL;
}

const char *fpga_passthrough_write_problem(int mux, int channel, uint8_t i2c_addr,
                                           uint8_t offset, uint8_t len,
                                           bool trigger_running)
{
    if (!fpga_channel_is_mapped(mux, channel)) {
        return NULL;
    }
    const FpgaRegisterEntry *e = fpga_register_at(mux, channel, i2c_addr, offset);
    if (e == NULL) {
        return "not a mapped FPGA register";
    }
    if (e->reg.direction == FPGA_DIR_RD) {
        return "read-only register";
    }
    if (strstr(e->name, "STATUS") != NULL) {
        return "status register";
    }
    if (strstr(e->name, "DYNAMIC") != NULL) {
        if (trigger_running) {
            return "control register needs the trigger stopped";
        }
        return (len >= 1U && len <= e->reg.data_len) ? NULL : "length does not match the register";
    }
    return (len == e->reg.data_len) ? NULL : "length does not match the register";
}

const FpgaRegister *fpga_register_lookup(const char *name)
{
    if (!name) return NULL;
    for (size_t i = 0; i < FPGA_REGISTER_TABLE_SIZE; i++) {
        if (strcmp(fpga_register_table[i].name, name) == 0) {
            return &fpga_register_table[i].reg;
        }
    }
    return NULL;
}
