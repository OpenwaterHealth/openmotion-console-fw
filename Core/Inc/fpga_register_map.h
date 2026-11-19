#ifndef FPGA_REGISTER_MAP_H
#define FPGA_REGISTER_MAP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FPGA_DIR_RD = 0,
    FPGA_DIR_WR = 1,
    FPGA_DIR_RW = 2
} FpgaDirection_e;

typedef struct {
    int      muxIdx;
    int      channel;
    uint8_t  data_len; /* bytes */
    uint8_t  i2cAddr;
    uint8_t  offset;
    uint8_t  direction; /* one of FPGA_DIR_* stored as uint8_t */
    float    scale;     /* conversion scale (default 1.0) */
} FpgaRegister;

/* Returns a pointer to the FpgaRegister for the given friendly name,
   or NULL if not found. */
const FpgaRegister *fpga_register_lookup(const char *name);

/* The table's own (static-lifetime) name string of a register returned by
   fpga_register_lookup(), or NULL for a pointer not from the table. */
const char *fpga_register_name(const FpgaRegister *reg);

/* Host I2C pass-through write filter (OW_CTRL_I2C_WR, CFW-08). Writes to a
   mux channel that holds a mapped FPGA (the TA driver, seed and EE/OPT safety
   FPGAs) must start at a mapped register and cover exactly that register; a
   read-only or status register is refused, and a DYNAMIC control register
   (EE/OPT bit 0 clears the latched safety faults) is accepted only while the
   trigger is stopped, as 1 byte up to the register length. A register the
   firmware owns (safety limits, driver clamps, static control: the
   CFG_KEY_SAFETY and unlisted keys of motion_cfg_keys.c) is refused in
   Release and Debug builds; an operational register (TA_CURRENT_DRV,
   TA_PULSE_WIDTH, SEED gains) is accepted only with a value inside its
   allow-list range, decoded little-endian from `data`. Writes to other
   channels are not filtered.
   Returns NULL when the write is allowed, else the reason it is refused. */
const char *fpga_passthrough_write_problem(int mux, int channel, uint8_t i2c_addr,
                                           uint8_t offset, uint8_t len, const uint8_t *data,
                                           bool trigger_running);

#endif /* FPGA_REGISTER_MAP_H */
