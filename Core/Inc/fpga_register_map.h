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

/* Host I2C pass-through write filter (OW_CTRL_I2C_WR, CFW-08). Writes to a
   mux channel that holds a mapped FPGA (the TA driver, seed and EE/OPT safety
   FPGAs) must start at a mapped register and cover exactly that register; a
   read-only or status register is refused, and a DYNAMIC control register
   (EE/OPT bit 0 clears the latched safety faults) is accepted only while the
   trigger is stopped, as 1 byte up to the register length. Writes to other
   channels are not filtered. No value ranges are checked here yet.
   Returns NULL when the write is allowed, else the reason it is refused. */
const char *fpga_passthrough_write_problem(int mux, int channel, uint8_t i2c_addr,
                                           uint8_t offset, uint8_t len,
                                           bool trigger_running);

#endif /* FPGA_REGISTER_MAP_H */
