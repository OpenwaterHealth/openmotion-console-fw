/*
 * safety_ufm_record.h - per-unit laser-safety record in the safety FPGA UFM.
 *
 * Each safety FPGA (EE on mux 0 channel 2, OPT on channel 3) holds one record
 * in its user flash memory (UFM): the per-unit limits that WI-00015 safety
 * calibration derives for that unit. It is the console's per-unit safety
 * record (PTR-2026-1-5 item C). It is written only at the factory by a
 * Factory build (OW_FACTORY_PROGRAMMING) through the UFM page commands; a
 * Release build can read it but has no code that writes it.
 *
 * The layout is the one the safety FPGA's own loader reads
 * (openmotion-safety-fpga src/ufm_config.v and docs/ufm_calibration.md,
 * branch calibration_support_merge; tools/make_ufm_record.py builds it), so a
 * loader-capable bitstream applies the same record at its power-on. The
 * firmware does not depend on that: at boot and on OW_CMD_FPGA_ATTEST it reads
 * the record itself (fpga_attest.c), checks magic, layout version, reserved
 * bytes and the checksum, checks every limit against the envelope of
 * safety_envelope.h, and writes the limits into the FPGA registers
 * (safety_envelope.c, main.c) and reads them back.
 *
 * A blank UFM (every byte 0x00 or every byte 0xFF: the released bitstreams
 * leave it so), a corrupt record or a limit outside the envelope latches
 * TRIG_INTERLOCK_FPGA_ATTEST: the laser cannot be armed. There is no "run on
 * the rated limits" fallback.
 *
 * Location: UFM pages 0 and 1 (32 bytes); the UFM is a separate sector from
 * the configuration flash that fpga_attest.c hashes (docs/fpga-attestation.md).
 *
 * Layout version 1, little-endian, raw register values (fpga_register_map.c
 * units: x 0.320 us for times, x 1.86 for DRIVE_CL, x 0.160 mA for currents):
 *
 *   off size field                    register  bound (safety_envelope.h)
 *     0   1  magic0 0x4F 'O'
 *     1   1  magic1 0x57 'W'
 *     2   1  layout version 0x01       0x32 on a loader bitstream
 *     3   1  reserved, zero
 *     4   4  pulse_width_lower_limit  0x00      <= pulse_width_upper_limit
 *     8   4  pulse_width_upper_limit  0x04      <= SAFETY_EE_OPT_PULSE_WIDTH_UL_MAX_US
 *    12   4  rate_lower_limit         0x08      >= SAFETY_EE_OPT_RATE_LL_MIN_US
 *    16   2  drive_current_limit      0x10      1 LSB .. SAFETY_EE_DRIVE_CL_MAX_MA (EE) / SAFETY_OPT_DRIVE_CL_MAX (OPT)
 *    18   2  pwm_current_limit        0x12      <= SAFETY_EE_OPT_PWM_CURRENT_MAX_MA
 *    20   2  cw_current_limit         0x14      <= SAFETY_EE_OPT_CW_CURRENT_MAX_MA
 *    22   8  reserved, zero
 *    30   2  checksum: 16-bit truncated sum of bytes 0..29
 *
 * The FPGA loader checks only the magic and the checksum; the firmware is
 * stricter (layout, reserved bytes, envelope). The record carries no device,
 * serial number, calibration date or operator: adding them changes the
 * layout ufm_config.v reads, which is an FPGA change (TODO in
 * docs/fpga-attestation.md §6).
 *
 * scripts/safety_ufm_record.py builds and checks the same layout on the host.
 */
#ifndef INC_SAFETY_UFM_RECORD_H_
#define INC_SAFETY_UFM_RECORD_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SAFETY_UFM_RECORD_MAGIC0   0x4FU          /* 'O' */
#define SAFETY_UFM_RECORD_MAGIC1   0x57U          /* 'W' */
#define SAFETY_UFM_RECORD_LAYOUT   1U
#define SAFETY_UFM_RECORD_SIZE     32U
#define SAFETY_UFM_RECORD_PAGE     0              /* first of two 16-byte UFM pages (ufm_config.v UFM_PAGE) */
#define SAFETY_UFM_RECORD_PAGES    2

/* Loader status register 0x31 of a loader-capable safety bitstream
 * (docs/ufm_calibration.md): bit0 valid, bit1 done, bit2 magic ok,
 * bit3 checksum ok, bit4 timeout, bit5 committed. */
#define SAFETY_UFM_LOADER_CALIBRATED   0x2FU      /* record loaded and applied */
#define SAFETY_UFM_LOADER_UNCALIBRATED 0x02U      /* no valid record, design defaults in use */

/* Device numbers (mux 0 channel), for the device-specific DRIVE_CL bound. */
#define SAFETY_UFM_DEVICE_EE   2U
#define SAFETY_UFM_DEVICE_OPT  3U

typedef struct safety_ufm_limits {   /* raw register values */
    uint32_t pulse_width_ll;
    uint32_t pulse_width_ul;
    uint32_t rate_ll;
    uint16_t drive_cl;
    uint16_t pwm_current;
    uint16_t cw_current;
} safety_ufm_limits_t;

/* Wire values (OW_CMD_FPGA_ATTEST ufm_status); every value but VALID latches
 * the attestation interlock. */
typedef enum {
    SAFETY_UFM_BLANK = 0,          /* no record: all 0x00 or all 0xFF */
    SAFETY_UFM_VALID,              /* record present, checksum and envelope good */
    SAFETY_UFM_BAD_MAGIC,          /* present but not a record */
    SAFETY_UFM_BAD_LAYOUT,         /* unknown layout version */
    SAFETY_UFM_LOADER_FALLBACK,    /* loader bitstream reports 0x31 != 0x2F (it runs on its defaults) */
    SAFETY_UFM_BAD_CHECKSUM,
    SAFETY_UFM_BAD_RESERVED,
    SAFETY_UFM_OUT_OF_BOUNDS       /* a limit outside safety_envelope.h, or LL > UL */
} safety_ufm_status_t;

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF); used by motion_config.c. */
uint16_t safety_crc16_ccitt(uint16_t crc, const uint8_t *data, size_t len);

/* The record checksum: 16-bit truncated sum of bytes 0..29. */
uint16_t safety_ufm_checksum(const uint8_t *raw);

/* Decode and validate a 32-byte record read from the UFM of the given
 * device (SAFETY_UFM_DEVICE_EE / _OPT). On SAFETY_UFM_VALID *out holds the
 * limits; otherwise *out is left untouched. */
safety_ufm_status_t safety_ufm_record_parse(const uint8_t *raw, size_t len,
                                            uint8_t device, safety_ufm_limits_t *out);

/* Every limit against the envelope (also used for Factory-tool input). */
bool safety_ufm_limits_in_envelope(const safety_ufm_limits_t *l, uint8_t device);

/* Short name of a status for logs and the host message. */
const char *safety_ufm_status_name(safety_ufm_status_t s);

/* Encode a record (host tests; scripts/safety_ufm_record.py for the bench). */
void safety_ufm_record_encode(const safety_ufm_limits_t *in,
                              uint8_t out[SAFETY_UFM_RECORD_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* INC_SAFETY_UFM_RECORD_H_ */
