/*
 * safety_ufm_record.h - per-unit laser-safety record in the safety FPGA UFM.
 *
 * Each safety FPGA (EE on mux 0 channel 2, OPT on channel 3) holds one record
 * in its user flash memory (UFM). The record carries the per-unit limits that
 * WI-00015 safety calibration derives for that unit. It is written only by
 * a Factory build (OW_FACTORY_PROGRAMMING) through the UFM page commands; a
 * Release build can read it but never write it. At boot and on
 * OW_CMD_FPGA_ATTEST the firmware reads the record (fpga_attest.c), checks
 * magic, layout version, device, reserved bytes and the 16-bit checksum, and
 * checks every limit against the hard bounds of safety_params.h. A record
 * that is present but invalid or out of bounds sets TRIG_INTERLOCK_FPGA_ATTEST.
 *
 * A BLANK UFM (every byte 0x00 or every byte 0xFF, which is what the released
 * bitstreams leave: openmotion-safety-fpga efb_i2c.v has EFB_UFM disabled and
 * no UFM initialisation) means "no per-unit record": the firmware then runs
 * on the compile-time limits of safety_params.h, which are the device
 * ratings, and reports the record as absent. It is not an interlock, so a
 * unit that was never provisioned still operates at its rated limits.
 *
 * Location: UFM pages SAFETY_UFM_RECORD_PAGE and the next one (32 bytes),
 * well inside the 639-page UFM of the MachXO2-2000 and far from the
 * configuration-sector boundary, so the record never overlaps the pages
 * that fpga_attest.c hashes (docs/fpga-attestation.md).
 *
 * Layout version 1, little-endian:
 *
 *   off size field
 *     0   4  magic              "OWSR"
 *     4   1  layout             SAFETY_UFM_RECORD_LAYOUT (1)
 *     5   1  device             2 = EE (MUX_FPGA_SAFE_EE), 3 = OPT (MUX_FPGA_SAFE_OPT)
 *     6   2  pulse_width_ul_us  EE/OPT_PULSE_WIDTH_UL   <= SAFETY_EE_OPT_PULSE_WIDTH_UL_US
 *     8   4  rate_ll_us         EE/OPT_RATE_LL          >= SAFETY_EE_OPT_RATE_LL_MIN_US
 *    12   2  drive_cl_ma        EE/OPT_DRIVE_CL         1 .. SAFETY_EE_OPT_DRIVE_CL_MA
 *    14   2  pwm_current_ma     EE/OPT_PWM_CURRENT      <= SAFETY_EE_OPT_PWM_CURRENT_MA
 *    16   2  cw_current_ma      EE/OPT_CW_CURRENT       <= SAFETY_EE_OPT_CW_CURRENT_MA
 *    18   2  ta_current_max_ma  per-unit ceiling for TA_CURRENT_DRV, 0 = use SAFETY_TA_CURRENT_DRV_MAX_MA
 *    20   2  ta_pulse_max_us    per-unit ceiling for TA_PULSE_WIDTH, 0 = use SAFETY_TA_PULSE_WIDTH_MAX_US
 *    22   8  reserved           zero
 *    30   2  crc16              CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over bytes 0..29
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

#define SAFETY_UFM_RECORD_MAGIC    0x5253574FUL   /* "OWSR" little-endian */
#define SAFETY_UFM_RECORD_LAYOUT   1U
#define SAFETY_UFM_RECORD_SIZE     32U
#define SAFETY_UFM_RECORD_PAGE     600            /* first of two 16-byte UFM pages */
#define SAFETY_UFM_RECORD_PAGES    2

typedef struct safety_ufm_limits {
    uint16_t pulse_width_ul_us;
    uint32_t rate_ll_us;
    uint16_t drive_cl_ma;
    uint16_t pwm_current_ma;
    uint16_t cw_current_ma;
    uint16_t ta_current_max_ma;   /* 0 = not set */
    uint16_t ta_pulse_max_us;     /* 0 = not set */
} safety_ufm_limits_t;

typedef enum {
    SAFETY_UFM_BLANK = 0,          /* no record: all 0x00 or all 0xFF */
    SAFETY_UFM_VALID,              /* record present, checksum and bounds good */
    SAFETY_UFM_BAD_MAGIC,          /* present but not a record */
    SAFETY_UFM_BAD_LAYOUT,         /* unknown layout version */
    SAFETY_UFM_BAD_DEVICE,         /* record for the other safety FPGA */
    SAFETY_UFM_BAD_CHECKSUM,
    SAFETY_UFM_BAD_RESERVED,
    SAFETY_UFM_OUT_OF_BOUNDS       /* a limit outside safety_params.h */
} safety_ufm_status_t;

/* CRC-16/CCITT-FALSE, the record's checksum; also used by motion_config.c. */
uint16_t safety_crc16_ccitt(uint16_t crc, const uint8_t *data, size_t len);

/* Decode and validate a 32-byte record read from UFM for the given device
 * (2 = EE, 3 = OPT). On SAFETY_UFM_VALID *out holds the limits; otherwise
 * *out is left untouched. */
safety_ufm_status_t safety_ufm_record_parse(const uint8_t *raw, size_t len,
                                            uint8_t device, safety_ufm_limits_t *out);

/* Short name of a status for logs and the host message. */
const char *safety_ufm_status_name(safety_ufm_status_t s);

/* Encode a record (used by the Factory build and the host tests). */
void safety_ufm_record_encode(const safety_ufm_limits_t *in, uint8_t device,
                              uint8_t out[SAFETY_UFM_RECORD_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* INC_SAFETY_UFM_RECORD_H_ */
