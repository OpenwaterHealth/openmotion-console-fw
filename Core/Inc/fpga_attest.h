/*
 * fpga_attest.h - FPGA configuration integrity check (firmware-anchored).
 *
 * The three MachXO2 designs on the console (seed driver, TA driver and the
 * laser-safety design, the latter in two devices: EE and OPT) are
 * factory-programmed. Nothing in the signed-firmware root of trust covered
 * them (PTR-2026-1-5 follow-up): a host with USB access could reprogram them
 * through OW_FPGA_PROG, and nothing checked at boot that the configuration
 * present was the released bitstream.
 *
 * This module closes the second half for the two safety devices (EE and
 * OPT, the laser interlock); the seed and TA drivers are reported as "not
 * checked" (decision 2026-10-22). For each attested device it
 *   1. opens the configuration interface in TRANSPARENT mode (user logic
 *      keeps running), resets the configuration address and streams every
 *      configuration page (FPGA_ATTEST_CFG_PAGES x 16 bytes) through SHA-256,
 *      then closes the interface;
 *   2. compares the digest with the one compiled into the firmware
 *      (fpga_attest_expected.h, generated at build time by
 *      scripts/gen_fpga_digests.py from the released .jed artefacts recorded
 *      in scripts/sbom-manifest.json);
 *   3. for the two safety devices additionally reads the per-unit UFM
 *      calibration record (safety_ufm_record.h) and validates it.
 *
 * The check runs in the background from the main loop (fpga_attest_service),
 * a few pages per call, so the console enumerates and answers commands while
 * it runs; the trigger cannot be armed until it has passed
 * (TRIG_INTERLOCK_FPGA_ATTEST). It starts once at boot and again on host
 * request (OW_CMD_FPGA_ATTEST, reserved = 1); it never starts inside a scan.
 * Any mismatch, read failure or invalid record is sticky until the next
 * run passes.
 *
 * Timing: I2C1 runs at fast mode (~385 kHz); the bench figures are in
 * docs/fpga-attestation.md section 5.
 */
#ifndef INC_FPGA_ATTEST_H_
#define INC_FPGA_ATTEST_H_

#include <stdbool.h>
#include <stdint.h>

#include "XO2_dev.h"
#include "safety_ufm_record.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FPGA_ATTEST_DEV_COUNT 4U   /* MUX_FPGA_SEED, _TA, _SAFE_EE, _SAFE_OPT */

typedef enum {
    FPGA_ATTEST_IDLE = 0,   /* never run since reset */
    FPGA_ATTEST_RUNNING,
    FPGA_ATTEST_PASS,
    FPGA_ATTEST_FAIL
} fpga_attest_state_t;

/* Per-device status bits (fpga_attest_dev_report_t.status). */
#define FPGA_ATTEST_ST_OK            0x00U
#define FPGA_ATTEST_ST_DIGEST        0x01U  /* configuration digest mismatch */
#define FPGA_ATTEST_ST_READ_FAIL     0x02U  /* configuration interface or page read failed */
#define FPGA_ATTEST_ST_UFM_INVALID   0x04U  /* safety record present but invalid or out of bounds */
#define FPGA_ATTEST_ST_UFM_BLANK     0x08U  /* informational: no safety record provisioned */
#define FPGA_ATTEST_ST_UFM_VALID     0x10U  /* informational: per-unit limits in force */
#define FPGA_ATTEST_ST_NOT_CHECKED   0x40U  /* informational: device not in the attested set (seed, TA) */
#define FPGA_ATTEST_ST_PENDING       0x80U  /* not checked yet in this run */
#define FPGA_ATTEST_ST_FAULT_MASK    (FPGA_ATTEST_ST_DIGEST | FPGA_ATTEST_ST_READ_FAIL | FPGA_ATTEST_ST_UFM_INVALID)

/* OW_CMD_FPGA_ATTEST reply, packed little-endian. */
typedef struct __attribute__((packed)) {
    uint8_t  device;          /* mux 0 channel: 0 seed, 1 TA, 2 EE, 3 OPT */
    uint8_t  status;          /* FPGA_ATTEST_ST_* */
    uint8_t  ufm_status;      /* safety_ufm_status_t (safety devices), 0xFF for others */
    uint8_t  reserved;
    uint16_t pages_read;
    uint16_t elapsed_ms;      /* configuration readback time of the last run */
    uint32_t devid;           /* Lattice device ID read before the open (0 when unreadable) */
    uint8_t  digest[32];      /* computed (zero until read) */
    uint8_t  expected[32];    /* compiled-in */
} fpga_attest_dev_report_t;

typedef struct __attribute__((packed)) {
    uint8_t  struct_version;  /* 1 */
    uint8_t  state;           /* fpga_attest_state_t */
    uint8_t  progress_pct;    /* 0..100 of the current/last run */
    uint8_t  interlocked;     /* 1 while TRIG_INTERLOCK_FPGA_ATTEST is set */
    uint16_t run_count;       /* completed runs since reset */
    uint16_t cfg_pages;       /* FPGA_ATTEST_CFG_PAGES */
    fpga_attest_dev_report_t dev[FPGA_ATTEST_DEV_COUNT];
} fpga_attest_report_t;

/* Bind the XO2 handle (mux 0; the channel is set per device) and reset the
 * state. The first run must be started explicitly once the buses are up. */
void fpga_attest_init(XO2Handle_t *h);

/* Start a run. Refused (false) while one is running or the trigger runs. */
bool fpga_attest_start(void);

/* Advance the current run by at most budget_ms of bus time. Call from the
 * main loop only. */
void fpga_attest_service(uint32_t budget_ms);

fpga_attest_state_t fpga_attest_state(void);
bool fpga_attest_running(void);

/* True until a run has passed: before the first run completes and after any
 * failure. Trigger_Start() refuses while it is set. */
bool fpga_attest_interlocked(void);
/* True after a run has FAILED (sticky until a run passes): Trigger_SetConfig()
 * refuses while it is set. */
bool fpga_attest_failed(void);

const fpga_attest_report_t *fpga_attest_report(void);

/* Per-unit limits of a safety device (2 = EE, 3 = OPT) from its UFM record,
 * or NULL when no valid record was read in the last completed run. */
const safety_ufm_limits_t *fpga_attest_ufm_limits(uint8_t device);

/* True once after a completed run whose UFM limits differ from the previous
 * run (or the first completed run): the caller re-applies the safety loads. */
bool fpga_attest_take_reapply_request(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_FPGA_ATTEST_H_ */
