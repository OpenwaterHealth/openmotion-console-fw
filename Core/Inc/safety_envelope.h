/*
 * safety_envelope.h - the laser-safety envelope compiled into the firmware.
 *
 * Every value here is compiled into the signed firmware image and is in force
 * from reset, whatever the state of the stored configuration record, of the
 * safety FPGA UFM records or of the host (PTR-2026-1-5 follow-up, CVA
 * DC-00094 observation on host-writable state; OI-12 of ECO-secure-boot-1.2.0;
 * SDD DC-00078 ARCH-125 / ARCH-83). None of them can be changed over USB.
 *
 * The envelope is the firmware's own ceiling. Each value is one of:
 *
 *   - a BOUND (_MAX / _MIN): the per-unit safety record in the EE/OPT safety
 *     FPGA UFM (safety_ufm_record.h, written at the factory, WI-00015) and the
 *     host-programmable operating point (motion_cfg_keys.c) are checked
 *     against it and can only be MORE conservative, never less. A record
 *     outside the envelope latches the attestation interlock
 *     (TRIG_INTERLOCK_FPGA_ATTEST); a setpoint outside it is refused;
 *   - a FIXED value the firmware writes itself (TEC trip, the TA and seed
 *     driver clamps). WI-00015 does not calibrate these per unit. A Factory
 *     build (OW_FACTORY_PROGRAMMING) may override them for bench work, and
 *     only within the bounds of motion_cfg_keys.c (the TEC trip only
 *     downwards).
 *
 * Which values WI-00015 calibrates per unit (openmotion-sdk
 * omotion/calibration/safety.py and safety_workflow.py, revision 2 with the
 * automated-process addendum), checked 2026-10-22:
 *   per unit:  EE/OPT_DRIVE_CL (the EE/OPT thresholds; EE/OPT_THRESH alias the
 *              same register): mean measured current x 1.1 (EE) / x 1.3 (OPT);
 *              EE/OPT_PULSE_WIDTH_UL: TA pulse width x 1.1, at most 660 us.
 *              Both come from the UFM record; this envelope bounds them.
 *   fixed:     TEC_TRIP = 40 C and EE/OPT_RATE_LL = 23125 us
 *              (DEFAULT_USER_CONFIG in laser.py, never recalculated);
 *              PWM/CW current limits and PULSE_WIDTH_LL are not touched by
 *              WI-00015 (safety FPGA registers.v reset values).
 *
 * Sources, per value, are given in the comments. "registers.v reset" is the
 * reset value in the FPGA design (openmotion-safety-fpga / -ta-fpga /
 * -seed-fpga src/registers.v at the tags in scripts/sbom-manifest.json), i.e.
 * the value the attested bitstream itself establishes at power-on. Values
 * marked "pending DC-00052 §9" have no rating in a released document yet and
 * must be confirmed by the safety review before the final tag (see
 * docs/fpga-attestation.md).
 */
#ifndef INC_SAFETY_ENVELOPE_H_
#define INC_SAFETY_ENVELOPE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- TEC thermal trip ----------------------------------------------------
 * Threshold temperature of the thermistor the TEC thermal trip compares
 * against (tec_trip_evaluate(), uart_comms.c). In force from reset; it does
 * not depend on any record (CFW-12 closure). */
#define SAFETY_TEC_TRIP_C         40.0   /* WI-00015 DEFAULT_USER_CONFIG TEC_TRIP = 40; not calibrated per unit */
#define SAFETY_TEC_TRIP_MIN_C     20.0   /* lowest Factory-build override; pending DC-00052 §9 */
#define SAFETY_TEC_TRIP_MAX_C     SAFETY_TEC_TRIP_C   /* an override may only trip earlier */

/* ---- Safety FPGA (EE and OPT monitors), registers.v of openmotion-safety-fpga
 * Register LSBs, identical to the scales of fpga_register_map.c (the host
 * tests check that): the UFM record carries raw register values. */
#define SAFETY_FPGA_TIME_LSB_US      0.320   /* PULSE_WIDTH_LL/UL, RATE_LL */
#define SAFETY_FPGA_DRIVE_LSB        1.86    /* DRIVE_CL: mA for EE; the optical monitor's ADC scale for OPT */
#define SAFETY_FPGA_CURRENT_LSB_MA   0.160   /* PWM_CURRENT, CW_CURRENT */

#define SAFETY_EE_OPT_PULSE_WIDTH_UL_MAX_US  660U    /* WI-00015 MAX_PULSE_WIDTH_LIMIT_US (600 us x 1.1) */
#define SAFETY_EE_OPT_RATE_LL_MIN_US         23125U  /* WI-00015 DEFAULT_USER_CONFIG EE_RATE_LL / OPT_RATE_LL: period >= 23125 us, at most 43.2 Hz for the 40 Hz scan (trigger.h TRIGGER_FREQ_MAX_HZ) */
#define SAFETY_EE_DRIVE_CL_MAX_MA            5000U   /* registers.v reset drive_current_limit 0x0a80 = 2688 x 1.86 mA ("Drive current: 5000mA") */
#define SAFETY_OPT_DRIVE_CL_MAX              7200U   /* register-map units (x 1.86 per LSB, optical monitor ADC scale): the largest OPT_THRESH in use on fleet configurations is 7143; pending DC-00052 §9 */
#define SAFETY_EE_OPT_PWM_CURRENT_MAX_MA     140U    /* registers.v reset pwm_current_limit 0x036b = 875 x 0.160 mA */
#define SAFETY_EE_OPT_CW_CURRENT_MAX_MA      140U    /* registers.v reset cw_current_limit 0x036b */

/* ---- TA driver FPGA, registers.v of openmotion-ta-fpga ------------------- */
#define SAFETY_TA_PULSE_WIDTH_MAX_US      600U    /* bound for the operational TA_PULSE_WIDTH: WI-00015 MAX_TA_PULSE_WIDTH_US; further lowered to the unit's EE/OPT PULSE_WIDTH_UL */
#define SAFETY_TA_CURRENT_DRV_MAX_MA      5000U   /* bound for the operational TA_CURRENT_DRV: the EE drive rating above; further lowered to the unit's EE_DRIVE_CL; pending DC-00052 §9 */
#define SAFETY_TA_CURRENT_DRV_MIN_MA      0U      /* 0 = laser off; WI-00015 calibration floor is 2000 mA but a lower value is safe */
#define SAFETY_TA_CURRENT_LIMIT_MA        5500U   /* fixed: registers.v reset drive_current_limit 0x8647 = 34375 x 0.160 mA ("Drive current: 5500mA"), the driver's own clamp */
#define SAFETY_TA_PWM_MON_CL              88U     /* fixed: registers.v reset pwm_mon_current_limit 0x00b0 = 176 x 0.500 */
#define SAFETY_TA_CW_MON_CL               80U     /* fixed: registers.v reset cw_mon_current_limit 0x00a0 = 160 x 0.500 */
#define SAFETY_TA_PERIOD_US               9011U   /* fixed: registers.v reset period 0x006e00 = 28160 x 0.320 us; not set by WI-00015 */

/* ---- Seed driver FPGA, registers.v of openmotion-seed-fpga --------------- */
#define SAFETY_SEED_DDS_CL_MA             80U     /* fixed: registers.v reset dds_current_limit 0x03dc = 988 x 0.081 mA ("Limit 80mA") */
#define SAFETY_SEED_CW_CL_MA              160U    /* fixed: registers.v reset cw_current_limit 0x0800 = 2048 x 0.079 mA ("limit 160mA") */
#define SAFETY_SEED_CW_GAIN_MAX_MA        160U    /* bound for the operational SEED_CW_GAIN (seed CW current, mA): the seed CW current limit above; WI-00015 default 140 */
#define SAFETY_SEED_DDS_GAIN_MAX          160U    /* bound for the operational SEED_DDS_GAIN (register-map units): the larger seed current rating (SEED_CW_CL); bench configurations used 100; the hardware DDS current limit (80 mA) clamps the drive; pending DC-00052 §9 */

/* ---- Firmware-owned register loads (safety_envelope.c) -------------------
 * The ordered list of registers the firmware writes and reads back at boot
 * and after every configuration apply. `per_unit` marks a value that came
 * from a safety FPGA UFM record. */
struct safety_ufm_limits;   /* safety_ufm_record.h */

typedef struct {
    const char *reg;     /* fpga_register_map.c name */
    uint32_t    raw;     /* raw register value, little-endian on the bus */
    bool        per_unit;
} safety_reg_load_t;

#define SAFETY_REG_LOADS_MAX 20U

/* The loads: the EE / OPT limits from that device's record (raw, verbatim)
 * when one is given; none for a device without a valid record (its
 * registers keep the design reset values and the attestation interlock is
 * set, fpga_attest.c); then the fixed TA and seed driver clamps. */
size_t   safety_reg_loads(const struct safety_ufm_limits *ee, const struct safety_ufm_limits *opt,
                          safety_reg_load_t *out, size_t max);
uint32_t safety_eng_to_raw(const char *reg, double eng);
/* Ceilings for the operational TA_CURRENT_DRV (mA) / TA_PULSE_WIDTH (us): the
 * envelope bound, lowered by the unit's records (EE_DRIVE_CL; the smaller of
 * EE/OPT_PULSE_WIDTH_UL). A NULL record leaves the envelope bound. */
double   safety_ta_current_drv_max_ma(const struct safety_ufm_limits *ee);
double   safety_ta_pulse_width_max_us(const struct safety_ufm_limits *ee, const struct safety_ufm_limits *opt);

#ifdef __cplusplus
}
#endif

#endif /* INC_SAFETY_ENVELOPE_H_ */
