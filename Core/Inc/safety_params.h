/*
 * safety_params.h - laser-safety parameters owned by the console firmware.
 *
 * Every value here is compiled into the signed firmware image and is in force
 * from reset, whatever the state of the stored configuration record or of
 * the host. None of them can be changed over USB in a Release build
 * (PTR-2026-1-5 follow-up, CVA DC-00094 observation on host-writable state;
 * OI-12 of ECO-secure-boot-1.2.0; SDD DC-00078 ARCH-125 / ARCH-83). The
 * values are applied to the FPGA registers by motion_cfg_apply_settings()
 * (Core/Src/main.c) through the register abstraction and read back; a
 * failed write refuses the trigger (CFW-10).
 *
 * Two kinds of value:
 *
 *   - a LIMIT the firmware writes into a laser-driver or safety-FPGA
 *     register (engineering units of fpga_register_map.c); and
 *   - a HARD BOUND the firmware checks a per-unit value against, where the
 *     per-unit value comes from the safety FPGA's UFM calibration record
 *     (safety_ufm_record.h) or, for an operational key, from the host
 *     configuration (motion_cfg_keys.c).
 *
 * Sources, per value, are given in the comments. "registers.v reset" is the
 * reset value in the FPGA design (openmotion-safety-fpga / -ta-fpga /
 * -seed-fpga src/registers.v at the tags in scripts/sbom-manifest.json), i.e.
 * the value the attested bitstream itself establishes at power-on. WI-00015
 * values are the Open-Motion laser calibration work instruction as encoded in
 * openmotion-sdk omotion/calibration/{laser,safety}.py (revision 2 with the
 * approved automated-process addendum). Values marked "pending DC-00052 §9"
 * have no rating in a released document yet and must be confirmed by the
 * safety review before the final tag (see docs/fpga-attestation.md).
 *
 * Factory builds (OW_FACTORY_PROGRAMMING, CMake preset "Factory") may
 * override the per-unit limits through the configuration for bench and
 * calibration work; the hard bounds still apply there.
 */
#ifndef INC_SAFETY_PARAMS_H_
#define INC_SAFETY_PARAMS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- TEC thermal trip ----------------------------------------------------
 * Threshold temperature of the thermistor the TEC thermal trip compares
 * against (tec_trip_evaluate(), uart_comms.c). In force from reset; it does
 * not depend on the configuration record any more (CFW-12 closure). */
#define SAFETY_TEC_TRIP_C         40.0   /* WI-00015 DEFAULT_USER_CONFIG TEC_TRIP = 40; former motion_config default */
#define SAFETY_TEC_TRIP_MIN_C     20.0   /* hard bounds for a Factory-build override; pending DC-00052 §9 */
#define SAFETY_TEC_TRIP_MAX_C     60.0

/* ---- Safety FPGA (EE and OPT monitors), registers.v of openmotion-safety-fpga
 * Engineering units are those of fpga_register_map.c (us, mA). */
#define SAFETY_EE_OPT_PULSE_WIDTH_LL_US   0U      /* registers.v reset: pulse_width_lower_limit = 0 */
#define SAFETY_EE_OPT_PULSE_WIDTH_UL_US   660U    /* hard bound: WI-00015 MAX_PULSE_WIDTH_LIMIT_US; the per-unit value (TA pulse width x 1.1) comes from the UFM record */
#define SAFETY_EE_OPT_RATE_LL_US          23125U  /* WI-00015 DEFAULT_USER_CONFIG EE_RATE_LL / OPT_RATE_LL (period >= 23125 us, i.e. at most 43.2 Hz for the 40 Hz scan) */
#define SAFETY_EE_OPT_RATE_LL_MIN_US      23125U  /* hard bound: a per-unit value may not allow a higher rate than this */
#define SAFETY_EE_OPT_DRIVE_CL_MA         5000U   /* hard bound: registers.v reset drive_current_limit 0x0a80 = 2688 x 1.86 mA ("Drive current: 5000mA"); per-unit value (measured x 1.3 / 1.1, WI-00015 safety calibration) from the UFM record */
#define SAFETY_OPT_DRIVE_CL_MAX           7200U   /* hard bound for a per-unit OPT_DRIVE_CL from the UFM record, in the register map's units (x 1.86 per LSB; for OPT this is the optical monitor's ADC scale, not a current): the largest value in use on fleet configurations is OPT_THRESH 7143; pending DC-00052 §9 */
#define SAFETY_EE_OPT_PWM_CURRENT_MA      140U    /* registers.v reset pwm_current_limit 0x036b = 875 x 0.160 mA */
#define SAFETY_EE_OPT_CW_CURRENT_MA       140U    /* registers.v reset cw_current_limit 0x036b */

/* ---- TA driver FPGA, registers.v of openmotion-ta-fpga ------------------- */
#define SAFETY_TA_PULSE_WIDTH_MAX_US      600U    /* hard bound for the operational TA_PULSE_WIDTH: WI-00015 MAX_TA_PULSE_WIDTH_US */
#define SAFETY_TA_CURRENT_DRV_MAX_MA      5000U   /* hard bound for the operational TA_CURRENT_DRV: WI-00015 DEFAULT_USER_CONFIG TA_CURRENT_DRV = 5000, the safety FPGA drive rating above; pending DC-00052 §9 */
#define SAFETY_TA_CURRENT_DRV_MIN_MA      0U      /* 0 = laser off; WI-00015 calibration floor is 2000 mA but a lower value is safe */
#define SAFETY_TA_CURRENT_LIMIT_MA        5500U   /* registers.v reset drive_current_limit 0x8647 = 34375 x 0.160 mA ("Drive current: 5500mA"): the driver's own clamp, above the safety monitors' 5000 mA */
#define SAFETY_TA_PWM_MON_CL              88U     /* registers.v reset pwm_mon_current_limit 0x00b0 = 176 x 0.500 */
#define SAFETY_TA_CW_MON_CL               80U     /* registers.v reset cw_mon_current_limit 0x00a0 = 160 x 0.500 */
#define SAFETY_TA_PERIOD_US               9011U   /* registers.v reset period 0x006e00 = 28160 x 0.320 us: not set by WI-00015; the design value governs */

/* ---- Seed driver FPGA, registers.v of openmotion-seed-fpga --------------- */
#define SAFETY_SEED_DDS_CL_MA             80U     /* registers.v reset dds_current_limit 0x03dc = 988 x 0.081 mA ("Limit 80mA") */
#define SAFETY_SEED_CW_CL_MA              160U    /* registers.v reset cw_current_limit 0x0800 = 2048 x 0.079 mA ("limit 160mA") */
#define SAFETY_SEED_ADC_DDS_CL            0U      /* registers.v reset dds_mon_current_limit 0 */
#define SAFETY_SEED_ADC_CW_CL             0U      /* registers.v reset cw_mon_current_limit 0 */
#define SAFETY_SEED_CW_GAIN_MAX_MA        160U    /* hard bound for the operational SEED_CW_GAIN (seed CW current, mA): the seed CW current limit above; WI-00015 default 140 */
#define SAFETY_SEED_DDS_GAIN_MAX          160U    /* hard bound for the operational SEED_DDS_GAIN (seed DDS drive, register-map units): the larger seed current rating (SEED_CW_CL); bench configurations used 100; the hardware DDS current limit (80 mA) clamps the drive; pending DC-00052 §9 */

/* ---- Firmware-owned register loads (safety_params.c) --------------------
 * The ordered list of registers the firmware writes and reads back at boot
 * and after every configuration apply. `per_unit` marks a value that came
 * from a safety FPGA UFM record rather than from this header. */
struct safety_ufm_limits;   /* safety_ufm_record.h */

typedef struct {
    const char *reg;     /* fpga_register_map.c name */
    uint32_t    raw;     /* raw register value, little-endian on the bus */
    bool        per_unit;
} safety_reg_load_t;

#define SAFETY_REG_LOADS_MAX 20U


size_t   safety_reg_loads(const struct safety_ufm_limits *ee, const struct safety_ufm_limits *opt,
                          safety_reg_load_t *out, size_t max);
uint32_t safety_eng_to_raw(const char *reg, double eng);
/* Ceilings for the operational TA_CURRENT_DRV / TA_PULSE_WIDTH: the header
 * bound, lowered by a per-unit record when one sets a smaller value. */
uint32_t safety_ta_current_drv_max_ma(const struct safety_ufm_limits *ee, const struct safety_ufm_limits *opt);
uint32_t safety_ta_pulse_width_max_us(const struct safety_ufm_limits *ee, const struct safety_ufm_limits *opt);

#ifdef __cplusplus
}
#endif

#endif /* INC_SAFETY_PARAMS_H_ */
