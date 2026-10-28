/*
 * motion_cfg_keys.h - allow-list and classification of configuration keys.
 *
 * Every key of the stored configuration JSON (motion_config.c) and of a host
 * OW_CMD_USR_CFG write is looked up here before it is stored or applied:
 *
 *   OPERATIONAL  host-writable, numeric, validated against [min, max] in the
 *                engineering units of fpga_register_map.c;
 *   SAFETY       owned by the firmware (safety_envelope.h, safety FPGA UFM
 *                record). Rejected in Release and Debug builds; accepted
 *                within [min, max] only when the build defines
 *                OW_FACTORY_PROGRAMMING (CMake preset "Factory"), for bench
 *                and calibration tooling;
 *   OPAQUE       host data the firmware stores but never interprets
 *                ("calibration": the application's camera calibration block);
 *   unknown      rejected.
 *
 * Pure C (jsmn only, no HAL) so host_tests/test_motion_cfg_keys.c builds it.
 * Record format v3 (MOTION_VER) stores operational and opaque keys only; a v2
 * record is migrated by motion_cfg_filter_json() (motion_config.c).
 */
#ifndef INC_MOTION_CFG_KEYS_H_
#define INC_MOTION_CFG_KEYS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "jsmn.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CFG_KEY_UNKNOWN = 0,
    CFG_KEY_OPERATIONAL,
    CFG_KEY_SAFETY,
    CFG_KEY_OPAQUE
} cfg_key_class_t;

typedef struct {
    const char      *name;
    cfg_key_class_t  cls;
    double           min;    /* numeric keys only */
    double           max;
    const char      *reg;    /* fpga_register_map.c register this key loads, or NULL */
} cfg_key_rule_t;

typedef enum {
    CFG_CHECK_OK = 0,
    CFG_CHECK_NOT_OBJECT,    /* no tokens / root is not an object */
    CFG_CHECK_SAFETY_KEY,    /* safety key present; not writable in this build */
    CFG_CHECK_UNKNOWN_KEY,
    CFG_CHECK_BAD_VALUE,     /* numeric key with a non-numeric, non-finite or out-of-range value */
    CFG_CHECK_TRUNCATED      /* token stream ends inside a value */
} cfg_check_t;

/* True when this build accepts safety keys from a host (Factory builds). */
bool motion_cfg_safety_keys_writable(void);

/* Lower an operational key's ceiling for this unit (the per-unit safety
 * record: TA_PULSE_WIDTH <= EE/OPT PULSE_WIDTH_UL). Never raises it above the rule's max. Applies to host
 * config writes, the stored-record apply and the I2C pass-through. */
void motion_cfg_set_unit_ceiling(const char *name, double max);

/* The rule's max, lowered by a per-unit ceiling. */
double motion_cfg_effective_max(const cfg_key_rule_t *rule);

/* Rule for a key, or NULL when the key is not in the allow-list. */
const cfg_key_rule_t *motion_cfg_key_rule(const char *name);

/* Iterate the allow-list (for the host message and the docs). */
size_t motion_cfg_key_rules(const cfg_key_rule_t **out);

/* Check a tokenized JSON object: every top-level key must be allowed and
 * every numeric value in range. `why` (may be NULL) receives "KEY: reason".
 * Returns CFG_CHECK_OK or the first problem found. */
cfg_check_t motion_cfg_check_tokens(const char *js, const jsmntok_t *t, int ntok,
                                    bool safety_writable, char *why, size_t why_len);

const char *motion_cfg_check_name(cfg_check_t c);

/* Rebuild the object keeping only the keys this build stores (operational,
 * opaque and, when keep_safety, safety keys whose values are in range); the
 * kept values are copied verbatim. Returns the number of keys dropped, or -1
 * when `out` is too small or the tokens are malformed. Used to migrate a v2
 * record. */
int motion_cfg_filter_json(const char *js, const jsmntok_t *t, int ntok, bool keep_safety,
                           char *out, size_t out_len);

/* Index of the token after the whole value starting at t[i] (skips nested
 * objects and arrays). Returns ntok when the stream is truncated. */
int motion_cfg_skip_token(const jsmntok_t *t, int ntok, int i);

/* Parse a primitive token as a finite number; false for true/false/null or
 * anything strtod does not consume completely. */
bool motion_cfg_token_number(const char *js, const jsmntok_t *tok, double *out);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTION_CFG_KEYS_H_ */
