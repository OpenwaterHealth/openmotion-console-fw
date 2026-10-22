/*
 * motion_cfg_keys.c - allow-list of configuration keys (see the header).
 */
#include "motion_cfg_keys.h"
#include "safety_params.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Bounds are the engineering units of fpga_register_map.c (us, mA) or the
 * key's own unit (TEC_TRIP: degrees C). The SAFETY rows exist so a Factory
 * build can take a per-unit override from the bench tooling within the hard
 * bounds of safety_params.h; in every other build they are refused. */
static const cfg_key_rule_t rules[] = {
    /* ---- operational: the per-unit laser operating point (WI-00015 laser calibration) */
    { "TA_CURRENT_DRV",     CFG_KEY_OPERATIONAL, SAFETY_TA_CURRENT_DRV_MIN_MA, SAFETY_TA_CURRENT_DRV_MAX_MA, "TA_CURRENT_DRV" },
    { "TA_PULSE_WIDTH",     CFG_KEY_OPERATIONAL, 1.0,  SAFETY_TA_PULSE_WIDTH_MAX_US, "TA_PULSE_WIDTH" },
    { "SEED_CW_GAIN",       CFG_KEY_OPERATIONAL, 0.0,  SAFETY_SEED_CW_GAIN_MAX_MA,   "SEED_CW_GAIN" },
    { "SEED_DDS_GAIN",      CFG_KEY_OPERATIONAL, 0.0,  SAFETY_SEED_DDS_GAIN_MAX,     "SEED_DDS_GAIN" },

    /* ---- opaque host data */
    { "calibration",        CFG_KEY_OPAQUE,      0.0,  0.0, NULL },

    /* ---- safety: firmware-owned (safety_params.h); Factory-build override only */
    { "TEC_TRIP",           CFG_KEY_SAFETY, SAFETY_TEC_TRIP_MIN_C, SAFETY_TEC_TRIP_MAX_C, NULL },
    { "EE_PULSE_WIDTH_LL",  CFG_KEY_SAFETY, 0.0, SAFETY_EE_OPT_PULSE_WIDTH_UL_US, "EE_PULSE_WIDTH_LL" },
    { "EE_PULSE_WIDTH_UL",  CFG_KEY_SAFETY, 0.0, SAFETY_EE_OPT_PULSE_WIDTH_UL_US, "EE_PULSE_WIDTH_UL" },
    { "EE_RATE_LL",         CFG_KEY_SAFETY, SAFETY_EE_OPT_RATE_LL_MIN_US, 4294967295.0, "EE_RATE_LL" },
    { "EE_DRIVE_CL",        CFG_KEY_SAFETY, 1.0, SAFETY_EE_OPT_DRIVE_CL_MA, "EE_DRIVE_CL" },
    { "EE_THRESH",          CFG_KEY_SAFETY, 1.0, SAFETY_EE_OPT_DRIVE_CL_MA, "EE_DRIVE_CL" },
    { "EE_GAIN",            CFG_KEY_SAFETY, 0.0, 100.0, NULL },
    { "EE_PWM_CURRENT",     CFG_KEY_SAFETY, 0.0, SAFETY_EE_OPT_PWM_CURRENT_MA, "EE_PWM_CURRENT" },
    { "EE_CW_CURRENT",      CFG_KEY_SAFETY, 0.0, SAFETY_EE_OPT_CW_CURRENT_MA,  "EE_CW_CURRENT" },
    { "OPT_PULSE_WIDTH_LL", CFG_KEY_SAFETY, 0.0, SAFETY_EE_OPT_PULSE_WIDTH_UL_US, "OPT_PULSE_WIDTH_LL" },
    { "OPT_PULSE_WIDTH_UL", CFG_KEY_SAFETY, 0.0, SAFETY_EE_OPT_PULSE_WIDTH_UL_US, "OPT_PULSE_WIDTH_UL" },
    { "OPT_RATE_LL",        CFG_KEY_SAFETY, SAFETY_EE_OPT_RATE_LL_MIN_US, 4294967295.0, "OPT_RATE_LL" },
    { "OPT_DRIVE_CL",       CFG_KEY_SAFETY, 1.0, SAFETY_OPT_DRIVE_CL_MAX, "OPT_DRIVE_CL" },
    { "OPT_THRESH",         CFG_KEY_SAFETY, 1.0, SAFETY_OPT_DRIVE_CL_MAX, "OPT_DRIVE_CL" },
    { "OPT_GAIN",           CFG_KEY_SAFETY, 0.0, 100.0, NULL },
    { "OPT_PWM_CURRENT",    CFG_KEY_SAFETY, 0.0, SAFETY_EE_OPT_PWM_CURRENT_MA, "OPT_PWM_CURRENT" },
    { "OPT_CW_CURRENT",     CFG_KEY_SAFETY, 0.0, SAFETY_EE_OPT_CW_CURRENT_MA,  "OPT_CW_CURRENT" },
    { "TA_PERIOD",          CFG_KEY_SAFETY, 0.0, 5368709.0, "TA_PERIOD" },
    { "TA_CURRENT_LIMIT",   CFG_KEY_SAFETY, 0.0, SAFETY_TA_CURRENT_LIMIT_MA, "TA_CURRENT_LIMIT" },
    { "TA_PWM_MON_CL",      CFG_KEY_SAFETY, 0.0, 32767.0, "TA_PWM_MON_CL" },
    { "TA_CW_MON_CL",       CFG_KEY_SAFETY, 0.0, 32767.0, "TA_CW_MON_CL" },
    { "SEED_DDS_CL",        CFG_KEY_SAFETY, 0.0, SAFETY_SEED_DDS_CL_MA, "SEED_DDS_CL" },
    { "SEED_CW_CL",         CFG_KEY_SAFETY, 0.0, SAFETY_SEED_CW_CL_MA,  "SEED_CW_CL" },
};

#define RULE_COUNT (sizeof(rules) / sizeof(rules[0]))

bool motion_cfg_safety_keys_writable(void)
{
#if defined(OW_FACTORY_PROGRAMMING)
    return true;
#else
    return false;
#endif
}

const cfg_key_rule_t *motion_cfg_key_rule(const char *name)
{
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < RULE_COUNT; i++) {
        if (strcmp(rules[i].name, name) == 0) {
            return &rules[i];
        }
    }
    return NULL;
}

size_t motion_cfg_key_rules(const cfg_key_rule_t **out)
{
    if (out != NULL) {
        *out = rules;
    }
    return RULE_COUNT;
}

int motion_cfg_skip_token(const jsmntok_t *t, int ntok, int i)
{
    if (i >= ntok) {
        return ntok;
    }
    int children = t[i].size;
    i++;
    for (int k = 0; k < children; k++) {
        i = motion_cfg_skip_token(t, ntok, i);
        if (i >= ntok) {
            return ntok;
        }
    }
    return i;
}

bool motion_cfg_token_number(const char *js, const jsmntok_t *tok, double *out)
{
    char buf[32];
    int len = tok->end - tok->start;

    if (tok->type != JSMN_PRIMITIVE || len <= 0 || len >= (int)sizeof(buf)) {
        return false;
    }
    if (!(js[tok->start] == '-' || (js[tok->start] >= '0' && js[tok->start] <= '9'))) {
        return false; /* true, false, null */
    }
    memcpy(buf, js + tok->start, (size_t)len);
    buf[len] = '\0';
    char *endp = NULL;
    double v = strtod(buf, &endp);
    if (endp != buf + len || !isfinite(v)) {
        return false;
    }
    *out = v;
    return true;
}

static void copy_key(char *dst, size_t dst_len, const char *js, const jsmntok_t *key)
{
    int len = key->end - key->start;
    if (len < 0) {
        len = 0;
    }
    if ((size_t)len >= dst_len) {
        len = (int)dst_len - 1;
    }
    memcpy(dst, js + key->start, (size_t)len);
    dst[len] = '\0';
}

static void explain(char *why, size_t why_len, const char *key, const char *reason)
{
    if (why != NULL && why_len > 0U) {
        (void)snprintf(why, why_len, "%s: %s", key, reason);
    }
}

/* True when the token stream holds a complete "key": value pair at t[i]. */
static bool pair_complete(const jsmntok_t *t, int ntok, int i)
{
    return i + 1 < ntok && t[i].type == JSMN_STRING && t[i].size == 1;
}

/* Check one pair against its rule. */
static cfg_check_t check_pair(const char *js, const char *key, const jsmntok_t *val,
                              bool safety_writable, char *why, size_t why_len)
{
    const cfg_key_rule_t *rule = motion_cfg_key_rule(key);

    if (rule == NULL) {
        explain(why, why_len, key, "not an allowed key");
        return CFG_CHECK_UNKNOWN_KEY;
    }
    if (rule->cls == CFG_KEY_SAFETY && !safety_writable) {
        explain(why, why_len, key, "safety parameter is firmware-owned");
        return CFG_CHECK_SAFETY_KEY;
    }
    if (rule->cls == CFG_KEY_OPAQUE) {
        return CFG_CHECK_OK;
    }
    double v;
    if (!motion_cfg_token_number(js, val, &v)) {
        explain(why, why_len, key, "not a finite number");
        return CFG_CHECK_BAD_VALUE;
    }
    if (v < rule->min || v > rule->max) {
        explain(why, why_len, key, "out of range");
        return CFG_CHECK_BAD_VALUE;
    }
    return CFG_CHECK_OK;
}

cfg_check_t motion_cfg_check_tokens(const char *js, const jsmntok_t *t, int ntok,
                                    bool safety_writable, char *why, size_t why_len)
{
    if (js == NULL || t == NULL || ntok < 1 || t[0].type != JSMN_OBJECT) {
        explain(why, why_len, "", "not a JSON object");
        return CFG_CHECK_NOT_OBJECT;
    }

    int i = 1;
    for (int k = 0; k < t[0].size; k++) {
        if (!pair_complete(t, ntok, i)) {
            explain(why, why_len, "", "truncated");
            return CFG_CHECK_TRUNCATED;
        }
        char key[40];
        copy_key(key, sizeof(key), js, &t[i]);
        cfg_check_t c = check_pair(js, key, &t[i + 1], safety_writable, why, why_len);
        if (c != CFG_CHECK_OK) {
            return c;
        }
        i = motion_cfg_skip_token(t, ntok, i + 1);
    }
    return CFG_CHECK_OK;
}

const char *motion_cfg_check_name(cfg_check_t c)
{
    switch (c) {
    case CFG_CHECK_OK:          return "ok";
    case CFG_CHECK_NOT_OBJECT:  return "not a JSON object";
    case CFG_CHECK_SAFETY_KEY:  return "safety key";
    case CFG_CHECK_UNKNOWN_KEY: return "unknown key";
    case CFG_CHECK_BAD_VALUE:   return "value out of range";
    case CFG_CHECK_TRUNCATED:   return "truncated";
    default:                    return "unknown";
    }
}

/* Whether a pair survives the migration: opaque keys always, operational
 * (and, when keep_safety, safety) keys with a finite in-range value. */
static bool pair_kept(const char *js, const char *key, const jsmntok_t *val, bool keep_safety)
{
    const cfg_key_rule_t *rule = motion_cfg_key_rule(key);
    if (rule == NULL) {
        return false;
    }
    if (rule->cls == CFG_KEY_OPAQUE) {
        return true;
    }
    if (rule->cls == CFG_KEY_SAFETY && !keep_safety) {
        return false;
    }
    double v;
    return motion_cfg_token_number(js, val, &v) && v >= rule->min && v <= rule->max;
}

/* Append "key":value (verbatim value text) to out at *pos; false when full. */
static bool append_pair(char *out, size_t out_len, size_t *pos, const char *js,
                        const jsmntok_t *key, const jsmntok_t *val, bool first)
{
    /* The value text spans from its first token to the end of its last;
     * a string value keeps its quotes. */
    int vstart = val->start;
    int vend = val->end;
    if (val->type == JSMN_STRING) {
        vstart--;
        vend++;
    }
    size_t klen = (size_t)(key->end - key->start);
    size_t vlen = (size_t)(vend - vstart);
    size_t need = klen + 3U + vlen + (first ? 0U : 1U);
    if (*pos + need + 2U > out_len) {
        return false;
    }
    if (!first) {
        out[(*pos)++] = ',';
    }
    out[(*pos)++] = '"';
    memcpy(&out[*pos], js + key->start, klen);
    *pos += klen;
    out[(*pos)++] = '"';
    out[(*pos)++] = ':';
    memcpy(&out[*pos], js + vstart, vlen);
    *pos += vlen;
    return true;
}

int motion_cfg_filter_json(const char *js, const jsmntok_t *t, int ntok, bool keep_safety,
                           char *out, size_t out_len)
{
    if (js == NULL || t == NULL || out == NULL || out_len < 3U || ntok < 1 || t[0].type != JSMN_OBJECT) {
        return -1;
    }
    size_t pos = 0;
    int dropped = 0;
    int kept = 0;
    out[pos++] = '{';

    int i = 1;
    for (int k = 0; k < t[0].size; k++) {
        if (!pair_complete(t, ntok, i)) {
            return -1;
        }
        char key[40];
        copy_key(key, sizeof(key), js, &t[i]);
        int next = motion_cfg_skip_token(t, ntok, i + 1);
        if (next > ntok) {
            return -1;
        }
        if (!pair_kept(js, key, &t[i + 1], keep_safety)) {
            dropped++;
        } else if (!append_pair(out, out_len, &pos, js, &t[i], &t[i + 1], kept == 0)) {
            return -1;
        } else {
            kept++;
        }
        i = next;
    }
    out[pos++] = '}';
    out[pos] = '\0';
    return dropped;
}
