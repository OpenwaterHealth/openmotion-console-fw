#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "jsmn.h"
#include "json_strict.h"

/* Pull the modules in directly for the host build (no STM32 HAL dependencies). */
#include "../Core/Src/jsmn.c"
#include "../Core/Src/json_strict.c"

/*
 * jsmn strict mode (CFW-15) and json_object_is_strict().
 *
 * The firmware builds jsmn with JSMN_STRICT (CMakeLists.txt) and checks a user
 * config with json_object_is_strict() before storing it (motion_config.c). The
 * documents the hosts really send must still parse; non-JSON that
 * Python's json.dumps can emit (NaN, Infinity, -Infinity) and malformed input
 * must not.
 */
#ifndef JSMN_STRICT
#error "build with -DJSMN_STRICT, as the firmware is"
#endif

#define NTOK 256

static jsmntok_t toks[NTOK];

static int parse(const char *js)
{
    jsmn_parser p;
    jsmn_init(&p, NULL);
    return jsmn_parse(&p, js, strlen(js), toks, NTOK, NULL);
}

static int strict_ok(const char *js)
{
    return json_object_is_strict(js, strlen(js), toks, NTOK);
}

/* What the SDK sends for SET_TRIG (compact) and what older SDKs sent (spaced). */
static const char *TRIGGER_COMPACT =
    "{\"TriggerStatus\":2,\"TriggerFrequencyHz\":40,\"TriggerPulseWidthUsec\":500,"
    "\"LaserPulseDelayUsec\":100,\"LaserPulseWidthUsec\":500,\"LaserPulseSkipInterval\":600,"
    "\"LaserPulseSkipDelayUsec\":1800,\"EnableSyncOut\":true,\"EnableTaTrigger\":true}";
static const char *TRIGGER_SPACED =
    "{\"TriggerStatus\": 1, \"TriggerFrequencyHz\": 40.0, \"TriggerPulseWidthUsec\": 500, "
    "\"LaserPulseDelayUsec\": 100, \"LaserPulseWidthUsec\": 500, \"LaserPulseSkipInterval\": 600, "
    "\"LaserPulseSkipDelayUsec\": 1800, \"EnableSyncOut\": false, \"EnableTaTrigger\": false}";
/* The bench console's stored user config (MotionConfig.to_wire_bytes). */
static const char *USER_CFG =
    "{\"TEC_TRIP\":40,\"OPT_THRESH\":7143,\"EE_THRESH\":5000,\"EE_GAIN\":1.86,\"OPT_GAIN\":1.86}";

int main(void)
{
    /* Valid documents parse in strict mode and pass the strict check. */
    const char *good[] = {
        TRIGGER_COMPACT, TRIGGER_SPACED, USER_CFG,
        "{}",
        "{\"a\":-1.5e-3,\"b\":0,\"c\":null,\"d\":[1,2,{\"e\":true}],\"f\":\"text, with: colons\"}",
        "{\"cal\":{\"c_min\":[0.1,0.2],\"i_max\":[1e3,2E+2]}}",
    };
    for (size_t i = 0; i < sizeof(good) / sizeof(good[0]); i++) {
        int r = parse(good[i]);
        if (r < 1 || toks[0].type != JSMN_OBJECT || !strict_ok(good[i])) {
            printf("FAIL: valid JSON rejected (r=%d): %s\n", r, good[i]);
            return 1;
        }
    }

    /* Malformed or non-JSON documents: rejected (most already at the jsmn step,
     * in strict mode; the jsmn result is printed for the record). */
    const char *rejects[] = {
        "{\"a\":NaN}",          /* Python json.dumps(float('nan')) */
        "{\"a\":Infinity}",     /* json.dumps(float('inf')) */
        "{\"a\":bare}",
        "{1:2}",                /* primitive as key */
        "{\"a\":1",             /* truncated: object not closed */
        ",",                    /* top-level comma (upstream fix 824d9a7) */
    };
    for (size_t i = 0; i < sizeof(rejects) / sizeof(rejects[0]); i++) {
        printf("  %-16s jsmn r=%d\n", rejects[i], parse(rejects[i]));
        if (strict_ok(rejects[i])) {
            printf("FAIL: accepted: %s\n", rejects[i]);
            return 1;
        }
    }

    /* jsmn strict mode checks only the first character of an unquoted value;
     * json_object_is_strict() closes that gap. */
    const char *value_rejects[] = {
        "{\"a\":-Infinity}",    /* json.dumps(float('-inf')) */
        "{\"a\":nan}",
        "{\"a\":nope}",
        "{\"a\":tru}",
        "{\"a\":1.2.3x}",
        "{\"a\":0x10}",
        "{\"a\":1e999}",        /* overflows to inf */
    };
    for (size_t i = 0; i < sizeof(value_rejects) / sizeof(value_rejects[0]); i++) {
        if (strict_ok(value_rejects[i])) {
            printf("FAIL: strict check accepted: %s\n", value_rejects[i]);
            return 1;
        }
    }

    /* Root must be an object. */
    assert(!strict_ok("[1,2]"));
    assert(!strict_ok("\"s\""));
    /* NOMEM: too many tokens for the pool is a failure, not a partial pass. */
    {
        jsmntok_t few[4];
        assert(!json_object_is_strict(USER_CFG, strlen(USER_CFG), few, 4));
    }

    printf("jsmn strict host tests OK\n");
    return 0;
}
