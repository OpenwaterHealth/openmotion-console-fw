#include "motion_config.h"

#include "common.h" // COMMAND_MAX_SIZE for internal wire buffer
#include "jsmn.h"
#include "json_strict.h"
#include "motion_cfg_keys.h"
#include "msg_queue.h"
#include "safety_ufm_record.h" // safety_crc16_ccitt

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

static motion_cfg_t g_cfg;
static bool       g_cfg_loaded = false;
static bool       g_cfg_defaulted = false; // running on defaults, see motion_cfg_is_defaulted()

static uint8_t g_cfg_wire_buf[COMMAND_MAX_SIZE];

/* Token pool for the strict parse and the key policy. Static: too large for
 * the command-handler stack. Single-threaded use only (main loop). */
static jsmntok_t g_cfg_tokens[MOTION_CFG_MAX_JSON_TOKENS];
/* Scratch for the v2 -> v3 migration. */
static char g_cfg_scratch[MOTION_CFG_JSON_MAX];

static char g_cfg_last_refusal[72];

// ------------------- CRC16-CCITT -------------------
// CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no final XOR (shared with the
// safety UFM record). The running value lets several ranges be chained.
static uint16_t crc16_ccitt_update(uint16_t crc, const uint8_t *data, size_t len)
{
    return safety_crc16_ccitt(crc, data, len);
}

// v1 record CRC: magic, version and seq only (the bytes before the crc field).
// Only used to recognise a v1 record for migration.
static uint16_t motion_cfg_calc_crc_v1(const motion_cfg_t *cfg)
{
    return crc16_ccitt_update(0xFFFFu, (const uint8_t *)cfg,
                              offsetof(motion_cfg_t, crc));
}

// v2+ record CRC (CFW-02): every byte of the record except the crc field
// itself -- magic, version, seq, the two reserved bytes and the whole JSON
// area, which motion_cfg_normalize_json() NUL-pads so the value is stable.
static uint16_t motion_cfg_calc_crc(const motion_cfg_t *cfg)
{
    const uint8_t *p = (const uint8_t *)cfg;
    const size_t after = offsetof(motion_cfg_t, crc) + sizeof(cfg->crc);
    uint16_t crc = crc16_ccitt_update(0xFFFFu, p, offsetof(motion_cfg_t, crc));

    return crc16_ccitt_update(crc, p + after, sizeof(motion_cfg_t) - after);
}

// Report a configuration storage error on the console and to the host.
static void motion_cfg_report(const char *msg)
{
    printf("ERROR: %s\r\n", msg);
    char m[MQ_MAX_MSG_SIZE];
    int n = snprintf(m, sizeof m, "{\"type\": \"system\", \"state\": \"error\", \"msg\": \"%s\"}", msg);
    if (n > 0 && (size_t)n < sizeof m) {
        (void)mq_push(m, (size_t)n);
    }
}

// ------------------- Helpers -------------------

// Ensure json is NUL-terminated and zero-pad tail so CRC is stable
static void motion_cfg_normalize_json(motion_cfg_t *cfg)
{
    cfg->json[MOTION_CFG_JSON_MAX - 1U] = '\0';

    size_t used = strnlen(cfg->json, MOTION_CFG_JSON_MAX);
    if (used < (MOTION_CFG_JSON_MAX - 1U)) {
        memset(&cfg->json[used + 1U],
               0U,
               (MOTION_CFG_JSON_MAX - 1U) - used);
    }
}

// Build a clean default config in RAM. v3 defaults carry no operational
// values: the laser operating point is per unit and comes from the host; the
// safety parameters are compiled in and need nothing here.
static void motion_cfg_make_defaults(motion_cfg_t *dst)
{
    memset(dst, 0, sizeof(motion_cfg_t));

    dst->magic      = MOTION_MAGIC;
    dst->version    = MOTION_VER;
    dst->seq        = 0;
    snprintf(dst->json, MOTION_CFG_JSON_MAX, "%s", "{}");

    motion_cfg_normalize_json(dst);
    dst->crc        = motion_cfg_calc_crc(dst);
}

// Structural validity of a record with a whole-record CRC (v2 or v3): magic,
// the given version, CRC, terminated JSON.
static bool motion_cfg_record_ok(const motion_cfg_t *cfg, uint32_t version)
{
    if (cfg->magic   != MOTION_MAGIC) { return false; }
    if (cfg->version != version)      { return false; }
    if (memchr(cfg->json, '\0', MOTION_CFG_JSON_MAX) == NULL) {
        return false;
    }
    return motion_cfg_calc_crc(cfg) == cfg->crc;
}

// A v1 record written by older firmware: its CRC covers only magic, version
// and seq, so its JSON is not protected. Accepted once, for migration.
static bool motion_cfg_is_valid_v1(const motion_cfg_t *cfg)
{
    if (cfg->magic   != MOTION_MAGIC)  { return false; }
    if (cfg->version != MOTION_VER_V1) { return false; }
    if (memchr(cfg->json, '\0', MOTION_CFG_JSON_MAX) == NULL) {
        return false;
    }
    return motion_cfg_calc_crc_v1(cfg) == cfg->crc;
}

// Do a complete write of g_cfg into flash page.
// - bumps seq
// - normalizes json
// - recomputes crc
// - erases page and writes words
static HAL_StatusTypeDef motion_cfg_writeback(void)
{
    HAL_StatusTypeDef st;

    // bump monotonic sequence
    g_cfg.seq++;

    motion_cfg_normalize_json(&g_cfg);
    g_cfg.crc = motion_cfg_calc_crc(&g_cfg);

    // Erase page [ADDR_FLASH_PAGE_62 .. ADDR_FLASH_PAGE_63)
    st = Flash_Erase(MOTION_CFG_PAGE_ADDR, MOTION_CFG_PAGE_END);
    if (st != HAL_OK) {
        return st;
    }

    // Program entire struct word-by-word
    st = Flash_Write(MOTION_CFG_PAGE_ADDR,
                     (uint32_t *)&g_cfg,
                     (uint32_t)(sizeof(motion_cfg_t) / sizeof(uint32_t)));
    if (st != HAL_OK) {
        motion_cfg_report("motion config flash write failed");
        return st;
    }

    // CFW-02: read the record back and compare. The D-cache is off, so this
    // reads the flash itself. A failed or partial write is reported and
    // returned; the caller (a host write) then gets OW_ERROR.
    if (memcmp((const void *)MOTION_CFG_PAGE_ADDR, &g_cfg, sizeof(motion_cfg_t)) != 0) {
        motion_cfg_report("motion config flash write verify failed");
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Raw load from flash into g_cfg
static void motion_cfg_load_raw(void)
{
    Flash_Read(MOTION_CFG_PAGE_ADDR,
               (uint32_t *)&g_cfg,
               (uint32_t)(sizeof(motion_cfg_t) / sizeof(uint32_t)));
}

/* Tokenize g_cfg_scratch-free: parse json[0..len) into the static pool.
 * Returns the token count, or <0 when it is not a strict JSON object. */
static int motion_cfg_tokenize(const char *json, size_t len)
{
    if (!json_object_is_strict(json, len, g_cfg_tokens, MOTION_CFG_MAX_JSON_TOKENS)) {
        return -1;
    }
    jsmn_parser parser;
    jsmn_init(&parser, NULL);
    return jsmn_parse(&parser, json, len, g_cfg_tokens, MOTION_CFG_MAX_JSON_TOKENS, NULL);
}

/* Migrate the JSON of a v1/v2 record to the v3 key policy: keep the
 * operational and opaque keys, drop (and log) the safety and unknown keys.
 * In a Factory build the safety keys are kept as bench overrides. Returns
 * false when the old JSON cannot be parsed or the result does not fit. */
static bool motion_cfg_migrate_json(void)
{
    size_t len = strnlen(g_cfg.json, MOTION_CFG_JSON_MAX);
    int r = motion_cfg_tokenize(g_cfg.json, len);
    if (r < 1) {
        return false;
    }
    int dropped = motion_cfg_filter_json(g_cfg.json, g_cfg_tokens, r,
                                         motion_cfg_safety_keys_writable(),
                                         g_cfg_scratch, sizeof(g_cfg_scratch));
    if (dropped < 0) {
        return false;
    }
    printf("motion config: migrating record v%lu to v3 (%d key(s) dropped: safety parameters are firmware-owned)\r\n",
           (unsigned long)(g_cfg.version >> 16), dropped);
    memcpy(g_cfg.json, g_cfg_scratch, strlen(g_cfg_scratch) + 1U);
    g_cfg.version = MOTION_VER;
    return true;
}

// Ensure g_cfg is initialized and valid
static void motion_cfg_ensure_loaded(void)
{
    if (g_cfg_loaded) {
        return;
    }

    motion_cfg_load_raw();

    bool ok;
    if (motion_cfg_record_ok(&g_cfg, MOTION_VER)) {
        // v3 record, intact. Its keys are re-checked by the apply.
        ok = true;
    } else if (motion_cfg_record_ok(&g_cfg, MOTION_VER_V2) || motion_cfg_is_valid_v1(&g_cfg)) {
        // Older record: migrate its JSON to the v3 policy and rewrite it.
        // A record that cannot be migrated is invalid as a whole.
        ok = motion_cfg_migrate_json();
        if (ok) {
            ok = (motion_cfg_writeback() == HAL_OK);
        }
        if (!ok) {
            motion_cfg_report("motion config migration failed; defaults in use, trigger refused");
        }
    } else {
        ok = false;
        motion_cfg_report("motion config invalid or missing; defaults in use, trigger refused");
    }

    if (!ok) {
        // CFW-03: no valid record (first boot, erased sector, corruption or
        // an unmigratable record). Run on the built-in defaults in RAM only:
        // the stored record is left as it is for inspection or recovery,
        // the host is told, and Trigger_Start() refuses until the host
        // writes a valid config.
        motion_cfg_make_defaults(&g_cfg);
        g_cfg_defaulted = true;
    }

    g_cfg_loaded = true;
}

// ------------------- Public API -------------------

const motion_cfg_t *motion_cfg_get(void)
{
    motion_cfg_ensure_loaded();
    return &g_cfg;
}

bool motion_cfg_is_defaulted(void)
{
    motion_cfg_ensure_loaded();
    return g_cfg_defaulted;
}

const char *motion_cfg_get_json_ptr(void)
{
    motion_cfg_ensure_loaded();
    return g_cfg.json;
}

const char *motion_cfg_result_name(motion_cfg_result_t r)
{
    switch (r) {
    case MOTION_CFG_OK:              return "ok";
    case MOTION_CFG_ERR_PARAM:       return "empty payload";
    case MOTION_CFG_ERR_TOO_LONG:    return "too long";
    case MOTION_CFG_ERR_NOT_STRICT:  return "not a strict JSON object";
    case MOTION_CFG_ERR_SAFETY_KEY:  return "safety parameter is firmware-owned";
    case MOTION_CFG_ERR_UNKNOWN_KEY: return "unknown key";
    case MOTION_CFG_ERR_RANGE:       return "value out of range";
    case MOTION_CFG_ERR_FLASH:       return "flash write failed";
    default:                         return "unknown";
    }
}

const char *motion_cfg_last_refusal(void)
{
    return g_cfg_last_refusal;
}

/* Why a host JSON cannot be stored, or MOTION_CFG_OK. Fills
 * g_cfg_last_refusal and reports to the host. */
static motion_cfg_result_t motion_cfg_check_json(const char *json, size_t len)
{
    g_cfg_last_refusal[0] = '\0';

    /* Refuse, rather than store, a config the boot-time apply could not parse:
     * one too long for the page (it used to be truncated to fit) or one that is
     * not a strict-JSON object (CFW-12, CFW-15). */
    if (len == 0U || len >= MOTION_CFG_JSON_MAX) {
        (void)snprintf(g_cfg_last_refusal, sizeof g_cfg_last_refusal,
                       "%u bytes (max %u)", (unsigned)len, (unsigned)(MOTION_CFG_JSON_MAX - 1U));
        return MOTION_CFG_ERR_TOO_LONG;
    }
    int r = motion_cfg_tokenize(json, len);
    if (r < 1) {
        return MOTION_CFG_ERR_NOT_STRICT;
    }

    /* Every key must be allowed and every value in range. A safety key is
     * refused outright (not dropped): the host must not believe it set it. */
    cfg_check_t c = motion_cfg_check_tokens(json, g_cfg_tokens, r, motion_cfg_safety_keys_writable(),
                                            g_cfg_last_refusal, sizeof g_cfg_last_refusal);
    switch (c) {
    case CFG_CHECK_OK:          return MOTION_CFG_OK;
    case CFG_CHECK_SAFETY_KEY:  return MOTION_CFG_ERR_SAFETY_KEY;
    case CFG_CHECK_UNKNOWN_KEY: return MOTION_CFG_ERR_UNKNOWN_KEY;
    case CFG_CHECK_BAD_VALUE:   return MOTION_CFG_ERR_RANGE;
    default:                    return MOTION_CFG_ERR_NOT_STRICT;
    }
}

motion_cfg_result_t motion_cfg_set_json(const char *json, size_t len)
{
    if (json == NULL) {
        return MOTION_CFG_ERR_PARAM;
    }

    motion_cfg_ensure_loaded();

    motion_cfg_result_t res = motion_cfg_check_json(json, len);
    if (res != MOTION_CFG_OK) {
        char m[MQ_MAX_MSG_SIZE];
        int n = snprintf(m, sizeof m,
                         "{\"type\":\"system\",\"state\":\"error\",\"msg\":\"config write refused: %s\",\"key\":\"%.40s\"}",
                         motion_cfg_result_name(res), g_cfg_last_refusal);
        if (n > 0 && (size_t)n < sizeof m) {
            (void)mq_push(m, (size_t)n);
        }
        printf("ERROR: motion config write refused: %s (%s); not stored\r\n",
               motion_cfg_result_name(res), g_cfg_last_refusal);
        return res;
    }

    memcpy(g_cfg.json, json, len);
    g_cfg.json[len] = '\0';
    g_cfg.version = MOTION_VER;

    // Persist. A verified write ends the defaulted state (CFW-03).
    if (motion_cfg_writeback() != HAL_OK) {
        return MOTION_CFG_ERR_FLASH;
    }
    g_cfg_defaulted = false;
    return MOTION_CFG_OK;
}

HAL_StatusTypeDef motion_cfg_wire_read(const uint8_t **out_buf,
                                       uint16_t *out_len,
                                       uint16_t max_payload_len)
{
    if (out_buf == NULL || out_len == NULL) {
        return HAL_ERROR;
    }

    motion_cfg_ensure_loaded();

    if (max_payload_len > (uint16_t)sizeof(g_cfg_wire_buf)) {
        max_payload_len = (uint16_t)sizeof(g_cfg_wire_buf);
    }

    if (max_payload_len < (uint16_t)sizeof(motion_cfg_wire_hdr_t)) {
        return HAL_ERROR;
    }

    motion_cfg_wire_hdr_t hdr;
    hdr.magic = g_cfg.magic;
    hdr.version = MOTION_WIRE_VER; // wire format, not the stored schema
    hdr.seq = g_cfg.seq;
    hdr.crc = g_cfg.crc;

    const uint16_t max_json = (uint16_t)(max_payload_len - sizeof(motion_cfg_wire_hdr_t));

    size_t json_total = strnlen(g_cfg.json, MOTION_CFG_JSON_MAX);
    // include '\0' if present/space allows
    if (json_total < MOTION_CFG_JSON_MAX) {
        json_total += 1U;
    }

    uint16_t json_len = (uint16_t)((json_total > max_json) ? max_json : json_total);
    hdr.json_len = json_len;

    memcpy(g_cfg_wire_buf, &hdr, sizeof(hdr));
    if (json_len > 0U) {
        memcpy(&g_cfg_wire_buf[sizeof(hdr)], g_cfg.json, json_len);
    }

    *out_buf = g_cfg_wire_buf;
    *out_len = (uint16_t)(sizeof(hdr) + json_len);
    return HAL_OK;
}

/* Length of the JSON text without a trailing NUL the host may have included. */
static size_t motion_cfg_text_len(const uint8_t *p, size_t len)
{
    while (len > 0U && p[len - 1U] == '\0') {
        len--;
    }
    return len;
}

motion_cfg_result_t motion_cfg_wire_write(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0U) {
        return MOTION_CFG_ERR_PARAM;
    }

    // Try to parse full wire format first.
    if (len >= (uint16_t)sizeof(motion_cfg_wire_hdr_t)) {
        motion_cfg_wire_hdr_t hdr;
        memcpy(&hdr, buf, sizeof(hdr));

        uint32_t expected_magic = MOTION_MAGIC;
        uint32_t expected_ver = MOTION_WIRE_VER;

        if (hdr.magic == expected_magic && hdr.version == expected_ver) {
            uint32_t total = (uint32_t)sizeof(motion_cfg_wire_hdr_t) + (uint32_t)hdr.json_len;
            if (total <= (uint32_t)len) {
                const uint8_t *json_ptr = &buf[sizeof(motion_cfg_wire_hdr_t)];
                return motion_cfg_set_json((const char *)json_ptr,
                                           motion_cfg_text_len(json_ptr, (size_t)hdr.json_len));
            }
        }
    }

    // Fallback: treat payload as raw JSON bytes.
    return motion_cfg_set_json((const char *)buf, motion_cfg_text_len(buf, (size_t)len));
}

HAL_StatusTypeDef motion_cfg_snapshot(motion_cfg_t *out)
{
    if (out == NULL) {
        return HAL_ERROR;
    }

    motion_cfg_ensure_loaded();
    memcpy(out, &g_cfg, sizeof(motion_cfg_t));
    return HAL_OK;
}

HAL_StatusTypeDef motion_cfg_save(const motion_cfg_t *new_cfg)
{
    if (new_cfg == NULL) {
        return HAL_ERROR;
    }
    motion_cfg_ensure_loaded();

    // The caller's JSON goes through the same policy as a host write; its
    // seq/crc/magic/version are regenerated.
    size_t len = strnlen(new_cfg->json, MOTION_CFG_JSON_MAX);
    return (motion_cfg_set_json(new_cfg->json, len) == MOTION_CFG_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef motion_cfg_commit(void)
{
    motion_cfg_ensure_loaded();
    // Write current g_cfg (useful if caller directly edited *motion_cfg_get()).
    return motion_cfg_writeback();
}

HAL_StatusTypeDef motion_cfg_factory_reset(void)
{
    motion_cfg_ensure_loaded();

    motion_cfg_make_defaults(&g_cfg);
    return motion_cfg_writeback();
}
