#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include "stm32h7xx_hal.h"
#include "flash_eeprom.h"
#include "memory_map.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Magic/version identifiers stored in flash
#define MOTION_MAGIC   (0x4D4F5449UL)  // 'MOTI'
// Stored record schema.
//   v1: CRC over magic/version/seq only.
//   v2 (CFW-02): CRC over the whole record, JSON included.
//   v3 (PTR-2026-1-5 host-writable safety state): the JSON holds OPERATIONAL
//       and OPAQUE keys only (motion_cfg_keys.c). The laser-safety parameters
//       (TEC_TRIP, the EE/OPT limits, the driver clamps) are compiled into the
//       firmware (safety_envelope.h) and are no longer part of the record. A v2
//       record is migrated at load: its operational keys are kept when in
//       range, its safety and unknown keys are dropped and logged, and it is
//       rewritten as v3. A v1 record is migrated the same way. A record that
//       cannot be migrated is treated as invalid (defaults, trigger refused),
//       never half-applied.
// Bump if the layout, the CRC coverage or the key policy changes.
#define MOTION_VER     (0x00030000UL)
#define MOTION_VER_V2  (0x00020000UL)
#define MOTION_VER_V1  (0x00010000UL)
// OW_CMD_USR_CFG wire-header version (SDK MotionConfig MOTION_VER). It
// describes the wire format, not the stored record, and stays 1.0.0 so hosts
// need no change.
#define MOTION_WIRE_VER (0x00010000UL)

// Flash layout info: one 2KB page at 0x0801F000
#define MOTION_CFG_PAGE_ADDR      (ADDR_FLASH_SECTOR_7_BANK2)
#define MOTION_CFG_PAGE_END       (ADDR_FLASH_END_ADDRESS)
#define MOTION_CFG_PAGE_SIZE      (2036U)

// magic(4) + version(4) + seq(4) + hv_settng(2) + hv_enabled(1) + auto_on(1) = 16 bytes
#define MOTION_CFG_HEADER_SIZE    (16U)


// The rest of the page is JSON storage (must include '\0'):
// 2048 - 16 - 2 = 2030
#define MOTION_CFG_JSON_MAX       (MOTION_CFG_PAGE_SIZE - MOTION_CFG_HEADER_SIZE)
// -> 2030 bytes

// Upper bound on the jsmn tokens needed to tokenize the config JSON. A JSON
// object with N keys needs 1 + 2*N tokens; the opaque "calibration" block the
// application stores (nested arrays of camera coefficients) needs one token
// per element. 256 covers the WI-00015 key set plus that block with headroom.
//
// Sized deliberately: a previous fixed value of 32 was silently too small for a
// real (~1 KB) config, so jsmn returned JSMN_ERROR_NOMEM and the firmware
// skipped applying settings -- which, before v3, left TEC_TRIP_VALUE at 0 and
// disarmed the TEC thermal trip. The trip threshold no longer depends on the
// record (safety_envelope.h), and a parse failure now refuses the trigger and
// is reported. Guarded by host_tests/test_motion_cfg_tokens.c.
#define MOTION_CFG_MAX_JSON_TOKENS  256U

// Persistent config blob that exactly fills one flash page.
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t magic;        // MOTION_MAGIC
    uint32_t version;      // MOTION_VER
    uint32_t seq;          // monotonic counter

    uint16_t crc;          // v2+: CRC16-CCITT over the whole record except this field
    uint8_t  reserved;
    uint8_t  reserved2;

    char     json[MOTION_CFG_JSON_MAX]; // NUL-terminated text blob
} motion_cfg_t;

// Sanity checks for layout
_Static_assert(sizeof(motion_cfg_t) == MOTION_CFG_PAGE_SIZE,
               "motion_cfg_t must fill one 128KB flash page");
_Static_assert((sizeof(motion_cfg_t) % 4U) == 0U,
               "motion_cfg_t size must be 32-bit word aligned for flash writes");

// Result of a host configuration write. The command handler answers OW_ERROR
// with the matching OW_CODE_* byte (common.h) so the host learns why.
typedef enum {
    MOTION_CFG_OK = 0,
    MOTION_CFG_ERR_PARAM,        // NULL / empty payload
    MOTION_CFG_ERR_TOO_LONG,     // does not fit the record
    MOTION_CFG_ERR_NOT_STRICT,   // not a strict JSON object (CFW-15)
    MOTION_CFG_ERR_SAFETY_KEY,   // names a firmware-owned safety parameter
    MOTION_CFG_ERR_UNKNOWN_KEY,  // key not in the allow-list
    MOTION_CFG_ERR_RANGE,        // value not a finite number or out of range
    MOTION_CFG_ERR_FLASH         // erase/program/verify failed
} motion_cfg_result_t;

// ======================== PUBLIC API ========================

// Returns pointer to the live in-RAM copy of the config.
// On first call, it will load from flash, validate magic/version/CRC,
// migrate an older record, and if invalid run on defaults in RAM.
const motion_cfg_t *motion_cfg_get(void);
// True while no valid stored record was found at load (first boot, erased
// sector or corruption) and the console runs on the built-in defaults.
// Cleared by a successful, verified host config write. Trigger_Start()
// refuses while it is set (CFW-03).
bool motion_cfg_is_defaulted(void);

// Copies the current config into *out so you can edit it offline.
HAL_StatusTypeDef motion_cfg_snapshot(motion_cfg_t *out);

// Saves a modified struct to flash (JSON is re-validated like a host write).
HAL_StatusTypeDef motion_cfg_save(const motion_cfg_t *new_cfg);

// Commits the *current* live config (as returned by motion_cfg_get())
// back to flash. This is only needed if you directly mutate *motion_cfg_get().
HAL_StatusTypeDef motion_cfg_commit(void);

// Restores factory defaults and writes them to flash.
HAL_StatusTypeDef motion_cfg_factory_reset(void);

// Convenience helpers for working with the JSON blob.
// - motion_cfg_get_json_ptr() returns a pointer into the live config (NUL-terminated).
// - motion_cfg_set_json() validates and persists the provided JSON text.
const char *motion_cfg_get_json_ptr(void);
motion_cfg_result_t motion_cfg_set_json(const char *json, size_t len);

// Text for a result, for the console log and the host message.
const char *motion_cfg_result_name(motion_cfg_result_t r);
// The key and reason of the last refused write ("KEY: reason"), for the log.
const char *motion_cfg_last_refusal(void);

// ======================== WIRE FORMAT (UART/USB) ========================
// When sending config over the command interface, we serialize as:
//   [motion_cfg_wire_hdr_t][json bytes (json_len)]
// For WRITE, you may send either:
//   - full wire buffer (header+json), OR
//   - raw JSON bytes (no header)
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t seq;
    uint16_t crc;
    uint16_t json_len; // number of JSON bytes included (may include trailing '\0')
} motion_cfg_wire_hdr_t;

// Produces a pointer to an internal buffer containing the serialized config.
// max_payload_len is the caller's maximum allowed payload size.
HAL_StatusTypeDef motion_cfg_wire_read(const uint8_t **out_buf,
                                       uint16_t *out_len,
                                       uint16_t max_payload_len);

// Validates a serialized buffer and writes it to flash.
// Accepts either full wire buffer (header+json) or raw JSON bytes.
motion_cfg_result_t motion_cfg_wire_write(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // MOTION_CONFIG_H
