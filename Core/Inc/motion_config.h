#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include "stm32h7xx_hal.h"
#include "flash_eeprom.h"
#include "memory_map.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Magic/version identifiers stored in flash
#define MOTION_MAGIC   (0x4D4F5449UL)  // 'MOTI'
// Stored record schema. v2 (CFW-02): the CRC covers the whole record, JSON
// included. v1 records (CRC over magic/version/seq only) are migrated to v2
// at load. Bump if the layout or the CRC coverage changes.
#define MOTION_VER     (0x00020000UL)
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

// Upper bound on the jsmn tokens needed to tokenize the config JSON in
// motion_cfg_apply_settings(). A JSON object with N keys needs 1 + 2*N tokens.
// The config carries the safety params (TEC_TRIP, OPT/EE thresholds and gains)
// plus the writable FPGA register keys from fpga_register_map.c -- ~75 keys
// today (~151 tokens). 256 leaves headroom for schema growth.
//
// Sized deliberately: a previous fixed value of 32 was silently too small for a
// real (~1 KB) config, so jsmn returned JSMN_ERROR_NOMEM and the firmware
// skipped applying settings -- leaving TEC_TRIP_VALUE at 0 and disarming the TEC
// thermal trip. If this bound is ever exceeded the parse now fails loudly
// (console + host error) rather than silently. Guarded by
// host_tests/test_motion_cfg_tokens.c.
#define MOTION_CFG_MAX_JSON_TOKENS  256U

// Persistent config blob that exactly fills one flash page.
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t magic;        // MOTION_MAGIC
    uint32_t version;      // MOTION_VER
    uint32_t seq;          // monotonic counter

    uint16_t crc;          // v2: CRC16-CCITT over the whole record except this field
    uint8_t  reserved;
    uint8_t  reserved2;

    char     json[MOTION_CFG_JSON_MAX]; // NUL-terminated text blob
} motion_cfg_t;

// Sanity checks for layout
_Static_assert(sizeof(motion_cfg_t) == MOTION_CFG_PAGE_SIZE,
               "motion_cfg_t must fill one 128KB flash page");
_Static_assert((sizeof(motion_cfg_t) % 4U) == 0U,
               "motion_cfg_t size must be 32-bit word aligned for flash writes");

// ======================== PUBLIC API ========================

// Returns pointer to the live in-RAM copy of the config.
// On first call, it will load from flash, validate magic/version/CRC,
// and if invalid it writes factory defaults to flash and returns that.
const motion_cfg_t *motion_cfg_get(void);

// Copies the current config into *out so you can edit it offline.
// Example:
//    motion_cfg_t work;
//    motion_cfg_snapshot(&work);
//    work.hv_settng = 123;
//    motion_cfg_save(&work);
HAL_StatusTypeDef motion_cfg_snapshot(motion_cfg_t *out);

// Saves a modified struct to flash.
// - You pass in a struct you edited (typically from motion_cfg_snapshot).
// - We copy fields we care about into internal storage,
//   bump seq, recalc CRC, erase/program flash.
HAL_StatusTypeDef motion_cfg_save(const motion_cfg_t *new_cfg);

// Commits the *current* live config (as returned by motion_cfg_get())
// back to flash. This is only needed if you directly mutate *motion_cfg_get().
HAL_StatusTypeDef motion_cfg_commit(void);

// Restores factory defaults and writes them to flash.
HAL_StatusTypeDef motion_cfg_factory_reset(void);

// Convenience helpers for working with the JSON blob.
// - motion_cfg_get_json_ptr() returns a pointer into the live config (NUL-terminated).
// - motion_cfg_set_json() persists the provided JSON text into flash.
const char *motion_cfg_get_json_ptr(void);
HAL_StatusTypeDef motion_cfg_set_json(const char *json, size_t len);

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

// Applies a serialized buffer to config and writes it to flash.
// Accepts either full wire buffer (header+json) or raw JSON bytes.
HAL_StatusTypeDef motion_cfg_wire_write(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // MOTION_CONFIG_H
