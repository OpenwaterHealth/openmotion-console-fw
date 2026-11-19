/*
 * odometer.c
 *
 *  Created on: Apr 29, 2026
 *      Author: Claude
 *
 * Storage backend: external 24AA025E48 I2C EEPROM (see odometer.h and
 * eeprom_24aa025.h). Persists a single combined {system,laser} record into a
 * wear-levelling ring of ODO_RING_SLOTS page-sized slots; each persist writes
 * the next slot, so writes spread evenly across the device.
 */

#include "odometer.h"
#include "eeprom_24aa025.h"
#include "trigger.h"
#include <stdio.h>
#include <string.h>

/* Static odometer data */
static SystemOdometer_t system_odo;
static LaserOdometer_t laser_odo;
static bool odometer_initialized = false;

/* True only between Odometer_Scan_Start() and the first Odometer_Scan_Finish()
 * that follows it. Trigger_Stop() runs on every stop path — STOP_TRIG, USB
 * disconnect, host port close, safety trips — so Scan_Finish must be a no-op
 * when no scan is in flight: the lsync counter isn't reset on stop, and a
 * stale-counter delta would re-add the previous scan's pulses. */
static bool scan_active = false;

/* EEPROM device + ring cursor. s_slot is the index of the newest record on the
 * device (-1 until the first write). s_seq is that record's sequence number. */
static eeprom_dev_t s_eeprom;
static int          s_slot = -1;
static uint16_t     s_seq  = 0;

/**
 * @brief CRC16-CCITT (poly 0x1021, init 0xFFFF). Bit-by-bit — records are tiny
 * and written infrequently, so a table isn't worth the footprint.
 */
static uint16_t odo_crc16(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/** @brief EEPROM byte address of ring slot i. */
static uint8_t odo_slot_addr(int i)
{
    return (uint8_t)(ODO_RING_BASE + (uint32_t)i * ODO_RECORD_SIZE);
}

/** @brief Populate a record with current values + magic + CRC. */
static void odo_pack(OdoRecord_t *rec, uint16_t seq,
                     uint32_t minutes, uint32_t pulses)
{
    rec->magic = ODO_RECORD_MAGIC;
    rec->seq = seq;
    rec->total_minutes = minutes;
    rec->total_pulses = pulses;
    rec->reserved = 0;
    rec->crc16 = odo_crc16(rec, offsetof(OdoRecord_t, crc16));
}

/** @brief True if a record read from the EEPROM has valid magic + CRC. */
static bool odo_record_valid(const OdoRecord_t *rec)
{
    if (rec->magic != ODO_RECORD_MAGIC) {
        return false;
    }
    return rec->crc16 == odo_crc16(rec, offsetof(OdoRecord_t, crc16));
}

/**
 * @brief Write the current in-memory totals to the next ring slot, advancing
 * the wear-levelling cursor. Each call lands on a fresh slot (mod ring size).
 */
static HAL_StatusTypeDef odo_persist(void)
{
    if (!s_eeprom.present) {
        return HAL_ERROR;
    }

    int next_slot = (s_slot < 0) ? 0 : ((s_slot + 1) % ODO_RING_SLOTS);
    uint16_t next_seq = (uint16_t)(s_seq + 1u);

    OdoRecord_t rec;
    odo_pack(&rec, next_seq, system_odo.total_minutes, laser_odo.total_pulses);

    HAL_StatusTypeDef status = EEPROM_Write(&s_eeprom, odo_slot_addr(next_slot),
                                            (uint8_t *)&rec, sizeof(rec));
    if (status != HAL_OK) {
        printf("Odometer: EEPROM write to slot %d failed (%d)\r\n", next_slot, status);
        return status;
    }

    s_slot = next_slot;
    s_seq = next_seq;
    return HAL_OK;
}

/**
 * @brief Initialize odometer module: detect the EEPROM, then load the newest
 * valid record from the ring. If no valid record exists (fresh device, foreign
 * bytes), starts both odometers at zero and writes an initial record so the
 * next boot reads clean.
 */
HAL_StatusTypeDef Odometer_Init(void)
{
    /* Zero in-memory state up front so a partial init still leaves sane values. */
    memset(&system_odo, 0, sizeof(system_odo));
    memset(&laser_odo, 0, sizeof(laser_odo));
    s_slot = -1;
    s_seq = 0;
    odometer_initialized = false;
    scan_active = false;

    if (EEPROM_Init(&s_eeprom) != HAL_OK) {
        printf("Odometer: EEPROM not present — odometers disabled\r\n");
        return HAL_ERROR;
    }

    /* Log the factory EUI-48 (also a free per-board serial number). */
    uint8_t eui[EEPROM_EUI48_LEN];
    if (EEPROM_ReadEUI48(&s_eeprom, eui) == HAL_OK) {
        printf("EEPROM EUI-48: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
               eui[0], eui[1], eui[2], eui[3], eui[4], eui[5]);
    }

    /* Scan every slot; keep the valid record with the newest (wrapping) seq. */
    bool found = false;
    OdoRecord_t best;
    int best_slot = -1;
    for (int i = 0; i < (int)ODO_RING_SLOTS; ++i) {
        OdoRecord_t rec;
        if (EEPROM_Read(&s_eeprom, odo_slot_addr(i), (uint8_t *)&rec, sizeof(rec)) != HAL_OK) {
            continue;
        }
        if (!odo_record_valid(&rec)) {
            continue;
        }
        if (!found || (int16_t)(rec.seq - best.seq) > 0) {
            best = rec;
            best_slot = i;
            found = true;
        }
    }

    if (found) {
        system_odo.total_minutes = best.total_minutes;
        laser_odo.total_pulses = best.total_pulses;
        s_slot = best_slot;
        s_seq = best.seq;
        odometer_initialized = true;
    } else {
        /* Fresh device: write an initial zero record so the next boot reads
         * clean. Best-effort — a failure is survivable (we retry next boot). */
        printf("Odometer: no valid EEPROM record — initializing to zero\r\n");
        odometer_initialized = true;  /* allow odo_persist() to run */
        if (odo_persist() != HAL_OK) {
            printf("Odometer: failed to write initial EEPROM record\r\n");
        }
    }

    system_odo.last_update_tick = HAL_GetTick();
    laser_odo.scan_start_pulses = 0;

    printf("Odometers initialized: System=%lu min, Laser=%lu pulses (slot %d, seq %u)\r\n",
           (unsigned long)system_odo.total_minutes,
           (unsigned long)laser_odo.total_pulses, s_slot, (unsigned)s_seq);
    return HAL_OK;
}

/**
 * @brief Update system odometer (call periodically).
 * Accumulates minutes since last persist; when the configured update interval
 * has elapsed, writes the next ring slot.
 */
HAL_StatusTypeDef Odometer_Update_System(void)
{
    if (!odometer_initialized) {
        return HAL_ERROR;
    }

    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed_ms;

    /* Handle tick overflow (every ~49.7 days) */
    if (current_tick >= system_odo.last_update_tick) {
        elapsed_ms = current_tick - system_odo.last_update_tick;
    } else {
        elapsed_ms = (0xFFFFFFFF - system_odo.last_update_tick) + current_tick + 1;
    }

    /* Persist on the configured interval (default 2 min — see odometer.h). */
    if (elapsed_ms >= SYSTEM_ODO_UPDATE_INTERVAL_MS) {
        uint32_t minutes_to_add = elapsed_ms / (60UL * 1000UL);
        system_odo.total_minutes += minutes_to_add;
        system_odo.last_update_tick = current_tick;

        HAL_StatusTypeDef status = odo_persist();
        if (status != HAL_OK) {
            return status;
        }
        printf("System odometer persisted: %lu minutes\r\n",
               (unsigned long)system_odo.total_minutes);
    }

    return HAL_OK;
}

/**
 * @brief Called when a scan starts. Records the current laser pulse count.
 */
HAL_StatusTypeDef Odometer_Scan_Start(void)
{
    if (!odometer_initialized) {
        return HAL_ERROR;
    }
    laser_odo.scan_start_pulses = get_lsync_pulse_count();
    scan_active = true;
    printf("Scan started, laser pulses at start: %lu\r\n",
           (unsigned long)laser_odo.scan_start_pulses);
    return HAL_OK;
}

/**
 * @brief Called when a scan finishes. Calculates pulses during the scan and
 * persists the updated total.
 */
HAL_StatusTypeDef Odometer_Scan_Finish(void)
{
    if (!odometer_initialized) {
        return HAL_ERROR;
    }
    if (!scan_active) {
        return HAL_OK;
    }
    scan_active = false;

    uint32_t current_pulses = get_lsync_pulse_count();
    uint32_t scan_pulses;
    if (current_pulses >= laser_odo.scan_start_pulses) {
        scan_pulses = current_pulses - laser_odo.scan_start_pulses;
    } else {
        /* Counter overflow (unlikely but possible) */
        scan_pulses = (0xFFFFFFFF - laser_odo.scan_start_pulses) + current_pulses + 1;
    }

    laser_odo.total_pulses += scan_pulses;
    printf("Scan finished, pulses this scan: %lu, total: %lu\r\n",
           (unsigned long)scan_pulses, (unsigned long)laser_odo.total_pulses);

    HAL_StatusTypeDef status = odo_persist();
    if (status != HAL_OK) {
        return status;
    }

    laser_odo.scan_start_pulses = 0;
    return HAL_OK;
}

/**
 * @brief Reset one or both odometers to zero and persist.
 */
HAL_StatusTypeDef Odometer_Reset(OdoResetTarget target)
{
    if (!odometer_initialized) {
        return HAL_ERROR;
    }

    switch (target) {
        case ODO_RESET_SYSTEM:
            system_odo.total_minutes = 0;
            system_odo.last_update_tick = HAL_GetTick();
            printf("Odometer reset: system\r\n");
            break;
        case ODO_RESET_LASER:
            laser_odo.total_pulses = 0;
            laser_odo.scan_start_pulses = 0;
            printf("Odometer reset: laser\r\n");
            break;
        case ODO_RESET_BOTH:
            system_odo.total_minutes = 0;
            system_odo.last_update_tick = HAL_GetTick();
            laser_odo.total_pulses = 0;
            laser_odo.scan_start_pulses = 0;
            printf("Odometer reset: both\r\n");
            break;
        default:
            return HAL_ERROR;
    }

    return odo_persist();
}

/**
 * @brief Get current system uptime in minutes.
 * Returns the persisted total plus any accumulated time since the last save,
 * so the value advances smoothly between persists.
 */
uint32_t Odometer_Get_System_Minutes(void)
{
    if (!odometer_initialized) {
        return 0;
    }

    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed_ms;
    if (current_tick >= system_odo.last_update_tick) {
        elapsed_ms = current_tick - system_odo.last_update_tick;
    } else {
        elapsed_ms = (0xFFFFFFFF - system_odo.last_update_tick) + current_tick + 1;
    }

    uint32_t additional_minutes = elapsed_ms / (60UL * 1000UL);
    return system_odo.total_minutes + additional_minutes;
}

/**
 * @brief Get total laser pulses.
 */
uint32_t Odometer_Get_Laser_Pulses(void)
{
    if (!odometer_initialized) {
        return 0;
    }
    return laser_odo.total_pulses;
}

/**
 * @brief Read the EEPROM's factory EUI-48 (unique per-board serial).
 */
HAL_StatusTypeDef Odometer_Get_EUI48(uint8_t eui[6])
{
    if (!s_eeprom.present || eui == NULL) {
        return HAL_ERROR;
    }
    return EEPROM_ReadEUI48(&s_eeprom, eui);
}
