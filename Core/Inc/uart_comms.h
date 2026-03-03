/*
 * uart_comms.h
 *
 *  Created on: May 13, 2024
 *      Author: gvigelet
 */

#ifndef INC_UART_COMMS_H_
#define INC_UART_COMMS_H_

#include "main.h"  // This should contain your HAL includes and other basic includes.
#include "common.h"
#include "lwrb.h"
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>

/* ---------------------------------------------------------------------------
 * Telemetry ring buffer
 * ---------------------------------------------------------------------------*/

/** Polling interval used when no hardware timer tick is pending (ms). */
#define TELEMETRY_POLL_INTERVAL_MS  25U

/** Number of TelemetrySamples held in the ring buffer. */
#define TELEMETRY_RING_SAMPLES      64U

#define ADC_MAX 4095.0f   // 12-bit
#define V_REF 3.30f 

/** One snapshot of system telemetry collected on each poll event. */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;   /*!< HAL_GetTick() at capture time */
    uint32_t acq_time_us;    /*!< Time spent acquiring sensor data (µs) */
    float    t1;             /*!< MAX31875 sensor 1 temperature (°C) */
    float    t2;             /*!< MAX31875 sensor 2 temperature (°C) */
    float    t3;             /*!< MAX31875 sensor 3 temperature (°C) */
    uint16_t tec_adc[4];     /*!< ADS7924 channels 0-3 (raw 12-bit codes) */
    bool     tec_status;     /* TEC Status */
} TelemetrySample;

/* ---------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------------*/

void comms_init(void);
void comms_process(void);
void comms_handle_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size);
void comms_handle_TxCallback(UART_HandleTypeDef *huart);
void comms_handle_ErrorCallback(UART_HandleTypeDef *huart);

void CDC_handle_TxCpltCallback();
void printUartPacket(const UartPacket* packet);

/**
 * @brief  Signal a telemetry poll from a hardware timer ISR.
 *         The actual I2C sensor reads happen in telemetry_poll() called
 *         from the main loop, keeping ISRs short.
 */
void comms_telemetry_tick(void);

/**
 * @brief  Call from the main loop — reads sensors if the timer tick fired
 *         or TELEMETRY_POLL_INTERVAL_MS has elapsed.  Non-blocking.
 */
void telemetry_poll(void);

/** @brief Returns the number of complete TelemetrySamples available to read. */
size_t telemetry_available(void);

/**
 * @brief  Reads up to @p count samples into @p out.
 * @return Number of samples actually read.
 */
size_t telemetry_read(TelemetrySample *out, size_t count);

#endif /* INC_UART_COMMS_H_ */
