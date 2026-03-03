/*
 * uart_comms.c
 *
 *  Created on: Mar 11, 2024
 *      Author: gvigelet
 */

#include "main.h"
#include "uart_comms.h"
#include "utils.h"
#include "usbd_cdc_if.h"
#include "tca9548a.h"
#include "trigger.h"
#include "fan_driver.h"
#include "ads7828.h"
#include "ad5761r.h"
#include "ads7924.h"
#include "max31875.h"
#include "led_driver.h"
#include "if_commands.h"

#include "lwrb.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

#define PDU_N 16

/* ---------------------------------------------------------------------------
 * Telemetry ring buffer — backing store and runtime state
 * ---------------------------------------------------------------------------*/
#define TELEMETRY_BUF_BYTES  (TELEMETRY_RING_SAMPLES * sizeof(TelemetrySample))
static uint8_t          s_rb_storage[TELEMETRY_BUF_BYTES];
static lwrb_t           s_telemetry_rb;
static volatile uint8_t s_telemetry_tick = 0; /* set by timer ISR */
static uint32_t         s_last_poll_ms   = 0;

// Private variables
extern uint8_t rxBuffer[COMMAND_MAX_SIZE];
extern uint8_t txBuffer[COMMAND_MAX_SIZE];
extern ADS7924_HandleTypeDef tec_ads;
extern ADS7828_HandleTypeDef adc_mon[2];

volatile uint32_t ptrReceive;
volatile uint8_t rx_flag = 0;
volatile uint8_t tx_flag = 0;
volatile uint8_t tx_busy = 0;

/* consoleTemps is owned by the command-processing module */
extern ConsoleTemperatures consoleTemps;

extern FAN_Driver fan;
extern bool _enter_dfu;

extern ad5761r_dev tec_dac;
extern double temp_trip_value;

// Local helper used for sending
void printUartPacket(const UartPacket* packet) {
	if (!packet) {
		printf("Invalid packet (NULL pointer).\n");
		return;
	}

	printf("UartPacket:\r\n");
	printf("  ID: %u\r\n", packet->id);
	printf("  Packet Type: 0x%02X\r\n", packet->packet_type);
	printf("  Command: 0x%02X\r\n", packet->command);
	printf("  Address: 0x%02X\r\n", packet->addr);
	printf("  Reserved: 0x%02X\r\n", packet->reserved);
	printf("  Data Length: %u\r\n", packet->data_len);

	printf("  Data: ");
	if (packet->data && packet->data_len > 0) {
		for (uint16_t i = 0; i < packet->data_len; ++i) {
			printf("0x%02X ", packet->data[i]);
		}
		printf("\r\n");
	} else {
		printf("No data\r\n");
	}

	printf("  CRC: 0x%04X\r\n\r\n", packet->crc);
}

static void UART_INTERFACE_SendDMA(UartPacket* pResp)
{
	if (!pResp) return;
	// Wait for previous transmission to complete
	while (tx_busy) {
		HAL_Delay(1);
	}
	memset(txBuffer, 0, sizeof(txBuffer));
	int bufferIndex = 0;

	txBuffer[bufferIndex++] = OW_START_BYTE;
	txBuffer[bufferIndex++] = pResp->id >> 8;
	txBuffer[bufferIndex++] = pResp->id & 0xFF;
	txBuffer[bufferIndex++] = pResp->packet_type;
	txBuffer[bufferIndex++] = pResp->command;
	txBuffer[bufferIndex++] = pResp->addr;
	txBuffer[bufferIndex++] = pResp->reserved;
	txBuffer[bufferIndex++] = (pResp->data_len) >> 8;
	txBuffer[bufferIndex++] = (pResp->data_len) & 0xFF;
	if (pResp->data_len > 0) {
		memcpy(&txBuffer[bufferIndex], pResp->data, pResp->data_len);
		bufferIndex += pResp->data_len;
	}
	uint16_t crc = util_crc16(&txBuffer[1], pResp->data_len + 8);
	txBuffer[bufferIndex++] = crc >> 8;
	txBuffer[bufferIndex++] = crc & 0xFF;

	txBuffer[bufferIndex++] = OW_END_BYTE;

	tx_flag = 0;
	tx_busy = 1;
	CDC_Transmit_FS(txBuffer, bufferIndex);
	// Wait for transmit complete (blocking). In no-OS designs this is typical from main loop
	while (!tx_flag) {
		HAL_Delay(1);
	}
	tx_busy = 0;
}

// Process one received packet if available. Call this periodically from main loop.
void comms_process(void)
{
	if (!rx_flag) return;

	UartPacket cmd = {0};
	UartPacket resp;
	uint16_t calculated_crc;
	int bufferIndex = 0;

	if (rxBuffer[bufferIndex++] != OW_START_BYTE) {
		resp.id = cmd.id;
		resp.data_len = 0;
		resp.packet_type = OW_NAK;
		goto NextDataPacket;
	}

	cmd.id = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
	bufferIndex += 2;
	cmd.packet_type = rxBuffer[bufferIndex++];
	cmd.command = rxBuffer[bufferIndex++];
	cmd.addr = rxBuffer[bufferIndex++];
	cmd.reserved = rxBuffer[bufferIndex++];

	// Extract payload length
	cmd.data_len = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
	bufferIndex += 2;

	// Check if data length is valid
	if (cmd.data_len > COMMAND_MAX_SIZE - bufferIndex && rxBuffer[COMMAND_MAX_SIZE-1] != OW_END_BYTE) {
		resp.id = cmd.id;
		resp.addr = 0;
		resp.reserved = 0;
		resp.data_len = 0;
		resp.packet_type = OW_NAK;
		goto NextDataPacket;
	}

	// Extract data pointer
	cmd.data = &rxBuffer[bufferIndex];
	if (cmd.data_len > COMMAND_MAX_SIZE) {
		bufferIndex = COMMAND_MAX_SIZE-3; // [3 bytes from the end should be the crc for a continuation packet]
	} else {
		bufferIndex += cmd.data_len; // move pointer to end of data
	}

	// Extract received CRC
	cmd.crc = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
	bufferIndex += 2;

	// Calculate CRC for received data
	if (cmd.data_len > COMMAND_MAX_SIZE) {
		calculated_crc = util_crc16(&rxBuffer[1], COMMAND_MAX_SIZE-3);
	} else {
		calculated_crc = util_crc16(&rxBuffer[1], cmd.data_len + 8);
	}

	// Check CRC
	if (cmd.crc != calculated_crc) {
		resp.id = cmd.id;
		resp.addr = 0;
		resp.reserved = 0;
		resp.data_len = 0;
		resp.packet_type = OW_BAD_CRC;
		goto NextDataPacket;
	}

	// Check end byte
	if (rxBuffer[bufferIndex++] != OW_END_BYTE) {
		resp.id = cmd.id;
		resp.data_len = 0;
		resp.addr = 0;
		resp.reserved = 0;
		resp.packet_type = OW_NAK;
		goto NextDataPacket;
	}

	process_if_command(&resp, &cmd);

NextDataPacket:
	UART_INTERFACE_SendDMA(&resp);
	memset(rxBuffer, 0, sizeof(rxBuffer));
	ptrReceive = 0;
	rx_flag = 0;
	// Restart reception
	CDC_ReceiveToIdle(rxBuffer, COMMAND_MAX_SIZE);
}

/* ---------------------------------------------------------------------------
 * Telemetry API
 * ---------------------------------------------------------------------------*/

/** Called from a hardware timer ISR — just sets a flag, no I2C in the ISR. */
void comms_telemetry_tick(void)
{
	s_telemetry_tick = 1;
}

/** Returns the number of complete TelemetrySamples waiting in the ring buffer. */
size_t telemetry_available(void)
{
	return lwrb_get_full(&s_telemetry_rb) / sizeof(TelemetrySample);
}

/** Reads up to @p count samples into @p out; returns actual number read. */
size_t telemetry_read(TelemetrySample *out, size_t count)
{
	if (!out || !count) return 0;
	size_t avail   = lwrb_get_full(&s_telemetry_rb) / sizeof(TelemetrySample);
	size_t to_read = (count < avail) ? count : avail;
	return lwrb_read(&s_telemetry_rb, out, to_read * sizeof(TelemetrySample))
	       / sizeof(TelemetrySample);
}


static inline float adc_to_voltage(uint16_t adc_code)
{
    return (adc_code * ADC_REF) / ADC_MAX;
}

/**
 * @brief  Main-loop telemetry worker.
 *
 * Runs every TELEMETRY_POLL_INTERVAL_MS or immediately when
 * comms_telemetry_tick() has been called from a timer ISR.
 * Reads temperatures and TEC ADC, updates consoleTemps, then
 * pushes a TelemetrySample into the ring buffer.  Oldest samples
 * are silently discarded if the consumer falls behind.
 */
void telemetry_poll(void)
{
	uint32_t now = HAL_GetTick();

	/* skip unless the timer fired or the fallback interval has elapsed */
	if (!s_telemetry_tick &&
	    (now - s_last_poll_ms) < TELEMETRY_POLL_INTERVAL_MS) {
		return;
	}
	s_telemetry_tick = 0;
	s_last_poll_ms   = now;

	TelemetrySample sample = {0};
	sample.timestamp_ms = now;

	/* --- begin timed acquisition --- */
	uint32_t dwt_start = DWT->CYCCNT;

	/* Temperatures — sensors sit behind mux 1 channel 1 */
	TCA9548A_SelectChannel(1, 1);
	sample.t1 = MAX31875_ReadTemperature(MAX31875_TEMP1_DEV_ADDR);
	sample.t2 = MAX31875_ReadTemperature(MAX31875_TEMP2_DEV_ADDR);
	sample.t3 = MAX31875_ReadTemperature(MAX31875_TEMP3_DEV_ADDR);

	/* Keep the shared console-temp struct in sync */
	consoleTemps.f.t1 = sample.t1;
	consoleTemps.f.t2 = sample.t2;
	consoleTemps.f.t3 = sample.t3;

	/* TEC ADC — four channels on ads7924.
	 * Use aligned locals to avoid -Waddress-of-packed-member on the
	 * packed struct members, then copy into the sample. */
	uint16_t tec_raw[4] = {0};
	ADS7924_ReadRaw(&tec_ads, ADS7924_CH0, &tec_raw[0], 10);
	ADS7924_ReadRaw(&tec_ads, ADS7924_CH1, &tec_raw[1], 10);
	ADS7924_ReadRaw(&tec_ads, ADS7924_CH2, &tec_raw[2], 10);
	ADS7924_ReadRaw(&tec_ads, ADS7924_CH3, &tec_raw[3], 10);
	sample.tec_adc[0] = tec_raw[0];
	sample.tec_adc[1] = tec_raw[1];
	sample.tec_adc[2] = tec_raw[2];
	sample.tec_adc[3] = tec_raw[3];

	sample.tec_status = HAL_GPIO_ReadPin(TEMPGD_GPIO_Port, TEMPGD_Pin)?false:true;  // active low

	/* --- end timed acquisition --- */
	uint32_t dwt_cycles = DWT->CYCCNT - dwt_start;
	sample.acq_time_us  = dwt_cycles / (SystemCoreClock / 1000000U);

	/* Evict oldest sample if ring buffer is full */
	if (lwrb_get_free(&s_telemetry_rb) < sizeof(TelemetrySample)) {
		lwrb_skip(&s_telemetry_rb, sizeof(TelemetrySample));
	}
	lwrb_write(&s_telemetry_rb, &sample, sizeof(TelemetrySample));
}

void comms_init(void) {
	printf("Initilize comms (no-RTOS)\r\n");

	/* initialize console temps via the command module ownership */
	consoleTemps.f.t1 = 0;
	consoleTemps.f.t2 = 0;
	consoleTemps.f.t3 = 0;

	tx_busy = 0;
	tx_flag = 0;
	rx_flag = 0;

	/* initialize telemetry ring buffer */
	lwrb_init(&s_telemetry_rb, s_rb_storage, sizeof(s_rb_storage));
	s_telemetry_tick = 0;
	s_last_poll_ms   = HAL_GetTick();

	CDC_FlushRxBuffer_FS();
	CDC_ReceiveToIdle(rxBuffer, COMMAND_MAX_SIZE);
}

// Callback functions
void comms_handle_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t pos) {

    if (huart->Instance == USART1) {
        // Notify the task
    	rx_flag = 1;
    }
}

void CDC_handle_RxCpltCallback(uint16_t len) {
	rx_flag = 1;
}

void CDC_handle_TxCpltCallback() {
	tx_flag = 1;
}

void comms_handle_TxCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART1) {
		tx_flag = 1;
	}
}

void comms_handle_ErrorCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART1) {
        // Handle errors here. Maybe reset DMA reception, etc.
    }
}
