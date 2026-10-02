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
#include "msg_queue.h"

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
/* Snapshot of the in-flight command. comms_process copies rxBuffer here and
 * re-arms reception immediately, so the next command can land in rxBuffer
 * while this one is still being processed/answered. */
static uint8_t procBuffer[COMMAND_MAX_SIZE];
extern ADS7924_HandleTypeDef tec_ads;
extern ADS7828_HandleTypeDef adc_mon[2];

volatile uint32_t ptrReceive;
volatile uint8_t rx_flag = 0;
volatile uint8_t tx_flag = 0;
volatile uint8_t tx_busy = 0;

/* Bound the blocking waits in UART_INTERFACE_SendDMA. tx_flag is set by the USB
 * CDC TX-complete callback, which never fires if the host vanished mid-transmit
 * — without a bound the superloop would hang here forever and the command
 * interface would stay dead until a power cycle. */
#ifndef TX_COMPLETE_TIMEOUT_MS
#define TX_COMPLETE_TIMEOUT_MS 100u
#endif

/* consoleTemps is owned by the command-processing module */
extern ConsoleTemperatures consoleTemps;

extern FAN_Driver fan;
extern bool _enter_dfu;

extern ad5761r_dev tec_dac;
extern double TEC_TRIP_VALUE;

volatile bool _tec_sample_lock = false;
volatile TecStats last_tec_stats = {0};

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

static void UART_INTERFACE_SendDMA(const UartPacket* pResp)
{
	if (!pResp) return;
	// Wait for previous transmission to complete — bounded so a USB cut that
	// left a prior TX stuck (tx-complete callback never fired) can't hang the
	// main loop forever.
	{
		uint32_t t0 = HAL_GetTick();
		while (tx_busy && (HAL_GetTick() - t0) < TX_COMPLETE_TIMEOUT_MS) {
			HAL_Delay(1);
		}
		tx_busy = 0; // give up on a stuck prior TX rather than block
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
	// Wait for transmit complete (blocking from the main loop in this no-OS
	// design) — bounded so an abrupt USB disconnect mid-transmit can't wedge
	// the superloop waiting on a tx-complete callback that will never fire.
	{
		uint32_t t0 = HAL_GetTick();
		while (!tx_flag && (HAL_GetTick() - t0) < TX_COMPLETE_TIMEOUT_MS) {
			HAL_Delay(1);
		}
	}
	tx_busy = 0;
}

// Process one received packet if available. Call this periodically from main loop.
void comms_process(void)
{
	if (!rx_flag) return;

	/* Snapshot the received frame, clear rxBuffer, and re-arm reception NOW —
	 * before processing/answering. The CDC RX only stores bytes while armed
	 * (read-to-idle); the old design re-armed only after the response was sent,
	 * so a command arriving in that window was silently dropped (which forced
	 * the host to pace commands ~10 ms apart). Re-arming up front lets the next
	 * command land in rxBuffer while we work on the snapshot. rx_flag is cleared
	 * before re-arming so a flag set for the next command can't be lost. */
	memcpy(procBuffer, rxBuffer, COMMAND_MAX_SIZE);
	memset(rxBuffer, 0, COMMAND_MAX_SIZE);
	ptrReceive = 0;
	rx_flag = 0;
	CDC_ReceiveToIdle(rxBuffer, COMMAND_MAX_SIZE);

	UartPacket cmd = {0};
	UartPacket resp;
	uint16_t calculated_crc;
	int bufferIndex = 0;

	if (procBuffer[bufferIndex++] != OW_START_BYTE) {
		resp.id = cmd.id;
		resp.data_len = 0;
		resp.packet_type = OW_NAK;
		goto NextDataPacket;
	}

	cmd.id = (procBuffer[bufferIndex] << 8 | (procBuffer[bufferIndex+1] & 0xFF ));
	bufferIndex += 2;
	cmd.packet_type = procBuffer[bufferIndex++];
	cmd.command = procBuffer[bufferIndex++];
	cmd.addr = procBuffer[bufferIndex++];
	cmd.reserved = procBuffer[bufferIndex++];

	// Extract payload length
	cmd.data_len = (procBuffer[bufferIndex] << 8 | (procBuffer[bufferIndex+1] & 0xFF ));
	bufferIndex += 2;

	// Check if data length is valid
	if (cmd.data_len > COMMAND_MAX_SIZE - bufferIndex && procBuffer[COMMAND_MAX_SIZE-1] != OW_END_BYTE) {
		resp.id = cmd.id;
		resp.addr = 0;
		resp.reserved = 0;
		resp.data_len = 0;
		resp.packet_type = OW_NAK;
		goto NextDataPacket;
	}

	// Extract data pointer
	cmd.data = &procBuffer[bufferIndex];
	if (cmd.data_len > COMMAND_MAX_SIZE) {
		bufferIndex = COMMAND_MAX_SIZE-3; // [3 bytes from the end should be the crc for a continuation packet]
	} else {
		bufferIndex += cmd.data_len; // move pointer to end of data
	}

	// Extract received CRC
	cmd.crc = (procBuffer[bufferIndex] << 8 | (procBuffer[bufferIndex+1] & 0xFF ));
	bufferIndex += 2;

	// Calculate CRC for received data
	if (cmd.data_len > COMMAND_MAX_SIZE) {
		calculated_crc = util_crc16(&procBuffer[1], COMMAND_MAX_SIZE-3);
	} else {
		calculated_crc = util_crc16(&procBuffer[1], cmd.data_len + 8);
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
	if (procBuffer[bufferIndex++] != OW_END_BYTE) {
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
	if(_tec_sample_lock){
        _tec_sample_lock = false;
	}
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
volatile uint32_t _trip_counter = 0;
volatile bool _trip_set = false;
/* Laser-safety (EE/OPT) trip edge guard — mirrors _trip_set for the TEC path.
 * Ensures the trip message is pushed once and the trigger stopped once per trip. */
volatile bool _laser_safety_trip_set = false;

/*
 * @brief  Evaluate the TEC over-voltage trip for one telemetry sample.
 *
 * Compares the TEC sense voltage (channel 0) against TEC_TRIP_VALUE and drives
 * the safety-disconnect / recovery edges: on the rising edge it opens the
 * safety disconnect and pushes a one-shot error message; it auto-clears after
 * 200 consecutive clean polls. Sets sample->tec_status. Extracted from
 * telemetry_poll to keep that function under the IEC 62304 complexity threshold.
 */
static void tec_trip_evaluate(TelemetrySample *sample)
{
	volatile float tec_volts = adc_to_voltage(sample->tec_adc[0]);
	if(TEC_TRIP_VALUE != 0.0 && tec_volts > TEC_TRIP_VALUE){
		// error
		_trip_counter = 0;
		sample->tec_status = false;
		Trigger_Safety_Disconnect();
		if(!_trip_set){
			/* push a system error JSON message into the message queue */
			const char *msg = "{\"type\": \"system\", \"state\": \"error\", \"msg\": \"TEC trip point reached\"}";
			if (!mq_push(msg, strlen(msg))) {
				printf("Failed to push trip message to queue\r\n");
			}
		}
		_trip_set = true;
		printf("++++> ERROR TEC TRIP SET\r\n");
	}else{
		_trip_counter++;
		if(_trip_counter > 200 && _trip_set){
			Trigger_Safety_Clear();
			_trip_set = false;
		}
		sample->tec_status = true;
	}
}

/*
 * @brief  Poll the EE/OPT laser-safety interlock and drive the trigger edges.
 *
 * Reads the EE/OPT safety-FPGA STATUS registers (I2C 0x41, mux1 ch6=EE /
 * ch7=OPT, reg 0x24; low 3 bits = peak/pulse/rate fault latch). The safety
 * FPGAs already hardware-inhibit the TA on a trip; this only tears down the
 * trigger *source* so clearing a fault can't refire the laser (test-app #56).
 * Firmware is NOT the safety guarantee. Mirrors the TEC-trip pattern, but with
 * an explicit (not auto-timeout) recovery: the latch clears only once the FPGA
 * STATUS reads clean again (i.e. the operator cleared the fault), and
 * Trigger_Start stays blocked until then. While latched, the status LED blinks
 * blue (fault indication) whatever the trigger state, including a fault
 * already present at connect. Extracted from telemetry_poll to keep
 * that function under the IEC 62304 complexity threshold.
 */
static void laser_safety_poll(void)
{
	uint8_t ee_status = 0, opt_status = 0;
	bool ee_read  = (TCA9548A_Read_Data(1, 6, 0x41, 0x24, 1, &ee_status)  == TCA9548A_OK);
	bool opt_read = (TCA9548A_Read_Data(1, 7, 0x41, 0x24, 1, &opt_status) == TCA9548A_OK);
	if (ee_read && opt_read) {
		uint8_t fault = (uint8_t)((ee_status | opt_status) & 0x07u);
		if (fault) {
			Trigger_LaserSafety_Trip();   /* stop trigger + latch */
			if (!_laser_safety_trip_set) {
				char msg[128];
				int n = snprintf(msg, sizeof(msg),
					"{\"type\": \"system\", \"state\": \"error\", \"msg\": \"Laser safety trip\", \"ee\": %u, \"opt\": %u}",
					(unsigned)ee_status, (unsigned)opt_status);
				if (n > 0 && (size_t)n < sizeof(msg) && !mq_push(msg, (size_t)n)) {
					printf("Failed to push laser-safety trip message to queue\r\n");
				}
				_laser_safety_trip_set = true;
				printf("++++> LASER SAFETY TRIP  EE=0x%02X OPT=0x%02X\r\n", ee_status, opt_status);
			}
		} else if (_laser_safety_trip_set) {
			Trigger_LaserSafety_Clear();  /* fault cleared -> allow re-arm */
			_laser_safety_trip_set = false;
		}
	}
	/* Driven every poll, not only after a good read, so the blink keeps time
	 * through a transient I2C miss. */
	LED_Fault_Indicate(_laser_safety_trip_set, HAL_GetTick());
}

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

	/* TEC over-voltage trip + laser-safety (EE/OPT) interlock. Both are
	 * extracted to helpers to keep telemetry_poll under the IEC 62304
	 * cyclomatic-complexity threshold; both run inside the DWT-timed window. */
	tec_trip_evaluate(&sample);
	laser_safety_poll();

	/* --- end timed acquisition --- */
	uint32_t dwt_cycles = DWT->CYCCNT - dwt_start;
	sample.acq_time_us  = dwt_cycles / (SystemCoreClock / 1000000U);

	/* Evict oldest sample if ring buffer is full */
	if (lwrb_get_free(&s_telemetry_rb) < sizeof(TelemetrySample)) {
		lwrb_skip(&s_telemetry_rb, sizeof(TelemetrySample));
	}
	lwrb_write(&s_telemetry_rb, &sample, sizeof(TelemetrySample));
	if(_tec_sample_lock){
		// if the sample is being read by the command module, skip copying to the shared last_tec_stats
		return;
	}else{
		last_tec_stats.timestamp_ms = sample.timestamp_ms;
		last_tec_stats.vout         = adc_to_voltage(sample.tec_adc[0]);
		last_tec_stats.temp_set     = adc_to_voltage(sample.tec_adc[1]);
		last_tec_stats.tec_curr     = adc_to_voltage(sample.tec_adc[2]);
		last_tec_stats.tec_volt     = adc_to_voltage(sample.tec_adc[3]);
		last_tec_stats.tec_status   = sample.tec_status;	
	}

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
void comms_handle_RxCpltCallback(const UART_HandleTypeDef *huart, uint16_t pos) {

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

void comms_handle_TxCallback(const UART_HandleTypeDef *huart) {

	if (huart->Instance == USART1) {
		tx_flag = 1;
	}
}

void comms_handle_ErrorCallback(const UART_HandleTypeDef *huart) {

    if (huart->Instance == USART1) {
        // Handle errors here. Maybe reset DMA reception, etc.
    }
}
