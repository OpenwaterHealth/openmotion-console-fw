/*
 * trigger.c
 *
 *  Created on: Nov 25, 2024
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "trigger.h"
#include "usb_events.h"
#include "odometer.h"
#include "led_driver.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>


// setup default
Trigger_Config_t trigger_config = { 40.0f, 1000, 250, 1000 };

volatile uint8_t _usb_trigger_interlock = 0;
volatile uint8_t _safety_trigger_interlock = 0;
/* Laser-safety (EE/OPT) trip latch. Set when telemetry_poll reads a latched
 * peak/pulse/rate fault from a safety FPGA; blocks Trigger_Start until the fault
 * clears. NOTE: this is not the laser-safety guarantee (the safety FPGAs
 * hardware-inhibit the TA on their own) — it only guarantees the trigger source
 * is torn down so clearing a fault can't refire the laser. Separate from
 * _safety_trigger_interlock (the TEC thermal path, which auto-recovers). */
volatile uint8_t _laser_safety_interlock = 0;

volatile uint32_t fsync_counter = 0;
volatile uint32_t lsync_counter = 0;

/* s_current_slot_is_dark is the decision the most recent FSYNC ISR made — it
 * controls the LASER_TIMER preload that becomes shadow at the next UPDATE
 * event, i.e. it's the slot for the cycle that runs *after* the cycle that's
 * about to fire CC1. The cycle that fires CC1 right now uses the shadow that
 * was loaded by the *previous* FSYNC ISR's preload write. So LSYNC must read
 * the previous decision, not the current one. We initialize s_prev to true to
 * match Trigger_Start's direct shadow write of long_lsync_arr. */
static volatile bool s_current_slot_is_dark = true;
static volatile bool s_prev_slot_is_dark    = true;

/* SPSC queue of pending PDC samples between the LSYNC ISR (producer) and the
 * main-loop pdc_poll_tick (consumer). Sized for ~200 ms of slack at 40 Hz so
 * brief main-loop stalls (USB bursts, comms handling) don't lose frames. */
#define PDC_PENDING_CAPACITY 32
typedef struct {
    uint32_t frame_idx;
    bool     dark_slot;
} pdc_pending_t;
static volatile pdc_pending_t s_pending_buf[PDC_PENDING_CAPACITY];
static volatile uint16_t s_pending_head     = 0;  /* write idx (ISR) */
static volatile uint16_t s_pending_tail     = 0;  /* read idx (main) */
static volatile uint16_t s_pending_count    = 0;
static volatile uint16_t s_pending_overwrites = 0;  /* drops-on-full since last consume */

uint32_t short_lsync_arr = 0;
uint32_t long_lsync_arr = 0;
uint32_t short_lsync_ccr1 = 0;
uint32_t long_lsync_ccr1 = 0;

bool fsync_disable_flag = false;

static int jsoneq(const char *json, const jsmntok_t *tok, const char *s) {
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
    return 0;
  }
  return -1;
}

static int jsonToTriggerConfigData(const char *jsonString, Trigger_Config_t* newConfig)
{
    int i, r;
    jsmn_parser parser;
    jsmntok_t t[32]; // Increased size to handle more tokens

    jsmn_init(&parser, NULL);
    r = jsmn_parse(&parser, jsonString, strlen(jsonString), t, sizeof(t) / sizeof(t[0]), NULL);
    if (r < 0) {
        printf("jsonToTriggerConfigData Failed to parse JSON: %d\n", r);
        return 1;
    }

    if (r < 1 || t[0].type != JSMN_OBJECT) {
        printf("jsonToTriggerConfigData Object expected\n");
        return 1;
    }

    for (i = 1; i < r; i++) {
        /* Every recognised key reads its value from t[i + 1]. A JSON object whose
         * last token is a recognised key would otherwise read one token past the
         * parsed set (uninitialised stack, or past the end of t[] when all 32 are
         * used) and use it as an offset into the input (CVA O8 / R7). The input
         * arrives over USB from the host. */
        if (i + 1 >= r) {
            break;
        }
        if (jsoneq(jsonString, &t[i], "TriggerFrequencyHz") == 0) {
            newConfig->frequencyHz = strtof(jsonString + t[i + 1].start, NULL);
            i++;
        } else if (jsoneq(jsonString, &t[i], "TriggerPulseWidthUsec") == 0) {
            newConfig->triggerPulseWidthUsec = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        } else if (jsoneq(jsonString, &t[i], "LaserPulseDelayUsec") == 0) {
            newConfig->laserPulseDelayUsec = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        } else if (jsoneq(jsonString, &t[i], "LaserPulseWidthUsec") == 0) {
            newConfig->laserPulseWidthUsec = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        } else if (jsoneq(jsonString, &t[i], "LaserPulseSkipInterval") == 0) {
            newConfig->LaserPulseSkipInterval = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        } else if (jsoneq(jsonString, &t[i], "TriggerStatus") == 0) {
            newConfig->TriggerStatus = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        } else if (jsoneq(jsonString, &t[i], "EnableSyncOut") == 0) {
            newConfig->EnableSyncOut = (strncmp(jsonString + t[i + 1].start, "true", 4) == 0);
            i++;
        } else if (jsoneq(jsonString, &t[i], "EnableTaTrigger") == 0) {
            newConfig->EnableTaTrigger = (strncmp(jsonString + t[i + 1].start, "true", 4) == 0);
            i++;
        } else if (jsoneq(jsonString, &t[i], "LaserPulseSkipDelayUsec") == 0) {
            newConfig->LaserPulseSkipDelayUsec = strtoul(jsonString + t[i + 1].start, NULL, 10);
            i++;
        }
    }
    return 0; // Success
}

static void trigger_GetConfigJSON(char *jsonString, size_t max_length)
{
    memset(jsonString, 0, max_length);
    snprintf(jsonString, max_length,
             "{"
             "\"TriggerFrequencyHz\": %.2f,"
             "\"TriggerPulseWidthUsec\": %lu,"
             "\"LaserPulseDelayUsec\": %lu,"
             "\"LaserPulseWidthUsec\": %lu,"
             "\"LaserPulseSkipInterval\": %lu,"
             "\"LaserPulseSkipDelayUsec\": %lu,"
             "\"EnableSyncOut\": %s,"
             "\"EnableTaTrigger\": %s,"
             "\"TriggerStatus\": %lu"
             "}",
             (double)trigger_config.frequencyHz,
             trigger_config.triggerPulseWidthUsec,
             trigger_config.laserPulseDelayUsec,
             trigger_config.laserPulseWidthUsec,
             trigger_config.LaserPulseSkipInterval,
             trigger_config.LaserPulseSkipDelayUsec,
             trigger_config.EnableSyncOut ? "true" : "false",
             trigger_config.EnableTaTrigger ? "true" : "false",
			 trigger_config.TriggerStatus);
}

static void updateTimerDataFromPeripheral()
{
	 uint32_t preScaler = FSYNC_TIMER.Instance->PSC;
	 uint32_t timerClockFrequency = HAL_RCC_GetPCLK1Freq() / (preScaler + 1);
	 uint32_t TIM_ARR = FSYNC_TIMER.Instance->ARR;
	 uint32_t TIM_CCRx = HAL_TIM_ReadCapturedValue(&FSYNC_TIMER, FSYNC_TIMER_CHAN);
	 trigger_config.frequencyHz = (float)timerClockFrequency / (float)(TIM_ARR + 1);
	 trigger_config.triggerPulseWidthUsec = ((TIM_CCRx * 100000) / timerClockFrequency) * 10;

	 // LASER_TIMER ARR/CCR1 alternate between short_lsync and long_lsync slots,
	 // both of which are composites of laserPulseDelayUsec + LaserPulseSkipDelayUsec.
	 // They can't be unambiguously reversed into the source fields, so trust the
	 // in-RAM values that Trigger_SetConfig already stored.

	 trigger_config.TriggerStatus = TIM_CHANNEL_STATE_GET(&FSYNC_TIMER, FSYNC_TIMER_CHAN);
}

static void trigger_usb_disconnect_cb(usb_event_t event)
{
    (void)event;
    _usb_trigger_interlock = 1;
    Trigger_Stop();
}

static void trigger_usb_connect_cb(usb_event_t event)
{
    (void)event;
    _usb_trigger_interlock = 0;
}

void trigger_init(void)
{
    usb_register_callback(USB_EVENT_DISCONNECT, trigger_usb_disconnect_cb);
    usb_register_callback(USB_EVENT_CONNECT, trigger_usb_connect_cb);
    /* A host closing the VCP (DTR de-asserted) means the host application has
     * detached even if the USB cable is still plugged — there is no DISCONNECT
     * in that case. Treat PORT_CLOSE exactly like a disconnect (interlock +
     * stop the laser) and PORT_OPEN like a connect (clear the interlock). This
     * replaces the old time-based host-loss watchdog with the deterministic
     * usb_events host-presence signal. */
    usb_register_callback(USB_EVENT_PORT_CLOSE, trigger_usb_disconnect_cb);
    usb_register_callback(USB_EVENT_PORT_OPEN, trigger_usb_connect_cb);
}

void Trigger_Safety_Disconnect(void)
{
    _safety_trigger_interlock = 1;
    Trigger_Stop();
}

void Trigger_Safety_Clear(void)
{
    _safety_trigger_interlock = 0;
}

void Trigger_LaserSafety_Trip(void)
{
    _laser_safety_interlock = 1;
    Trigger_Stop();
}

void Trigger_LaserSafety_Clear(void)
{
    _laser_safety_interlock = 0;
}

HAL_StatusTypeDef Trigger_SetConfig(const Trigger_Config_t *config) {
    if (config == NULL) {
        return HAL_ERROR; // Null pointer guard
    }

    // Add range checks for the configuration parameters
    if (config->frequencyHz < 1.0f || config->triggerPulseWidthUsec == 0 || config->frequencyHz > 100.0f) {
        return HAL_ERROR; // Invalid configuration values
    }

	if(trigger_config.TriggerStatus == HAL_TIM_CHANNEL_STATE_BUSY)
	{
		// stop timer pwm
		HAL_TIM_OC_Stop_IT(&FSYNC_TIMER, FSYNC_TIMER_CHAN);

		trigger_config.TriggerStatus = HAL_TIM_CHANNEL_STATE_READY;

		HAL_GPIO_WritePin(enSyncOUT_GPIO_Port, enSyncOUT_Pin, GPIO_PIN_SET); // disable fsync output
		HAL_GPIO_WritePin(nTRIG_GPIO_Port, nTRIG_Pin, GPIO_PIN_SET); // disable TA Trigger to fpga
	}

    // Reset the counters on SetConfig
	lsync_counter = 1;
	fsync_counter = 1;
    fsync_disable_flag = false;
    
    // Use fixed 1 MHz timer tick (1 µs per tick)
    uint32_t fsync_prescaler = 119; // (e.g., 120MHz / (119+1) = 1 MHz)
    FSYNC_TIMER.Instance->PSC = fsync_prescaler;

    // Calculate ARR and CCR1
    uint32_t arr_ticks = (uint32_t)(1000000.0f / config->frequencyHz + 0.5f); // period in µs, rounded to nearest tick
    if (config->triggerPulseWidthUsec >= arr_ticks) {
        return HAL_ERROR; // Pulse width too long
    }

    FSYNC_TIMER.Instance->ARR = arr_ticks - 1;
    FSYNC_TIMER.Instance->CCR1 = config->triggerPulseWidthUsec;

    // Configure LASER timer (same tick frequency)
    LASER_TIMER.Instance->PSC = fsync_prescaler;

    uint32_t laser_delay_ticks = config->laserPulseDelayUsec;
    uint32_t laser_width_ticks = config->laserPulseWidthUsec;

    uint32_t laser_delay = config->LaserPulseSkipDelayUsec;
    
    short_lsync_arr = laser_delay_ticks + laser_width_ticks - 1;
    long_lsync_arr = short_lsync_arr + laser_delay;

    short_lsync_ccr1 = laser_delay_ticks;
    long_lsync_ccr1 = laser_delay_ticks + laser_delay;

    LASER_TIMER.Instance->ARR = long_lsync_arr;     // First frame should be a dark frame with the delay
    LASER_TIMER.Instance->CCR1 = long_lsync_ccr1;

    // Force register update
    FSYNC_TIMER.Instance->EGR |= TIM_EGR_UG;
    LASER_TIMER.Instance->EGR |= TIM_EGR_UG;

    LASER_TIMER.Instance->CR1 |= TIM_CR1_ARPE;            // ARR preload enable
    LASER_TIMER.Instance->CCMR1 |= TIM_CCMR1_OC1PE;       // CCR1 preload enable

    // Enable the interrupt that happens when FSYNC TRGO fires
    __HAL_TIM_CLEAR_FLAG(&FSYNC_TIMER, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&FSYNC_TIMER, TIM_IT_UPDATE);

    // Update the global trigger configuration
    trigger_config = *config;
    return HAL_OK;
}


HAL_StatusTypeDef Trigger_Start() {
    if(_usb_trigger_interlock || _laser_safety_interlock){
        return HAL_ERROR;
    }

    // Clear any pending stop request to avoid a one-pulse stop on restart
    fsync_disable_flag = false;

	HAL_GPIO_WritePin(enSyncOUT_GPIO_Port, enSyncOUT_Pin, trigger_config.EnableSyncOut? GPIO_PIN_RESET:GPIO_PIN_SET); // fsync out
	HAL_GPIO_WritePin(nTRIG_GPIO_Port, nTRIG_Pin, trigger_config.EnableTaTrigger? GPIO_PIN_RESET:GPIO_PIN_SET); // TA Trigger enable

	lsync_counter = 1;
	fsync_counter = 1;
	/* Match initial Trigger_SetConfig direct shadow write of long_lsync_arr. */
	s_current_slot_is_dark = true;
	s_prev_slot_is_dark = true;
	s_pending_head = 0;
	s_pending_tail = 0;
	s_pending_count = 0;
	s_pending_overwrites = 0;

	/* Update laser odometer at scan start */
	Odometer_Scan_Start();

	__HAL_TIM_ENABLE_IT(&LASER_TIMER, TIM_IT_CC1);
	__HAL_TIM_CLEAR_FLAG(&LASER_TIMER, TIM_FLAG_UPDATE);
	__HAL_TIM_ENABLE_IT(&LASER_TIMER, TIM_IT_UPDATE);
	if(HAL_TIM_OC_Start_IT(&FSYNC_TIMER, FSYNC_TIMER_CHAN) != HAL_OK) {
        return HAL_ERROR; // Handle error
	}

	LED_Indicator_Set(LED_BLUE); // trigger/laser active
    return HAL_OK;
}

HAL_StatusTypeDef Trigger_Stop() {

    // If already stopped, clear any pending stop request so next start isn't cut short
    if (TIM_CHANNEL_STATE_GET(&FSYNC_TIMER, FSYNC_TIMER_CHAN) != HAL_TIM_CHANNEL_STATE_BUSY) {
        fsync_disable_flag = false;
        HAL_TIM_OC_Stop_IT(&FSYNC_TIMER, FSYNC_TIMER_CHAN);
    } else {
        fsync_disable_flag = true;
    }

    __HAL_TIM_DISABLE(&LASER_TIMER);
    __HAL_TIM_DISABLE_IT(&LASER_TIMER, TIM_IT_UPDATE);

	HAL_GPIO_WritePin(enSyncOUT_GPIO_Port, enSyncOUT_Pin, GPIO_PIN_SET); // disable fsync output
	HAL_GPIO_WritePin(nTRIG_GPIO_Port, nTRIG_Pin, GPIO_PIN_SET); // disable TA Trigger to fpga

	/* Update laser odometer at scan finish */
	Odometer_Scan_Finish();

	/* Return the indicator to idle on EVERY stop path — STOP_TRIG command,
	 * USB disconnect, host VCP close (PORT_CLOSE), and the TEC safety trip —
	 * so the console shows idle (green) whenever the laser is actually
	 * stopped. Pure GPIO, safe from the USB ISR context. Not applied while a
	 * laser-safety fault is latched: the trip stops the trigger on every
	 * telemetry poll, and an idle green here would mask the fault blink. */
	LED_Indicator_Set(LED_GREEN); // idle
    return HAL_OK;
}


HAL_StatusTypeDef Trigger_SetConfigFromJSON(const char *jsonString, size_t str_len)
{
	uint8_t tempArr[255] = {0};
	bool ret = HAL_OK;

	// Seed from current config so any field absent from JSON keeps its current value
	Trigger_Config_t new_config = trigger_config;
    // Copy the JSON string to tempArr
    memcpy((char *)tempArr, jsonString, str_len);
    // printf("Trigger_SetConfigFromJSON: %s\r\n", (char *)tempArr);

	if (jsonToTriggerConfigData((const char *)tempArr, &new_config) == 0)
	{
        Trigger_PrintConfig((const Trigger_Config_t*)&new_config);
		Trigger_SetConfig(&new_config);
		ret = HAL_OK;
	}
	else{
		ret = HAL_ERROR;
	}

	return ret;

}

HAL_StatusTypeDef Trigger_GetConfigToJSON(char *jsonString, size_t max_length)
{
	updateTimerDataFromPeripheral();
	trigger_GetConfigJSON(jsonString, 0xFF);
    return HAL_OK;
}

uint32_t get_lsync_pulse_count(void)
{
	return lsync_counter;
}

uint32_t get_fsync_pulse_count(void)
{
	return fsync_counter;
}

void Trigger_PrintConfig(const Trigger_Config_t *config)
{
    if (config == NULL) { return; }
    printf("Trigger_Config_t:\r\n");
    printf("  frequencyHz:           %.2f\r\n", (double)config->frequencyHz);
    printf("  triggerPulseWidthUsec: %lu\r\n", config->triggerPulseWidthUsec);
    printf("  laserPulseDelayUsec:   %lu\r\n", config->laserPulseDelayUsec);
    printf("  laserPulseWidthUsec:   %lu\r\n", config->laserPulseWidthUsec);
    printf("  LaserPulseSkipInterval:%lu\r\n", config->LaserPulseSkipInterval);
    printf("  LaserPulseSkipDelayUs: %lu\r\n", config->LaserPulseSkipDelayUsec);
    printf("  EnableSyncOut:         %s\r\n", config->EnableSyncOut  ? "true" : "false");
    printf("  EnableTaTrigger:       %s\r\n", config->EnableTaTrigger? "true" : "false");
    printf("  TriggerStatus:         %lu\r\n", config->TriggerStatus);
}

void FSYNC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Disable fsync output when the flag goes high on the last pulse
    if(fsync_disable_flag){
        HAL_TIM_OC_Stop_IT(&FSYNC_TIMER, FSYNC_TIMER_CHAN);
        fsync_disable_flag = false;
    }
}

void FSYNC_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    fsync_counter++;
    if (trigger_config.LaserPulseSkipInterval > 0) {
        bool dark = ((fsync_counter % trigger_config.LaserPulseSkipInterval) == 0)
                    || (fsync_counter < NUM_DARK_FRAMES_AT_START);
        if (dark) {
            __HAL_TIM_SET_AUTORELOAD(&LASER_TIMER, long_lsync_arr);
            __HAL_TIM_SET_COMPARE  (&LASER_TIMER, TIM_CHANNEL_1, long_lsync_ccr1);
        } else {
            __HAL_TIM_SET_AUTORELOAD(&LASER_TIMER, short_lsync_arr);
            __HAL_TIM_SET_COMPARE  (&LASER_TIMER, TIM_CHANNEL_1, short_lsync_ccr1);
        }
        /* Save the slot decision that controls the cycle about to fire — i.e.
         * the previous ISR's decision — before overwriting with this ISR's. */
        s_prev_slot_is_dark = s_current_slot_is_dark;
        s_current_slot_is_dark = dark;
    }
}
void LSYNC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* Fires on laser-pulse rising edge (CC1 compare match).  Increment the
     * frame counter and enqueue (frame_idx, dark_slot) for pdc_poll_tick to
     * consume from the main loop.  Drop-oldest on overflow so the freshest
     * samples win; the overwrite count propagates to the ring buffer's drop
     * counter via consume_pdc_pending_overwrites().
     *
     * We enqueue at the rising edge rather than the falling edge because
     * STM32 TIM_UPDATE events get coalesced when interrupts are delayed.
     *
     * dark_slot is computed locally from lsync_counter rather than read from
     * s_current_slot_is_dark / s_prev_slot_is_dark — neither was reliable
     * empirically, because FSYNC ISR for frame N can run either *before* or
     * *after* LSYNC ISR for cycle N depending on what other ISRs the CPU is
     * busy with. By computing the slot from lsync_counter directly we get a
     * deterministic answer that exactly matches what the LASER_TIMER shadow
     * was during this cycle (which is set by ISR N-1's preload decision = the
     * decision based on fsync_counter post-incremented to N).
     *
     * Cycle K's actual slot:
     *   cycle 1: initial long (set by Trigger_SetConfig) -> dark
     *   cycle K>=2: ISR(K-1)'s preload decision, which uses fsync_counter==K:
     *     dark iff (K < NUM_DARK_FRAMES_AT_START) || (K % skip == 0)
     * Both branches reduce to the same formula on K = lsync_counter - 1.
     */
    lsync_counter++;
    uint32_t cycle = lsync_counter - 1;
    bool dark = (cycle < NUM_DARK_FRAMES_AT_START)
             || (trigger_config.LaserPulseSkipInterval > 0
                 && cycle > 0
                 && (cycle % trigger_config.LaserPulseSkipInterval) == 0);

    if (s_pending_count == PDC_PENDING_CAPACITY) {
        s_pending_tail = (uint16_t)((s_pending_tail + 1) % PDC_PENDING_CAPACITY);
        s_pending_count--;
        s_pending_overwrites++;
    }
    s_pending_buf[s_pending_head].frame_idx = lsync_counter;
    s_pending_buf[s_pending_head].dark_slot = dark;
    s_pending_head = (uint16_t)((s_pending_head + 1) % PDC_PENDING_CAPACITY);
    s_pending_count++;
}

void LSYNC_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* No-op. PDC sample enqueue moved to LSYNC_DelayElapsedCallback (rising
     * edge) to avoid TIM_UPDATE coalescing. Kept declared so the dispatch in
     * main.c still compiles; the dispatch could be removed once we're sure
     * the rising-edge approach is stable. */
    (void)htim;
}

bool get_current_slot_is_dark(void) { return s_current_slot_is_dark; }

bool consume_pdc_sample_pending(bool *out_dark_slot, uint32_t *out_frame_idx)
{
    __disable_irq();
    bool pending = (s_pending_count > 0);
    if (pending) {
        *out_dark_slot = s_pending_buf[s_pending_tail].dark_slot;
        *out_frame_idx = s_pending_buf[s_pending_tail].frame_idx;
        s_pending_tail = (uint16_t)((s_pending_tail + 1) % PDC_PENDING_CAPACITY);
        s_pending_count--;
    }
    __enable_irq();
    return pending;
}

uint16_t consume_pdc_pending_overwrites(void)
{
    __disable_irq();
    uint16_t n = s_pending_overwrites;
    s_pending_overwrites = 0;
    __enable_irq();
    return n;
}
