/*
 * trigger.h
 *
 *  Created on: Nov 25, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_TRIGGER_H_
#define INC_TRIGGER_H_

#include "jsmn.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define NUM_DARK_FRAMES_AT_START 10

/* Longest trigger-configuration JSON accepted by OW_CTRL_SET_TRIG, in bytes
 * without the terminating NUL. */
#define TRIGGER_JSON_MAX 511U

/* Operating limits enforced by Trigger_SetConfig (ARCH-74, ARCH-69).
 * Frequency: 40 Hz is the system maximum (the sensor's 15 ms acceptance
 * interval). Laser gate width: the same 20..2200 us range the application
 * allows (bloodflow-app LASER_PULSE_WIDTH_MIN_US / _MAX_US); below 20 us the
 * TA driver compare underflows. The laser delay, gate and dark-frame skip
 * delay must also end inside one frame period. */
#define TRIGGER_FREQ_MIN_HZ          1.0f
#define TRIGGER_FREQ_MAX_HZ          40.0f
#define TRIGGER_LASER_WIDTH_MIN_US   20U
#define TRIGGER_LASER_WIDTH_MAX_US   2200U

/* Trigger_GetInterlocks() bits, also the status byte of a refused
 * OW_CTRL_START_TRIG reply. */
#define TRIG_INTERLOCK_HOST_LOSS     0x01U  /* USB disconnect or VCP closed */
#define TRIG_INTERLOCK_LASER_SAFETY  0x02U  /* EE/OPT safety FPGA fault latched */
#define TRIG_INTERLOCK_TEC           0x04U  /* TEC thermal trip */
#define TRIG_START_TIMER_FAILED      0x80U  /* no interlock; FSYNC timer did not start */

typedef struct {
    float    frequencyHz;        // Trigger frequency in Hz 1.00 - 40.00 (TRIGGER_FREQ_MIN_HZ/_MAX_HZ)
    uint32_t triggerPulseWidthUsec;     // Pulse width in microseconds max determined by freq
    uint32_t laserPulseDelayUsec;     // Pulse width in microseconds max based on selected freq
    uint32_t laserPulseWidthUsec;     // Pulse width in microseconds max based on selected freq
    uint32_t LaserPulseSkipInterval;     // skip laser every nTH pulse
    bool EnableSyncOut;
    bool EnableTaTrigger;
    uint32_t TriggerStatus;
    uint32_t LaserPulseSkipDelayUsec;
} Trigger_Config_t;

void trigger_init(void);
HAL_StatusTypeDef Trigger_SetConfig(const Trigger_Config_t *config);
HAL_StatusTypeDef Trigger_Start() ;
HAL_StatusTypeDef Trigger_Stop();
HAL_StatusTypeDef Trigger_SetConfigFromJSON(const char *jsonString, size_t str_len);
HAL_StatusTypeDef Trigger_GetConfigToJSON(char *jsonString, size_t max_length);
uint32_t get_lsync_pulse_count(void);
uint32_t get_fsync_pulse_count(void);
void Trigger_Safety_Disconnect(void);
void Trigger_Safety_Clear(void);
void Trigger_LaserSafety_Trip(void);
void Trigger_LaserSafety_Clear(void);
uint8_t Trigger_GetInterlocks(void);
void Trigger_PrintConfig(const Trigger_Config_t *config);

void FSYNC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void LSYNC_DelayElapsedCallback(TIM_HandleTypeDef *htim);

void FSYNC_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void LSYNC_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
bool get_current_slot_is_dark(void);
bool consume_pdc_sample_pending(bool *out_dark_slot, uint32_t *out_frame_idx);
uint16_t consume_pdc_pending_overwrites(void);

extern Trigger_Config_t trigger_config;

#endif /* INC_TRIGGER_H_ */

