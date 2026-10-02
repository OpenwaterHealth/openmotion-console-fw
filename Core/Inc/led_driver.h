/*
 * led_driver.h
 *
 *  Created on: Nov 22, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_LED_DRIVER_H_
#define INC_LED_DRIVER_H_

#include "main.h"
#include <stdbool.h>

/* Laser-safety fault indication: blink blue, 500 ms on / 500 ms off (same
 * period as Error_Handler and the host app's critical-error blink). */
#define LED_FAULT_BLINK_HALF_PERIOD_MS 500U

// LED States
typedef enum {
    LED_OFF = 0,
    LED_ON = 1
} LED_State;

typedef enum {
    LED_NONE = 0,
    LED_RED = 1,
    LED_GREEN = 2,
    LED_BLUE = 3
} LED_COLORS;

// Function prototypes
void LED_Init(void);
void LED_SetState(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, LED_State state);
void LED_Toggle(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin);
void LED_RGB_SET(uint8_t state);
uint8_t LED_RGB_GET(void);

/* Status-indicator writes (trigger start/stop, host OW_CTRL_SET_IND) go through
 * LED_Indicator_Set rather than LED_RGB_SET: while a laser-safety fault is
 * latched the fault blink owns the LED and these writes are not applied. */
void LED_Indicator_Set(uint8_t state);
/* Call every telemetry poll with the laser-safety latch state. While latched,
 * blinks the LED blue; when the latch clears, returns it to idle green. */
void LED_Fault_Indicate(bool fault_latched, uint32_t now_ms);

#endif /* INC_LED_DRIVER_H_ */
