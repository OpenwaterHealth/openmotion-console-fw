/*
 * led_driver.c
 *
 *  Created on: Nov 22, 2024
 *      Author: GeorgeVigelette
 */
#include "led_driver.h"

#include <stdio.h>


static uint8_t rgb_state = 0;
/* Set while a laser-safety fault is latched. Read from the USB ISR context
 * (Trigger_Stop on disconnect / port close), hence volatile. */
static volatile bool s_fault_active = false;
static uint32_t s_fault_since_ms = 0;

// Initialize the LEDs
void LED_Init(void)
{
	printf("Initializing Indicators\r\n");
    // Turn all LEDs off initially
    HAL_GPIO_WritePin(IND1_GPIO_Port, IND1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IND2_GPIO_Port, IND2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IND3_GPIO_Port, IND3_Pin, GPIO_PIN_SET);
}

void LED_RGB_SET(uint8_t rgbState)
{
	rgb_state = rgbState;
	HAL_GPIO_WritePin(IND1_GPIO_Port, IND1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IND2_GPIO_Port, IND2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IND3_GPIO_Port, IND3_Pin, GPIO_PIN_SET);
	switch(rgb_state)
	{
		case 1:
			HAL_GPIO_WritePin(IND1_GPIO_Port, IND1_Pin, GPIO_PIN_RESET); // red
			break;
		case 2:
			HAL_GPIO_WritePin(IND2_GPIO_Port, IND2_Pin, GPIO_PIN_RESET); // blue
			break;
		case 3:
			HAL_GPIO_WritePin(IND3_GPIO_Port, IND3_Pin, GPIO_PIN_RESET); // green
			break;
		case 0:
		default:
			break;
	}
}

uint8_t LED_RGB_GET(void)
{
	return rgb_state;
}

void LED_Indicator_Set(uint8_t rgbState)
{
	if (s_fault_active) {
		return; /* latched laser-safety fault owns the indicator */
	}
	LED_RGB_SET(rgbState);
}

void LED_Fault_Indicate(bool fault_latched, uint32_t now_ms)
{
	if (fault_latched) {
		if (!s_fault_active) {
			s_fault_since_ms = now_ms; /* first blue edge now */
			s_fault_active = true;
		}
		/* Unsigned subtraction stays correct across HAL tick wraparound. */
		uint32_t phase = ((now_ms - s_fault_since_ms) / LED_FAULT_BLINK_HALF_PERIOD_MS) & 1U;
		uint8_t want = (phase == 0U) ? LED_BLUE : LED_NONE;
		if (want != rgb_state) {
			LED_RGB_SET(want);
		}
	} else if (s_fault_active) {
		/* Latch cleared. The trip stopped the trigger and Trigger_Start is
		 * refused while latched, so the console is idle. */
		s_fault_active = false;
		LED_RGB_SET(LED_GREEN);
	}
}

// Set the state of an LED (ON/OFF)
void LED_SetState(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, LED_State state)
{
    if (state == LED_ON) {
        HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET); // Turn LED on
    } else {
        HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET); // Turn LED off
    }
}

// Toggle the state of an LED
void LED_Toggle(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin)
{
    HAL_GPIO_TogglePin(GPIO_Port, GPIO_Pin); // Toggle LED
}


