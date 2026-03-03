/*
 * utils.h
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"
#include <stdint.h>

#define MIN_TEMP   -40
#define MAX_TEMP   125
#define TABLE_SIZE (MAX_TEMP - MIN_TEMP + 1)


/* ===== Constants ===== */
#define SCALE_V   0.0909
#define SCALE_I   0.25
#define V_REF     2.459      // Empirical reference voltage
#define R_1       18000.0
#define R_2       8160.0
#define R_3       49900.0
#define R230      300000.0
#define R234      300000.0
#define R_S       0.020

uint16_t util_crc16(const uint8_t* buf, uint32_t size);
uint16_t util_hw_crc16(uint8_t* buf, uint32_t size);
void printBuffer(const uint8_t* buffer, uint32_t size);
void DWT_Init(void);
void delay_us(uint32_t us);
void GPIO_SetHiZ(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_SetOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
double temperature_to_resistance(double temp_c);
uint8_t BoardV_Read(void);
double solve_R_TH(double v);
double solve_v(double R_TH);
#endif /* INC_UTILS_H_ */
