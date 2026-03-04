/*
 * utils.c
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#include "utils.h"
#include <stdio.h>

/* Resistance table from -40°C to 125°C */
static const long resistance_table[TABLE_SIZE] = {
	336098, 314553, 294524, 275897, 258563, 242427, 227398, 213394, 200339, 188163,
	177000, 166198, 156294, 147042, 138393, 130306, 122741, 115661, 109032, 102824,
	97006, 91553, 86439, 81641, 77138, 72911, 68940, 65209, 61703, 58405,
	55304, 52385, 49638, 47050, 44613, 42317, 40151, 38110, 36184, 34366,
	32651, 31031, 29500, 28054, 26687, 25395, 24172, 23016, 21921, 20885,
	19903, 18973, 18092, 17257, 16465, 15714, 15001, 14324, 13682, 13073,
	12493, 11943, 11420, 10923, 10450, 10000, 9572, 9165, 8777, 8408,
	8056, 7721, 7402, 7097, 6807, 6530, 6266, 6014, 5774, 5544,
	5325, 5116, 4916, 4724, 4542, 4367, 4200, 4040, 3887, 3741,
	3601, 3467, 3339, 3216, 3098, 2985, 2877, 2773, 2674, 2579,
	2487, 2399, 2315, 2234, 2157, 2082, 2011, 1942, 1876, 1813,
	1752, 1693, 1637, 1582, 1530, 1480, 1432, 1385, 1341, 1298,
	1256, 1216, 1178, 1141, 1105, 1070, 1037, 1005, 974, 945,
	916, 888, 862, 836, 811, 787, 764, 741, 720, 699,
	678, 659, 640, 622, 604, 587, 571, 555, 539, 524,
	510, 496, 482, 469, 457, 444, 432, 421, 410, 399,
	388, 378, 368, 359, 350, 341};

// CRC16-ccitt lookup table
const uint16_t crc16_tab[256] = {
	0x0000,
	0x1021,
	0x2042,
	0x3063,
	0x4084,
	0x50a5,
	0x60c6,
	0x70e7,
	0x8108,
	0x9129,
	0xa14a,
	0xb16b,
	0xc18c,
	0xd1ad,
	0xe1ce,
	0xf1ef,
	0x1231,
	0x0210,
	0x3273,
	0x2252,
	0x52b5,
	0x4294,
	0x72f7,
	0x62d6,
	0x9339,
	0x8318,
	0xb37b,
	0xa35a,
	0xd3bd,
	0xc39c,
	0xf3ff,
	0xe3de,
	0x2462,
	0x3443,
	0x0420,
	0x1401,
	0x64e6,
	0x74c7,
	0x44a4,
	0x5485,
	0xa56a,
	0xb54b,
	0x8528,
	0x9509,
	0xe5ee,
	0xf5cf,
	0xc5ac,
	0xd58d,
	0x3653,
	0x2672,
	0x1611,
	0x0630,
	0x76d7,
	0x66f6,
	0x5695,
	0x46b4,
	0xb75b,
	0xa77a,
	0x9719,
	0x8738,
	0xf7df,
	0xe7fe,
	0xd79d,
	0xc7bc,
	0x48c4,
	0x58e5,
	0x6886,
	0x78a7,
	0x0840,
	0x1861,
	0x2802,
	0x3823,
	0xc9cc,
	0xd9ed,
	0xe98e,
	0xf9af,
	0x8948,
	0x9969,
	0xa90a,
	0xb92b,
	0x5af5,
	0x4ad4,
	0x7ab7,
	0x6a96,
	0x1a71,
	0x0a50,
	0x3a33,
	0x2a12,
	0xdbfd,
	0xcbdc,
	0xfbbf,
	0xeb9e,
	0x9b79,
	0x8b58,
	0xbb3b,
	0xab1a,
	0x6ca6,
	0x7c87,
	0x4ce4,
	0x5cc5,
	0x2c22,
	0x3c03,
	0x0c60,
	0x1c41,
	0xedae,
	0xfd8f,
	0xcdec,
	0xddcd,
	0xad2a,
	0xbd0b,
	0x8d68,
	0x9d49,
	0x7e97,
	0x6eb6,
	0x5ed5,
	0x4ef4,
	0x3e13,
	0x2e32,
	0x1e51,
	0x0e70,
	0xff9f,
	0xefbe,
	0xdfdd,
	0xcffc,
	0xbf1b,
	0xaf3a,
	0x9f59,
	0x8f78,
	0x9188,
	0x81a9,
	0xb1ca,
	0xa1eb,
	0xd10c,
	0xc12d,
	0xf14e,
	0xe16f,
	0x1080,
	0x00a1,
	0x30c2,
	0x20e3,
	0x5004,
	0x4025,
	0x7046,
	0x6067,
	0x83b9,
	0x9398,
	0xa3fb,
	0xb3da,
	0xc33d,
	0xd31c,
	0xe37f,
	0xf35e,
	0x02b1,
	0x1290,
	0x22f3,
	0x32d2,
	0x4235,
	0x5214,
	0x6277,
	0x7256,
	0xb5ea,
	0xa5cb,
	0x95a8,
	0x8589,
	0xf56e,
	0xe54f,
	0xd52c,
	0xc50d,
	0x34e2,
	0x24c3,
	0x14a0,
	0x0481,
	0x7466,
	0x6447,
	0x5424,
	0x4405,
	0xa7db,
	0xb7fa,
	0x8799,
	0x97b8,
	0xe75f,
	0xf77e,
	0xc71d,
	0xd73c,
	0x26d3,
	0x36f2,
	0x0691,
	0x16b0,
	0x6657,
	0x7676,
	0x4615,
	0x5634,
	0xd94c,
	0xc96d,
	0xf90e,
	0xe92f,
	0x99c8,
	0x89e9,
	0xb98a,
	0xa9ab,
	0x5844,
	0x4865,
	0x7806,
	0x6827,
	0x18c0,
	0x08e1,
	0x3882,
	0x28a3,
	0xcb7d,
	0xdb5c,
	0xeb3f,
	0xfb1e,
	0x8bf9,
	0x9bd8,
	0xabbb,
	0xbb9a,
	0x4a75,
	0x5a54,
	0x6a37,
	0x7a16,
	0x0af1,
	0x1ad0,
	0x2ab3,
	0x3a92,
	0xfd2e,
	0xed0f,
	0xdd6c,
	0xcd4d,
	0xbdaa,
	0xad8b,
	0x9de8,
	0x8dc9,
	0x7c26,
	0x6c07,
	0x5c64,
	0x4c45,
	0x3ca2,
	0x2c83,
	0x1ce0,
	0x0cc1,
	0xef1f,
	0xff3e,
	0xcf5d,
	0xdf7c,
	0xaf9b,
	0xbfba,
	0x8fd9,
	0x9ff8,
	0x6e17,
	0x7e36,
	0x4e55,
	0x5e74,
	0x2e93,
	0x3eb2,
	0x0ed1,
	0x1ef0,
};

void printBuffer(const uint8_t *buffer, uint32_t size)
{
	printf("\r\nBuffer\r\n\r\n");
	for (uint32_t i = 0; i < size; i++)
	{
		printf("%02X ", buffer[i]); // Print each byte in hexadecimal format
	}
	printf("\r\n\r\n"); // Print a newline character to separate the output
}

uint8_t BoardV_Read(void)
{
	uint32_t idr = GPIOE->IDR;
	uint8_t v0 = (idr >> 9) & 1u; // PE9
	uint8_t v1 = (idr >> 8) & 1u; // PE8
	uint8_t v2 = (idr >> 7) & 1u; // PE7
	return (v0 << 0) | (v1 << 1) | (v2 << 2);
}

uint16_t util_crc16(const uint8_t *buf, uint32_t size)
{
	uint16_t crc = 0xFFFF;

	for (int i = 0; i < size; i++)
	{
		uint8_t byte = buf[i];
		crc = (crc << 8) ^ crc16_tab[(crc >> 8) ^ byte];
	}

	return crc;
}

uint16_t util_hw_crc16(uint8_t *buf, uint32_t size)
{
	uint32_t uwCRCValue = HAL_CRC_Accumulate(&hcrc, (uint32_t *)buf, size);
	printf("uwCRCValue 0x%08lx\r\n", uwCRCValue);
	return (uint16_t)uwCRCValue;
}

void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
	uint32_t cycles_per_us = SystemCoreClock / 1000000;
	uint32_t start = DWT->CYCCNT;
	uint32_t delay_cycles = us * cycles_per_us;

	while ((DWT->CYCCNT - start) < delay_cycles)
		;
}

void GPIO_SetHiZ(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Disable the pin before reconfiguring
	HAL_GPIO_DeInit(GPIOx, GPIO_Pin);

	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;		 // Input mode
	GPIO_InitStruct.Pull = GPIO_NOPULL;			 // No pull-up, no pull-down
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Speed doesn't matter for input

	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_SetOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Disable the pin before reconfiguring
	HAL_GPIO_DeInit(GPIOx, GPIO_Pin);

	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;	 // Input mode
	GPIO_InitStruct.Pull = GPIO_NOPULL;			 // No pull-up, no pull-down
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Speed doesn't matter for input

	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
}

double temperature_to_resistance(double temp_c)
{
    if (temp_c < MIN_TEMP || temp_c > MAX_TEMP)
    {
        return -1;  // Out of range indicator
    }

    int lower_index = (int)(temp_c) - MIN_TEMP;
    int upper_index = lower_index + 1;

    /* Exact integer temperature */
    if (temp_c == (int)temp_c)
    {
        return resistance_table[lower_index];
    }

    /* Linear interpolation */
    double r1 = resistance_table[lower_index];
    double r2 = resistance_table[upper_index];

    double fraction = temp_c - (int)temp_c;

    return r1 + (r2 - r1) * fraction;
}

/* ==========================================================
   Solve for Thermistor Resistance (R_TH) from ADC Voltage
   ========================================================== */
double solve_R_TH(double v)
{
    double term = (v / ((V_REF / 2.0) * R_3))
                  - (1.0 / R_3)
                  + (1.0 / R_1);

    double R_TH = (1.0 / term) - R_2;

    return R_TH;
}

/* ==========================================================
   Solve for ADC Voltage (v) from Thermistor Resistance
   ========================================================== */
double solve_v(double R_TH)
{
    double v = ( (1.0 / (R_TH + R_2))
               + (1.0 / R_3)
               - (1.0 / R_1) )
               * (V_REF / 2.0)
               * R_3;

    return v;
}
