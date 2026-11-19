#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* Host stand-ins for the HAL pieces led_driver.c touches. Defining the real
 * main.h's include guard keeps it (and the STM32 HAL behind it) out. */
#define __MAIN_H
typedef struct { int unused; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
static GPIO_TypeDef port_a, port_d;
#define IND1_GPIO_Port (&port_a)
#define IND1_Pin 0x0008U
#define IND2_GPIO_Port (&port_d)
#define IND2_Pin 0x0010U
#define IND3_GPIO_Port (&port_d)
#define IND3_Pin 0x0020U

static int gpio_writes;
static void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState s)
{
    (void)port; (void)pin; (void)s;
    gpio_writes++;
}
static void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port; (void)pin;
    gpio_writes++;
}

/* pull in the source directly for the host build */
#include "../Core/Src/led_driver.c"

#define HALF LED_FAULT_BLINK_HALF_PERIOD_MS

int main(void) {
    /* No fault: status writes apply; a clear latch is a no-op. */
    LED_Indicator_Set(LED_GREEN);
    assert(LED_RGB_GET() == LED_GREEN);
    gpio_writes = 0;
    LED_Fault_Indicate(false, 1000);
    assert(LED_RGB_GET() == LED_GREEN && gpio_writes == 0);

    /* Fault latched while idle: first blue edge immediately, then 500/500. */
    LED_Fault_Indicate(true, 1000);
    assert(LED_RGB_GET() == LED_BLUE);
    gpio_writes = 0;
    LED_Fault_Indicate(true, 1000 + HALF - 1);
    assert(LED_RGB_GET() == LED_BLUE && gpio_writes == 0);   /* no rewrite mid-phase */
    LED_Fault_Indicate(true, 1000 + HALF);
    assert(LED_RGB_GET() == LED_NONE);
    LED_Fault_Indicate(true, 1000 + 2 * HALF - 1);
    assert(LED_RGB_GET() == LED_NONE);
    LED_Fault_Indicate(true, 1000 + 2 * HALF);
    assert(LED_RGB_GET() == LED_BLUE);

    /* While latched, trigger-stop idle green, trigger-start blue and host
     * OW_CTRL_SET_IND writes are all ignored, with no pin activity. */
    LED_Fault_Indicate(true, 1000 + 3 * HALF);
    assert(LED_RGB_GET() == LED_NONE);
    gpio_writes = 0;
    LED_Indicator_Set(LED_GREEN);
    LED_Indicator_Set(LED_BLUE);
    LED_Indicator_Set(LED_RED);
    assert(LED_RGB_GET() == LED_NONE && gpio_writes == 0);

    /* Latch clears: back to idle green, and status writes apply again. */
    LED_Fault_Indicate(false, 1000 + 3 * HALF + 10);
    assert(LED_RGB_GET() == LED_GREEN);
    gpio_writes = 0;
    LED_Fault_Indicate(false, 1000 + 3 * HALF + 35);
    assert(gpio_writes == 0);                                /* clear edge only once */
    LED_Indicator_Set(LED_BLUE);
    assert(LED_RGB_GET() == LED_BLUE);

    /* Fault during a scan (LED already blue): stays blue, phase restarts
     * at the new latch, and the trip's Trigger_Stop green is ignored. */
    gpio_writes = 0;
    LED_Fault_Indicate(true, 7777);
    assert(LED_RGB_GET() == LED_BLUE && gpio_writes == 0);
    LED_Indicator_Set(LED_GREEN);
    assert(LED_RGB_GET() == LED_BLUE);
    LED_Fault_Indicate(true, 7777 + HALF);
    assert(LED_RGB_GET() == LED_NONE);
    LED_Fault_Indicate(false, 9000);
    assert(LED_RGB_GET() == LED_GREEN);

    /* HAL tick wraparound mid-fault keeps the cadence. */
    const uint32_t t0 = 0xFFFFFF00u;
    LED_Fault_Indicate(true, t0);
    assert(LED_RGB_GET() == LED_BLUE);
    LED_Fault_Indicate(true, t0 + HALF - 1);                 /* wrapped */
    assert(LED_RGB_GET() == LED_BLUE);
    LED_Fault_Indicate(true, t0 + HALF);
    assert(LED_RGB_GET() == LED_NONE);
    LED_Fault_Indicate(true, t0 + 2 * HALF);
    assert(LED_RGB_GET() == LED_BLUE);
    LED_Fault_Indicate(false, t0 + 2 * HALF + 1);
    assert(LED_RGB_GET() == LED_GREEN);

    printf("led_driver host tests OK\n");
    return 0;
}
