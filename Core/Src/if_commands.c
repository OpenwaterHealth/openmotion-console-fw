/*
 * if_commands.c
 *
 *  Created as refactor target for command processing from uart_comms.c
 */

#include "version.h"
#include "main.h"
#include "common.h"
#include "uart_comms.h"
#include "if_fpga_prog.h"
#include "utils.h"
#include "tca9548a.h"
#include "console_i2c_health.h"
#include "trigger.h"
#include "fan_driver.h"
#include "ads7828.h"
#include "ad5761r.h"
#include "ads7924.h"
#include "max31875.h"
#include "led_driver.h"
#include "motion_config.h"
#include "msg_queue.h"
#include "pdc_buffer.h"
#include "odometer.h"
#include "console_serial.h"
#include "logging.h"

#include <string.h>

#define PDU_N 16

ConsoleTemperatures consoleTemps;
static uint8_t board_id;

typedef struct
{
    uint16_t raws[PDU_N];
    float vals[PDU_N];
} PDUFields_t;

typedef union
{
    PDUFields_t f;
    uint8_t bytes[sizeof(PDUFields_t)];
} PDUFrame_t;

static PDUFrame_t pdu_frame;

extern ADS7924_HandleTypeDef tec_ads;
extern ADS7828_HandleTypeDef adc_mon[2];

extern FAN_Driver fan;
extern bool _enter_dfu;

extern ad5761r_dev tec_dac;
static uint8_t temp_val_buf[4] = {0};
static uint8_t tec_adc_buf[16] = {0}; // 4 floats × 4 bytes each

static uint8_t last_fan_rpm[2] = {0, 0};

/* The three fans expose a tach feedback signal on dedicated GPIO inputs:
 *   index 1 -> FAN_TOP_GD2 (PE1)
 *   index 2 -> FAN_TOP_GD3 (PE14)
 *   index 3 -> FAN_TOP_GD4 (PE15)
 * The line is a ~50% duty square wave at 2 pulses per revolution; the
 * useful information is the frequency, not the duty cycle. We count
 * rising edges in a fixed sample window and convert to RPM.
 *
 * An unplugged fan sits high (board pull-up) with no edges, which
 * naturally gives 0 RPM. */
#define FAN_SAMPLE_MS 100U
#define FAN_PULSES_PER_REV 2U

static uint16_t fan_measure_rpm(uint8_t index)
{
    GPIO_TypeDef *port;
    uint16_t pin;
    switch (index)
    {
    case 1:
        port = FAN_TOP_GD2_GPIO_Port;
        pin = FAN_TOP_GD2_Pin;
        break;
    case 2:
        port = FAN_TOP_GD3_GPIO_Port;
        pin = FAN_TOP_GD3_Pin;
        break;
    case 3:
        port = FAN_TOP_GD4_GPIO_Port;
        pin = FAN_TOP_GD4_Pin;
        break;
    default:
        return 0;
    }

    uint32_t edges = 0;
    GPIO_PinState prev = HAL_GPIO_ReadPin(port, pin);
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < FAN_SAMPLE_MS)
    {
        GPIO_PinState now = HAL_GPIO_ReadPin(port, pin);
        if (now == GPIO_PIN_SET && prev == GPIO_PIN_RESET)
        {
            edges++;
        }
        prev = now;
    }

    /* edges over FAN_SAMPLE_MS -> Hz = edges * (1000 / FAN_SAMPLE_MS)
     * RPM = Hz * 60 / pulses_per_rev */
    uint32_t rpm = (edges * (1000U / FAN_SAMPLE_MS) * 60U) / FAN_PULSES_PER_REV;
    if (rpm > 0xFFFFU)
        rpm = 0xFFFFU;
    return (uint16_t)rpm;
}
/* OW_CMD_HWID reply: 96-bit UID in words 0-2, word 3 is constant zero
 * padding. The reply is 16 bytes, so the array must be too — a 3-word
 * array leaked the adjacent static into bytes 12-15 (#53). */
static uint32_t id_words[4] = {0};

/* OW_CMD_BOOT_INFO reply. Reports the RUNTIME vector-table base (SCB->VTOR),
 * not the compile-time BARE_METAL_BUILD flag, so the device evidences where it
 * is actually executing: 0x08000000 => bare-metal, 0x08020400 => bootloader
 * slot. The host derives the boot mode. Packed, little-endian on the wire —
 * byte-identical to the sensor's reply (openmotion-sensor-fw #110) so one host
 * parser covers both boards. */
typedef struct __attribute__((packed)) {
	uint8_t  struct_version;   /* 1 */
	uint8_t  reserved[3];
	uint32_t vtor;             /* SCB->VTOR */
	uint32_t flash_base;       /* image link base */
} boot_info_t;
static boot_info_t boot_info = {0};

static uint8_t i2c_list[10] = {0};
static uint8_t i2c_data[0xff] = {0};
static uint32_t last_fsync_count = 0;
static uint32_t last_lsync_count = 0;
static uint32_t last_system_odo = 0;
static uint32_t last_laser_odo = 0;

static float tecadc_last_volts[4];
static uint16_t tecadc_last_raw[4];

static char retTriggerJson[0xFF];
static float tec_setpoint = 0.0;
static float temp_val = 0;

extern volatile bool _trip_set;
extern volatile bool _tec_sample_lock;
extern volatile TecStats last_tec_stats;

static _Bool process_controller_command(UartPacket *uartResp, UartPacket *cmd)
{
    _Bool ret = true;
    uartResp->command = cmd->command;
    switch (cmd->command)
    {
    case OW_CTRL_I2C_SCAN:
        uartResp->command = OW_CTRL_I2C_SCAN;
        if (cmd->data_len != 2)
        {
            uartResp->packet_type = OW_ERROR;
            uartResp->data_len = 0;
            uartResp->data = NULL;
        }
        else
        {
            int iRet;
            memset(i2c_list, 0, 10);
            iRet = TCA9548A_scan_channel(cmd->data[0], cmd->data[1], i2c_list, 10, false);
            if (iRet < 0)
            {
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            }
            else
            {
                uartResp->data_len = (uint16_t)iRet;
                uartResp->data = i2c_list;
            }
        }
        break;
    case OW_CTRL_I2C_STATUS:
        uartResp->command = OW_CTRL_I2C_STATUS;
        /* reserved==1 -> re-run the (passive) scan live before returning. */
        if (cmd->reserved == 1)
        {
            console_i2c_health_scan();
        }
        uartResp->data_len = sizeof(console_i2c_health_t);
        uartResp->data = (uint8_t *)console_i2c_health_get();
        break;
    case OW_CTRL_SET_IND:
        uartResp->command = OW_CTRL_SET_IND;
        if (uartResp->reserved > 3)
        {
            uartResp->packet_type = OW_ERROR;
            uartResp->data_len = 0;
            uartResp->data = NULL;
        }
        else
        {
            LED_RGB_SET(cmd->reserved);
        }
        break;
    case OW_CTRL_GET_IND:
        uartResp->command = OW_CTRL_GET_IND;
        uartResp->reserved = LED_RGB_GET();
        break;
    case OW_CTRL_SET_FAN:
        uartResp->command = OW_CTRL_SET_FAN;
        if (cmd->addr > 1 || cmd->data_len != 1)
        {
            uartResp->packet_type = OW_ERROR;
            uartResp->data_len = 0;
            uartResp->data = NULL;
        }
        else
        {
            printf("Set fan to: %d\r\n", cmd->data[0]);
            FAN_SetManualPWM(&fan, cmd->data[0]);
        }
        break;
    case OW_CTRL_GET_FAN:
        uartResp->command = OW_CTRL_GET_FAN;
        if (cmd->addr < 1 || cmd->addr > 3)
        {
            uartResp->packet_type = OW_ERROR;
            uartResp->data_len = 0;
            uartResp->data = NULL;
        }
        else
        {
            uint16_t rpm = fan_measure_rpm(cmd->addr);
            last_fan_rpm[0] = (uint8_t)(rpm & 0xFF);
            last_fan_rpm[1] = (uint8_t)((rpm >> 8) & 0xFF);
            uartResp->data_len = 2;
            uartResp->data = last_fan_rpm;
        }
        break;
    case OW_CTRL_I2C_RD:
        uartResp->command = OW_CTRL_I2C_RD;
        if (cmd->data_len != 5)
        {
            uartResp->packet_type = OW_ERROR;
            uartResp->data_len = 0;
            uartResp->data = NULL;
        }
        else
        {
            uint8_t mux_index = cmd->data[0];
            uint8_t channel = cmd->data[1];
            uint8_t i2c_addr = cmd->data[2];
            uint8_t reg_addr = cmd->data[3];
            uint8_t data_len = cmd->data[4];
            memset(i2c_data, 0, 0xff);
            int8_t tca_ret = TCA9548A_Read_Data(mux_index, channel, i2c_addr, reg_addr, data_len, i2c_data);
            if (tca_ret != TCA9548A_OK)
            {
                printf("error selecting channel\r\n");
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            }
            else
            {
                uartResp->data_len = data_len;
                uartResp->data = i2c_data;
            }
        }
        break;
    case OW_CTRL_I2C_WR:
        uartResp->command = OW_CTRL_I2C_WR;
        if (cmd->data_len < 5)
        {
            uartResp->packet_type = OW_ERROR;
            uartResp->data_len = 0;
            uartResp->data = NULL;
        }
        else
        {
            uint8_t mux_index = cmd->data[0];
            uint8_t channel = cmd->data[1];
            uint8_t i2c_addr = cmd->data[2];
            uint8_t reg_addr = cmd->data[3];
            uint8_t data_len = cmd->data[4];
            uint8_t *pData = &cmd->data[5];
            // printf("I2C Write MUX: %d CH: %d ADDR: 0x%02X REG: 0x%02X LEN: %d ", mux_index, channel, i2c_addr, reg_addr, data_len);
            // printBufferAsArr(pData, data_len);
            int8_t tca_ret = TCA9548A_Write_Data(mux_index, channel, i2c_addr, reg_addr, data_len, pData);
            if (tca_ret != TCA9548A_OK)
            {
                printf("error selecting channel\r\n");
                uartResp->packet_type = OW_ERROR;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            }
        }
        break;
    case OW_CTRL_SET_TRIG:
        uartResp->command = OW_CTRL_SET_TRIG;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        uartResp->data_len = 0;

        if (Trigger_SetConfigFromJSON((char *)cmd->data, cmd->data_len) != HAL_OK)
        {
            uartResp->packet_type = OW_ERROR;
        }
        else
        {
            memset(retTriggerJson, 0, sizeof(retTriggerJson));
            if (Trigger_GetConfigToJSON(retTriggerJson, 0xFF) != HAL_OK)
            {
                uartResp->packet_type = OW_ERROR;
            }
            else
            {
                uartResp->data_len = strlen(retTriggerJson);
                uartResp->data = (uint8_t *)retTriggerJson;
            }
        }

        break;

    case OW_CTRL_GET_TRIG:
        uartResp->command = OW_CTRL_GET_TRIG;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        uartResp->data_len = 0;
        memset(retTriggerJson, 0, sizeof(retTriggerJson));
        if (Trigger_GetConfigToJSON(retTriggerJson, 0xFF) != HAL_OK)
        {
            uartResp->packet_type = OW_ERROR;
        }
        else
        {
            uartResp->data_len = strlen(retTriggerJson);
            uartResp->data = (uint8_t *)retTriggerJson;
        }
        break;
    case OW_CTRL_START_TRIG:
        uartResp->command = OW_CTRL_START_TRIG;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        uartResp->data_len = 0;

        Trigger_Start(); // sets the indicator blue on success
        break;
    case OW_CTRL_STOP_TRIG:
        uartResp->command = OW_CTRL_STOP_TRIG;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        uartResp->data_len = 0;

        Trigger_Stop(); // returns the indicator to idle (green)
        break;
    case OW_CTRL_GET_FSYNC:
        uartResp->command = OW_CTRL_GET_FSYNC;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        uartResp->data_len = 4;
        last_fsync_count = get_fsync_pulse_count();
        uartResp->data = (uint8_t *)&last_fsync_count;
        break;
    case OW_CTRL_GET_LSYNC:
        uartResp->command = OW_CTRL_GET_LSYNC;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        uartResp->data_len = 4;
        last_lsync_count = get_lsync_pulse_count();
        uartResp->data = (uint8_t *)&last_lsync_count;
        break;
    case OW_CTRL_TEC_DAC:
        uartResp->command = OW_CTRL_TEC_DAC;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        if (cmd->reserved == 0)
        {
            uint16_t reg_data = 0;
            if (ad5761r_register_readback(&tec_dac, CMD_RD_DAC_REG, &reg_data) == HAL_OK)
            {
                temp_val = 0;
                temp_val = code_to_volts(&tec_dac, reg_data);
                memcpy(temp_val_buf, &temp_val, sizeof(float));

                uartResp->data = (uint8_t *)temp_val_buf;
                uartResp->data_len = sizeof(float);
            }
            else
            {
                uartResp->data_len = 0;
                uartResp->data = NULL;
                uartResp->packet_type = OW_ERROR;
            }
        }
        else if (cmd->reserved == 1 && cmd->data_len == 4)
        {
            float set_voltage;
            memcpy(&set_voltage, cmd->data, sizeof(float));
            uint16_t reg_data = 0;
            reg_data = volts_to_code(&tec_dac, set_voltage);

            if (ad5761r_write_update_dac_register(&tec_dac, reg_data) != 0)
            {
                printf("TEC DAC Failed to set DAC Voltage\r\n");
                uartResp->data_len = 0;
                uartResp->data = NULL;
                uartResp->packet_type = OW_ERROR;
            }
            else
            {
                tec_setpoint = set_voltage;
                memcpy(temp_val_buf, &tec_setpoint, sizeof(float));
                uartResp->data_len = sizeof(float);
                uartResp->data = (uint8_t *)temp_val_buf;
            }
        }
        else
        {
            uartResp->data_len = 0;
            uartResp->data = NULL;
            uartResp->packet_type = OW_UNKNOWN;
        }

        break;
    case OW_CTRL_GET_TEMPS:
        uartResp->command = OW_CTRL_GET_TEMPS;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        {
            /* Drain the telemetry ring buffer — up to 1024 bytes of samples. */
            static TelemetrySample s_telem_resp_buf[1024U / sizeof(TelemetrySample)];
            size_t n = telemetry_read(s_telem_resp_buf,
                                      sizeof(s_telem_resp_buf) / sizeof(TelemetrySample));
            uartResp->data_len = (uint16_t)(n * sizeof(TelemetrySample));
            uartResp->data = (uint8_t *)s_telem_resp_buf;
        }

        break;
    case OW_CTRL_TEC_STATUS:
        uartResp->command = OW_CTRL_TEC_STATUS;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;

        _tec_sample_lock = true;

        uartResp->data_len = sizeof(last_tec_stats);
        uartResp->data = (uint8_t *)&last_tec_stats;

        break;
    case OW_CTRL_TECADC:
        uartResp->command = OW_CTRL_TECADC;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        if (uartResp->reserved > 4)
        {
            uartResp->data_len = 0;
            uartResp->packet_type = OW_UNKNOWN;
        }
        else
        {
            memset(tecadc_last_volts, 0, 16);
            if (cmd->reserved == 4)
            {
                if (ADS7924_ReadAllRaw(&tec_ads, tecadc_last_raw, 100) != ADS7924_OK)
                {
                    printf("Failed to read ADC channels\r\n");
                    uartResp->data_len = 0;
                    uartResp->packet_type = OW_UNKNOWN;
                }
                else
                {
                    for (int i = 0; i < 4; i++)
                    {
                        tecadc_last_volts[i] = ADS7924_CodeToVolts(&tec_ads, tecadc_last_raw[i]);
                    }
                    memcpy(tec_adc_buf, tecadc_last_volts, sizeof(tecadc_last_volts));
                    uartResp->data_len = sizeof(tecadc_last_volts);
                    uartResp->data = tec_adc_buf;
                }
                break;
            }

            if (ADS7924_ReadVoltage(&tec_ads, uartResp->reserved, &tecadc_last_volts[uartResp->reserved], 100) != ADS7924_OK)
            {
                printf("Failed to read ADC channel %d\r\n", uartResp->reserved);
                uartResp->data_len = 0;
                uartResp->packet_type = OW_UNKNOWN;
            }
            else
            {
                memcpy(tec_adc_buf, &tecadc_last_volts[uartResp->reserved], sizeof(float));
                uartResp->data_len = 4;
                uartResp->data = (uint8_t *)tec_adc_buf;
            }
        }
        break;
    case OW_CTRL_BOARDID:
        uartResp->command = OW_CTRL_BOARDID;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        uartResp->data_len = 1;
        board_id = BoardV_Read();
        uartResp->data = (uint8_t *)(&board_id);
        break;
    case OW_CTRL_PDUMON:
        uartResp->command = OW_CTRL_PDUMON;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        if (ADS7828_ReadAllChannels2(&adc_mon[0], &pdu_frame.f.raws[0], &pdu_frame.f.vals[0]) != HAL_OK)
        {
            printf("Failed to read ADC MON 0\r\n");
            uartResp->data_len = 0;
            uartResp->data = NULL;
            uartResp->packet_type = OW_ERROR;
            break;
        }
        if (ADS7828_ReadAllChannels2(&adc_mon[1], &pdu_frame.f.raws[8], &pdu_frame.f.vals[8]) != HAL_OK)
        {
            printf("Failed to read ADC MON 1\r\n");
            uartResp->data_len = 0;
            uartResp->data = NULL;
            uartResp->packet_type = OW_ERROR;
            break;
        }
        uartResp->data_len = (uint16_t)sizeof(pdu_frame);
        uartResp->data = pdu_frame.bytes;
        break;
    case OW_CTRL_GET_PDC_BUFFER: {
        uartResp->command = OW_CTRL_GET_PDC_BUFFER;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;

        /* Request payload: 1 byte = max_samples (clamped to 64). */
        uint8_t requested = (cmd->data_len >= 1) ? cmd->data[0] : 64;
        if (requested == 0 || requested > 64) requested = 64;

        /* Response layout: [dropped:u16 LE][count:u8][count * sizeof(pdc_sample_t)] */
        static pdc_sample_t drained[64];
        size_t n = pdc_buffer_drain(drained, requested);
        uint16_t dropped = pdc_buffer_dropped_since_last_drain();

        static uint8_t resp[3 + 64 * sizeof(pdc_sample_t)];
        resp[0] = (uint8_t)(dropped & 0xFF);
        resp[1] = (uint8_t)((dropped >> 8) & 0xFF);
        resp[2] = (uint8_t)n;
        memcpy(&resp[3], drained, n * sizeof(pdc_sample_t));

        uartResp->data_len = (uint16_t)(3 + n * sizeof(pdc_sample_t));
        uartResp->data = resp;
    } break;
    case OW_CTRL_GET_SYSTEM_ODO:
        uartResp->command = OW_CTRL_GET_SYSTEM_ODO;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        uartResp->data_len = 4;
        last_system_odo = Odometer_Get_System_Minutes();
        uartResp->data = (uint8_t *)&last_system_odo;
        break;
    case OW_CTRL_GET_LASER_ODO:
        uartResp->command = OW_CTRL_GET_LASER_ODO;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        uartResp->data_len = 4;
        last_laser_odo = Odometer_Get_Laser_Pulses();
        uartResp->data = (uint8_t *)&last_laser_odo;
        break;
    case OW_CTRL_GET_EEPROM_EUI: {
        /* Return the external EEPROM's factory EUI-48 (6 bytes, MSB first) as a
         * unique per-board serial. OW_ERROR if the EEPROM wasn't detected. */
        uartResp->command = OW_CTRL_GET_EEPROM_EUI;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        static uint8_t eui_resp[6];
        if (Odometer_Get_EUI48(eui_resp) == HAL_OK) {
            uartResp->data_len = sizeof(eui_resp);
            uartResp->data = eui_resp;
        } else {
            uartResp->data_len = 0;
            uartResp->packet_type = OW_ERROR;
        }
        break;
    }
    case OW_CTRL_RESET_ODO: {
        uartResp->command = OW_CTRL_RESET_ODO;
        uartResp->addr = cmd->addr;
        uartResp->reserved = cmd->reserved;
        /* Request payload: 1 byte = target (0=system, 1=laser, 2=both).
         * No payload defaults to "both" for caller convenience. */
        OdoResetTarget target = ODO_RESET_BOTH;
        if (cmd->data_len >= 1) {
            if (cmd->data[0] > (uint8_t)ODO_RESET_BOTH) {
                uartResp->data_len = 0;
                uartResp->packet_type = OW_ERROR;
                break;
            }
            target = (OdoResetTarget)cmd->data[0];
        }
        static uint8_t reset_resp;
        reset_resp = (Odometer_Reset(target) == HAL_OK) ? 0 : 1;
        uartResp->data_len = 1;
        uartResp->data = &reset_resp;
        if (reset_resp != 0) {
            uartResp->packet_type = OW_ERROR;
        }
    } break;
    default:
        uartResp->data_len = 0;
        uartResp->packet_type = OW_UNKNOWN;
        break;
    }

    return ret;
}

_Bool process_if_command(UartPacket *uartResp, UartPacket *cmd)
{
    uartResp->id = cmd->id;
    uartResp->packet_type = OW_RESP;
    uartResp->addr = 0;
    uartResp->reserved = 0;
    uartResp->data_len = 0;
    uartResp->data = 0;
    switch (cmd->packet_type)
    {
    case OW_CMD:
        uartResp->command = cmd->command;
        switch (cmd->command)
        {
        case OW_CMD_PING:
            break;
        case OW_CMD_NOP:
            break;
        case OW_CMD_VERSION:
            uartResp->data_len = sizeof(FW_VERSION_STRING);
            uartResp->data = (uint8_t *)FW_VERSION_STRING;
            break;
        case OW_CMD_ECHO:
            uartResp->data_len = cmd->data_len;
            uartResp->data = cmd->data;
            break;
        case OW_CMD_HWID:
            id_words[0] = HAL_GetUIDw0();
            id_words[1] = HAL_GetUIDw1();
            id_words[2] = HAL_GetUIDw2();
            id_words[3] = 0u;
            uartResp->data_len = sizeof(id_words);
            uartResp->data = (uint8_t *)&id_words;
            break;
        case OW_CMD_BOOT_INFO:
            /* Runtime vector-table base classifies the boot mode; see the
             * boot_info_t comment. Host maps 0x08000000 -> bare-metal,
             * 0x08020400 -> bootloader slot. */
            boot_info.struct_version = 1u;
            boot_info.reserved[0] = 0u;
            boot_info.reserved[1] = 0u;
            boot_info.reserved[2] = 0u;
            boot_info.vtor       = SCB->VTOR;
            boot_info.flash_base = SCB->VTOR;
            uartResp->data_len = sizeof(boot_info);
            uartResp->data = (uint8_t *)&boot_info;
            break;
        case OW_CMD_DEBUG_FLAGS:
            uartResp->command = OW_CMD_DEBUG_FLAGS;
            /* reserved bit0: 0 = get, 1 = set (payload = uint32 little-endian).
             * Either way we respond with the current flags. */
            if ((cmd->reserved & 0x01) != 0)
            {
                if (cmd->data_len != sizeof(uint32_t))
                {
                    uartResp->packet_type = OW_ERROR;
                    uartResp->data_len = 0;
                    uartResp->data = NULL;
                    break;
                }
                uint32_t new_flags = 0;
                memcpy(&new_flags, cmd->data, sizeof(new_flags));
                logging_set_debug_flags(new_flags);
            }
            {
                static uint32_t debug_flags_resp;
                debug_flags_resp = logging_get_debug_flags();
                uartResp->data_len = sizeof(debug_flags_resp);
                uartResp->data = (uint8_t *)&debug_flags_resp;
            }
            break;
        case OW_CMD_MESSAGES:
            uartResp->command = OW_CMD_MESSAGES;
            {
                const uint16_t max_payload = (uint16_t)(COMMAND_MAX_SIZE - 12U);
                static uint8_t out_buf[COMMAND_MAX_SIZE];
                size_t out_idx = 0;
                char tmp_msg[MQ_MAX_MSG_SIZE];
                size_t msg_len = 0;

                while (mq_peek(tmp_msg, sizeof(tmp_msg), &msg_len))
                {
                    /* check if message + optional separator fits */
                    size_t sep = (out_idx == 0) ? 0 : 1; /* newline between messages */
                    if ((out_idx + sep + msg_len) > (size_t)max_payload)
                    {
                        break; /* won't fit */
                    }

                    /* pop message and append */
                    if (!mq_pop(tmp_msg, sizeof(tmp_msg), &msg_len))
                    {
                        break; /* race or error */
                    }

                    if (sep)
                    {
                        out_buf[out_idx++] = '\n';
                    }
                    memcpy(&out_buf[out_idx], tmp_msg, msg_len);
                    out_idx += msg_len;
                }

                if (out_idx == 0)
                {
                    uartResp->data_len = 0;
                    uartResp->data = NULL;
                }
                else
                {
                    uartResp->data_len = (uint16_t)out_idx;
                    uartResp->data = out_buf;
                }
            }
            break;
        case OW_CMD_TOGGLE_LED:
            break;
        case OW_CMD_USR_CFG:
            // reserved == 0: READ
            // reserved == 1: WRITE (cmd->data is JSON text)
            if (cmd->reserved == 0)
            {
                const uint8_t *wire_buf = NULL;
                uint16_t wire_len = 0;
                const uint16_t max_payload = (uint16_t)(COMMAND_MAX_SIZE - 12U); // framing overhead in txBuffer
                if (motion_cfg_wire_read(&wire_buf, &wire_len, max_payload) != HAL_OK || wire_buf == NULL)
                {
                    uartResp->packet_type = OW_ERROR;
                    uartResp->data_len = 0;
                    uartResp->data = NULL;
                    break;
                }

                uartResp->data_len = wire_len;
                uartResp->data = (uint8_t *)wire_buf;
            }
            else if (cmd->reserved == 1)
            {
                if (cmd->data == NULL || cmd->data_len == 0)
                {
                    uartResp->packet_type = OW_ERROR;
                    uartResp->data_len = 0;
                    uartResp->data = NULL;
                    break;
                }

                if (motion_cfg_wire_write(cmd->data, cmd->data_len) != HAL_OK)
                {
                    uartResp->packet_type = OW_ERROR;
                    uartResp->data_len = 0;
                    uartResp->data = NULL;
                    break;
                }

                // Apply updated config to runtime variables.
                motion_cfg_apply_settings();

                // Return the updated header as an ACK payload.
                const uint8_t *wire_buf = NULL;
                uint16_t wire_len = 0;
                const uint16_t max_payload = (uint16_t)(COMMAND_MAX_SIZE - 12U);
                if (motion_cfg_wire_read(&wire_buf, &wire_len, max_payload) != HAL_OK || wire_buf == NULL)
                {
                    uartResp->packet_type = OW_ERROR;
                    uartResp->data_len = 0;
                    uartResp->data = NULL;
                    break;
                }
                uartResp->data_len = (uint16_t)sizeof(motion_cfg_wire_hdr_t);
                uartResp->data = (uint8_t *)wire_buf;
            }
            else
            {
                uartResp->packet_type = OW_UNKNOWN;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            }
            break;
        case OW_CMD_SERIAL:
            // reserved == 0: READ  -> payload = ASCII serial (data_len 0 == unprogrammed)
            // reserved == 1: WRITE (guarded; OW_ERROR if already programmed)
            // reserved == 2: WRITE (force)
            if (cmd->reserved == 0)
            {
                static char serial_buf[SERIAL_MAX_LEN];
                uint8_t serial_len = 0;
                if (Serial_Read(serial_buf, &serial_len) != HAL_OK)
                {
                    // Unprogrammed (or no EEPROM): empty payload, not an error.
                    uartResp->data_len = 0;
                    uartResp->data = NULL;
                }
                else
                {
                    uartResp->data_len = serial_len;
                    uartResp->data = (uint8_t *)serial_buf;
                }
            }
            else if (cmd->reserved == 1 || cmd->reserved == 2)
            {
                _Bool force = (cmd->reserved == 2);
                if (cmd->data == NULL || cmd->data_len == 0 ||
                    cmd->data_len > SERIAL_MAX_LEN)
                {
                    uartResp->packet_type = OW_ERROR;
                    uartResp->data_len = 0;
                    uartResp->data = NULL;
                    break;
                }
                HAL_StatusTypeDef st =
                    Serial_Write((const char *)cmd->data, (uint8_t)cmd->data_len, force);
                if (st != HAL_OK)
                {
                    // HAL_BUSY == refused overwrite; HAL_ERROR == bad input / EEPROM fail.
                    uartResp->packet_type = OW_ERROR;
                    uartResp->data_len = 0;
                    uartResp->data = NULL;
                    break;
                }
                // ACK with empty payload on success.
                uartResp->data_len = 0;
                uartResp->data = NULL;
            }
            else
            {
                uartResp->packet_type = OW_UNKNOWN;
                uartResp->data_len = 0;
                uartResp->data = NULL;
            }
            break;
        case OW_CMD_RESET:
            uartResp->command = cmd->command;
            uartResp->addr = cmd->addr;
            uartResp->reserved = cmd->reserved;
            uartResp->data_len = 0;

            __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);
            __HAL_TIM_SET_COUNTER(&htim15, 0);
            if (HAL_TIM_Base_Start_IT(&htim15) != HAL_OK)
            {
                uartResp->packet_type = OW_ERROR;
            }
            break;

        case OW_CMD_DFU:
#if defined(BARE_METAL_BUILD)
            printf("Enter DFU (STM32 ROM bootloader)\r\n");
#else
            printf("Enter DFU (custom bootloader)\r\n");
#endif
            uartResp->command = cmd->command;
            uartResp->addr = cmd->addr;
            uartResp->reserved = cmd->reserved;
            uartResp->data_len = 0;

            /* Enter DFU. The reset is deferred to the TIM15 callback so this
             * response is transmitted and peripherals shut down cleanly first.
             * The callback arms the DFU magic and issues NVIC_SystemReset():
             *   - bare-metal build: RAM magic -> CheckBootloaderFlag() jumps to
             *     the STM32 system-memory ROM DFU loader after reset.
             *   - custom-bootloader build: RTC->BKP7R magic -> the secure
             *     bootloader enters its own USB DFU instead of this image. */
            _enter_dfu = true;

            __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);
            __HAL_TIM_SET_COUNTER(&htim15, 0);
            if (HAL_TIM_Base_Start_IT(&htim15) != HAL_OK)
            {
                uartResp->packet_type = OW_ERROR;
            }
            break;
        default:
            break;
        }
        break;
    case OW_CONTROLLER:
        return process_controller_command(uartResp, cmd);
    case OW_FPGA_PROG:
        return process_fpga_prog_command(uartResp, cmd);
        break;
    default:
        uartResp->data_len = 0;
        uartResp->packet_type = OW_UNKNOWN;
        break;
    }

    return true;
}
