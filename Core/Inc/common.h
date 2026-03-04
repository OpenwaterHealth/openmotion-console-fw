/*
 * common.h
 *
 *  Created on: Mar 12, 2024
 *      Author: gvigelet
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#define COMMAND_MAX_SIZE 2048

#include <stdio.h>
#include <stdint.h>

/*
 * UART Communication Protocol Packet Structure:
 *
 * | Start Byte | ID | Packet Type | Command | addr | reserved | Length | Payload | CRC16 | End Byte |
 *
 * Definitions:
 *
 * Start Byte:
 *   - A predefined byte to indicate the beginning of a packet.
 *   - Value: 0xAA (as per USTX_ProtocolTypes)
 *
 * ID:
 *   - ID for transaction used for response or continuation data.
 *
 * Packet Type:
 *   - Indicates the type of the packet being sent or received.
 *   - Possible values:
 *     - OW_ACK: Acknowledgment packet (Value: 0xE0)
 *     - OW_NAK: Negative acknowledgment packet (Value: 0xE1)
 *     - OW_CMD: Command packet (Value: 0xE2)
 *     - OW_RESP: Response packet (Value: 0xE3)
 *     - OW_DATA: Data packet (Value: 0xE4)
 *     - OW_JSON: JSON data packet (Value: 0xE5)
 *     - OW_BAD_CRC: Bad CRC error packet (Value: 0xEE)
 *     - OW_ERROR: General error packet (Value: 0xEF)
 *
 * Command:
 *   - Specifies the command or action to be taken.
 *   - Possible values:
 *     - USTX_NOP: No operation command (Value: 0xB0)
 *     - USTX_PING: Ping command (Value: 0xB1)
 *     - USTX_VERSION: Request for version information (Value: 0xB2)
 *     - USTX_ID: Request for ID information (Value: 0xB3)
 *
 * Length:
 *   - Indicates the length of the payload data.
 *
 * Payload:
 *   - Contains the actual data or information being sent or received.
 *   - Size can vary up to a maximum of COMMAND_MAX_SIZE (2048 bytes).
 *
 * CRC16:
 *   - A 16-bit Cyclic Redundancy Check value for error-checking purposes.
 *   - Helps in detecting errors in the transmitted data.
 *
 * End Byte:
 *   - A predefined byte to indicate the end of a packet.
 *   - Value: 0xDD (as per USTX_ProtocolTypes)
 *
 */



typedef enum {
	OW_START_BYTE = 0xAA,
	OW_END_BYTE = 0xDD,
} OWProtocolTypes;

typedef enum {
	OW_ACK = 0xE0,
	OW_NAK = 0xE1,
	OW_CMD = 0xE2,
	OW_RESP = 0xE3,
	OW_DATA = 0xE4,
	OW_JSON = 0xE5,
	OW_I2C_PASSTHRU = 0xE9,
	OW_CONTROLLER = 0xEA,
	OW_FPGA_PROG = 0xEB,
	OW_BAD_PARSE = 0xEC,
	OW_BAD_CRC = 0xED,
	OW_UNKNOWN = 0xEE,
	OW_ERROR = 0xEF,

} OWPacketTypes;

typedef enum {
	OW_CTRL_I2C_SCAN = 0x10,
	OW_CTRL_SET_IND = 0x11,
	OW_CTRL_GET_IND = 0x12,
	OW_CTRL_SET_TRIG = 0x13,
	OW_CTRL_GET_TRIG = 0x14,
	OW_CTRL_START_TRIG = 0x15,
	OW_CTRL_STOP_TRIG = 0x16,
	OW_CTRL_SET_FAN = 0x17,
	OW_CTRL_GET_FAN = 0x18,
	OW_CTRL_I2C_RD = 0x19,
	OW_CTRL_I2C_WR = 0x1A,
	OW_CTRL_GET_FSYNC = 0x1B,
	OW_CTRL_GET_LSYNC = 0x1C,
	OW_CTRL_TEC_DAC = 0x1D,
	OW_CTRL_READ_ADC = 0x1E,
	OW_CTRL_READ_GPIO = 0x1F,
	OW_CTRL_GET_TEMPS = 0x20,
	OW_CTRL_TECADC = 0x21,
	OW_CTRL_TEC_STATUS = 0x22,
	OW_CTRL_BOARDID = 0x23,
	OW_CTRL_PDUMON = 0x24,
	OW_CTRL_MCP42_SET_WIPER = 0x25,
	OW_CTRL_MCP42_SET_BOTH = 0x26,
	OW_CTRL_MCP42_SET_WIPERS = 0x27,
	OW_CTRL_MCP42_GET_WIPER = 0x28,
	OW_CTRL_MCP42_SHUTDOWN = 0x29,
	OW_CTRL_MCP42_WAKEUP = 0x2A,
	OW_CTRL_MCP42_SET_RES = 0x2B,
	OW_CTRL_MCP42_INC = 0x2C,
	OW_CTRL_MCP42_DEC = 0x2D,
} MotionControllerCommands;

typedef enum {
	OW_CODE_SUCCESS = 0x00,
	OW_CODE_IDENT_ERROR = 0xFD,
	OW_CODE_DATA_ERROR = 0xFE,
	OW_CODE_ERROR = 0xFF,
} OWErrorCodes;

typedef enum {
	OW_CMD_PING = 0x00,
	OW_CMD_PONG = 0x01,
	OW_CMD_VERSION = 0x02,
	OW_CMD_ECHO = 0x03,
	OW_CMD_TOGGLE_LED = 0x04,
	OW_CMD_HWID = 0x05,
	OW_CMD_MESSAGES = 0x09,
	OW_CMD_USR_CFG = 0x0A,
	OW_CMD_DFU = 0x0D,
	OW_CMD_NOP = 0x0E,
	OW_CMD_RESET = 0x0F,
} OWGlobalCommands;

typedef enum {
	OW_CMD_FPGA_PROG_OPEN          = 0x30, /* Open cfg interface in offline mode */
	OW_CMD_FPGA_PROG_ERASE         = 0x31, /* Erase flash sectors (1-byte mode bitmap) */
	OW_CMD_FPGA_PROG_CFG_RESET     = 0x32, /* Reset CFG address pointer */
	OW_CMD_FPGA_PROG_CFG_WRITE_PAGE= 0x33, /* Write one 16-byte CFG page */
	OW_CMD_FPGA_PROG_CFG_READ_PAGE = 0x34, /* Read back one 16-byte CFG page */
	OW_CMD_FPGA_PROG_UFM_RESET     = 0x35, /* Reset UFM address pointer */
	OW_CMD_FPGA_PROG_UFM_WRITE_PAGE= 0x36, /* Write one 16-byte UFM page */
	OW_CMD_FPGA_PROG_UFM_READ_PAGE = 0x37, /* Read back one 16-byte UFM page */
	OW_CMD_FPGA_PROG_FEATROW_WRITE = 0x38, /* Write feature row (8 feature + 2 feabits) */
	OW_CMD_FPGA_PROG_FEATROW_READ  = 0x39, /* Read back feature row (returns 10 bytes) */
	OW_CMD_FPGA_PROG_SET_DONE      = 0x3A, /* Set DONE bit */
	OW_CMD_FPGA_PROG_REFRESH       = 0x3B, /* Refresh / boot from flash */
	OW_CMD_FPGA_PROG_CLOSE         = 0x3C, /* Close cfg interface (abort path) */
	OW_CMD_FPGA_PROG_CFG_WRITE_PAGES = 0x3D, /* Write N 16-byte CFG pages (N*16 bytes payload) */
	OW_CMD_FPGA_PROG_UFM_WRITE_PAGES = 0x3E, /* Write N 16-byte UFM pages (N*16 bytes payload) */
	OW_CMD_FPGA_PROG_READ_STATUS     = 0x3F, /* Read 32-bit Status Register (returns 4 bytes) */
} OWProgFPGACommands;

typedef struct  {
	uint16_t id;
	uint8_t packet_type;
	uint8_t command;
	uint8_t addr;
	uint8_t reserved;
	uint16_t data_len;
	uint8_t* data;
	uint16_t crc;
} UartPacket;

typedef union {
    struct __attribute__((packed)) {
        float t1;
        float t2;
        float t3;
    } f;
    uint8_t bytes[12];
} ConsoleTemperatures;

_Bool process_if_command(UartPacket *uartResp, UartPacket *cmd);

#endif /* INC_COMMON_H_ */
