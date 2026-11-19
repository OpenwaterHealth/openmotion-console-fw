# UART Command Handling System Documentation

This document describes the UART command/packet handling system used by the firmware. It is intended to help future developers understand how commands are framed, parsed, dispatched, and responded to.

---

## High-Level Overview

The firmware communicates over UART using a framed packet protocol. Each packet represents a **command**, **response**, **data transfer**, or **error condition**. Incoming packets are parsed into a `UartPacket` structure and passed into the command handler (`process_if_command`). The handler inspects the packet type and command code, executes the requested operation, and populates a response packet.

At a high level:

1. Bytes are received over UART
2. A packet parser validates framing and CRC
3. The parsed packet is represented as a `UartPacket`
4. `process_if_command()` dispatches based on packet type and command
5. A response packet is constructed and transmitted

---

## Packet Format

All UART traffic uses the following packet structure:

```
| Start | ID | Type | Command | Addr | Reserved | Length | Payload | CRC16 | End |
```

### Field Definitions

| Field    | Size    | Description                                                     |
| -------- | ------- | --------------------------------------------------------------- |
| Start    | 1 byte  | Start-of-packet marker (`0xAA`)                                 |
| ID       | 2 bytes | Transaction identifier used to correlate requests and responses |
| Type     | 1 byte  | Packet type (command, response, data, error, etc.)              |
| Command  | 1 byte  | Command opcode                                                  |
| Addr     | 1 byte  | Address or sub-target (device, channel, register, etc.)         |
| Reserved | 1 byte  | Reserved for future use (must be zero)                          |
| Length   | 2 bytes | Length of payload in bytes                                      |
| Payload  | N bytes | Command-specific data (0–2048 bytes)                            |
| CRC16    | 2 bytes | CRC16 of header + payload                                       |
| End      | 1 byte  | End-of-packet marker (`0xDD`)                                   |

Maximum payload size is defined by `COMMAND_MAX_SIZE` (2048 bytes).

---

## Core Data Structures

### `UartPacket`

All parsed packets are represented using the `UartPacket` structure:

* `id`: Transaction ID
* `packet_type`: One of `OWPacketTypes`
* `command`: Command opcode
* `addr`: Address or selector byte
* `reserved`: Reserved, should be zero
* `data_len`: Payload length
* `data`: Pointer to payload buffer
* `crc`: CRC16 value from the packet

This structure is used for both incoming commands and outgoing responses.

---

## Packet Types (`OWPacketTypes`)

Packet types define *how* a packet should be interpreted:

| Type              | Meaning                            |
| ----------------- | ---------------------------------- |
| `OW_CMD`          | Incoming command from host         |
| `OW_RESP`         | Command response                   |
| `OW_DATA`         | Raw binary data transfer           |
| `OW_JSON`         | JSON-encoded data                  |
| `OW_ACK`          | Acknowledgement (success)          |
| `OW_NAK`          | Negative acknowledgement           |
| `OW_BAD_PARSE`    | Packet framing or format error     |
| `OW_BAD_CRC`      | CRC mismatch                       |
| `OW_UNKNOWN`      | Unknown command or type            |
| `OW_ERROR`        | General error response             |
| `OW_I2C_PASSTHRU` | I2C passthrough operation          |
| `OW_CONTROLLER`   | Motion/controller-specific command |

The `packet_type` is always evaluated first in the command handler.

---

## Command Namespaces

Commands are grouped into logical namespaces based on the packet type and command value.

### Global Commands (`OWGlobalCommands`)

These commands apply to the device as a whole and are typically sent using `OW_CMD`:

| Command             | Description                 |
| ------------------- | --------------------------- |
| `OW_CMD_PING`       | Connectivity check          |
| `OW_CMD_PONG`       | Ping response               |
| `OW_CMD_VERSION`    | Firmware version query      |
| `OW_CMD_ECHO`       | Echo payload back to sender |
| `OW_CMD_TOGGLE_LED` | Toggle status LED           |
| `OW_CMD_HWID`       | Hardware ID query           |
| `OW_CMD_SERIAL`     | Read/write console serial number (see below) |
| `OW_CMD_DEBUG_FLAGS`| Get/set runtime debug flags (see below) |
| `OW_CMD_DFU`        | Enter DFU/bootloader mode   |
| `OW_CMD_NOP`        | No operation                |
| `OW_CMD_RESET`      | Software reset              |

These commands usually generate an `OW_RESP` or `OW_ACK` packet.

### OW_CMD_SERIAL (0x07)

Read or write the console hardware serial number stored in the external EEPROM.
Selected by the `reserved` byte:

| reserved | Action            | Payload (in)        | Payload (out)                              |
|----------|-------------------|---------------------|--------------------------------------------|
| 0        | read              | —                   | ASCII serial; `data_len` = length (0 = unprogrammed) |
| 1        | write (guarded)   | ASCII serial bytes  | empty ACK; `OW_ERROR` if already programmed |
| 2        | write (force)     | ASCII serial bytes  | empty ACK                                  |

Serial is 1–24 uppercase-alphanumeric (`[A-Z0-9]`) characters. Stored as a
32-byte CRC16-checked record at EEPROM offset `0xD0`.

### OW_CMD_DEBUG_FLAGS (0x0C)

Get or set the runtime debug flags. The flags are a 32-bit bitmask held in RAM
only (they reset to `0` on every boot). Selected by `reserved` bit 0:

| reserved bit 0 | Action | Payload (in)            | Payload (out)              |
|----------------|--------|-------------------------|----------------------------|
| 0              | get    | —                       | current flags (`uint32` LE)|
| 1              | set    | new flags (`uint32` LE) | resulting flags (`uint32` LE)|

A `set` with a payload length other than 4 returns `OW_ERROR`. Either form
responds with the current flag value, so a client can confirm what took effect.

Defined flags (bit assignments mirror `openmotion-sensor-fw`; the console
currently implements only bit 0):

| Bit | Flag                  | Effect                                                              |
|-----|-----------------------|--------------------------------------------------------------------|
| 0   | `DEBUG_FLAG_USB_PRINTF` | Mirror firmware `printf()` output to the USB CDC host link        |

When `DEBUG_FLAG_USB_PRINTF` is set, `printf()` continues to go to the debug
UART and is additionally buffered and shipped to the host as `OW_DATA` packets
with command `OW_CMD_ECHO` and `id = 0`. The host must demultiplex these from
normal command responses (an `OW_DATA`/`OW_CMD_ECHO`/`id == 0` packet is log
text, never a command reply). Log output is buffered in a 2 KB ring and flushed
from the main loop; on overflow the oldest bytes are dropped and the gap is
marked with a `~`. Clearing the flag discards any buffered log data.

---

### Motion / Controller Commands (`MotionControllerCommands`)

These commands are used when `packet_type == OW_CONTROLLER` and target the motion controller or peripherals:

Examples include:

| Command              | Description                             |
| -------------------- | --------------------------------------- |
| `OW_CTRL_I2C_SCAN`   | Scan I2C bus for connected devices      |
| `OW_CTRL_SET_IND`    | Set indicator (LED) state               |
| `OW_CTRL_GET_IND`    | Get indicator (LED) state               |
| `OW_CTRL_SET_TRIG`   | Configure trigger parameters            |
| `OW_CTRL_GET_TRIG`   | Read current trigger configuration      |
| `OW_CTRL_START_TRIG` | Start/arm trigger operation             |
| `OW_CTRL_STOP_TRIG`  | Stop/disarm trigger operation           |
| `OW_CTRL_SET_FAN`    | Set MAX6663 fan PWM. `addr` must be 0 or 1; 1-byte payload = duty 0..100. |
| `OW_CTRL_GET_FAN`    | Read measured fan tach for one fan. `addr` selects fan 1..3 (FAN_TOP_GD2/GD3/GD4 = PE1/PE14/PE15); response is 2 bytes little-endian uint16 = RPM (assumes 2 pulses/rev), sampled over a ~100 ms window. 0 = no signal / unplugged. |
| `OW_CTRL_I2C_RD`     | Perform I2C read transaction            |
| `OW_CTRL_I2C_WR`     | Perform I2C write transaction           |
| `OW_CTRL_GET_FSYNC`  | Read frame sync (FSYNC) status          |
| `OW_CTRL_GET_LSYNC`  | Read line sync (LSYNC) status           |
| `OW_CTRL_TEC_DAC`    | Set TEC control DAC output              |
| `OW_CTRL_GET_TEMPS`  | Read temperature sensor values          |
| `OW_CTRL_TECADC`     | Read TEC-related ADC measurements       |
| `OW_CTRL_TEC_STATUS` | Read TEC controller status and faults   |
| `OW_CTRL_BOARDID`    | Read board identification information   |
| `OW_CTRL_PDUMON`     | Read power distribution monitoring data |
| `OW_CTRL_GET_PDC_BUFFER` | Drain up to N per-frame PDC samples from the SRAM ring buffer. Request payload: 1 byte = max_samples (1..64). Response: 2-byte LE drop counter + 1-byte sample count + N × 7-byte packed `{u32 frame_idx, u16 pdc_raw, u8 flags}`. `flags` bit 0 = `dark_slot`. |

Each command defines its own payload format and response payload.

---

## Error Codes (`OWErrorCodes`)

Error codes are returned in response payloads or error packets:

| Code                  | Meaning                          |
| --------------------- | -------------------------------- |
| `OW_CODE_SUCCESS`     | Operation completed successfully |
| `OW_CODE_IDENT_ERROR` | Invalid ID or addressing         |
| `OW_CODE_DATA_ERROR`  | Invalid or malformed payload     |
| `OW_CODE_ERROR`       | General failure                  |

---

## Command Handling Flow

The central entry point for command processing is:

```
_Bool process_if_command(UartPacket *uartResp, UartPacket *cmd);
```

### Responsibilities

* Validate the incoming packet type
* Dispatch based on `packet_type` and `command`
* Execute the requested operation
* Populate `uartResp` with:

  * Matching transaction ID
  * Appropriate response packet type
  * Response payload and length
  * Error codes if applicable

The function returns:

* `true` if the command was recognized and handled
* `false` if the command was unsupported or invalid

---

## Typical Command Lifecycle

1. **Receive**: Host sends `OW_CMD` or `OW_CONTROLLER` packet
2. **Parse**: Firmware validates framing and CRC
3. **Dispatch**: `process_if_command()` routes the command
4. **Execute**: Hardware or firmware operation is performed
5. **Respond**: Firmware sends `OW_RESP`, `OW_ACK`, `OW_DATA`, or `OW_ERROR`

The response packet:

* Reuses the incoming `id`
* Indicates success or failure via packet type and payload

---

## Adding a New Command

To add a new command:

1. Add a new enum value in the appropriate command enum
2. Define the payload format (document it!)
3. Add a case in `process_if_command()`
4. Validate `data_len` and payload contents
5. Populate the response packet
6. Return `true` on success

Always:

* Check payload length
* Avoid blocking operations
* Return meaningful error codes

---

## Design Notes & Best Practices

* **Transaction IDs** must be preserved across responses
* **CRC failures** should generate `OW_BAD_CRC`
* **Unknown commands** should generate `OW_UNKNOWN`
* Reserved fields must remain zero for forward compatibility
* Payloads should be endian-safe when possible

---

## Summary

This command handling system provides a structured, extensible UART protocol with:

* Clear packet framing
* Strong error detection
* Namespaced command sets
* Transaction-safe request/response handling

Understanding this flow is essential before modifying or extending the firmware command interface.
