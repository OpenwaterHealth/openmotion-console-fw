/* if_commands.h - command processing API */
#ifndef INC_IF_FPGA_PROG_COMMANDS_H_
#define INC_IF_FPGA_PROG_COMMANDS_H_

#include "common.h"
#include "XO2_dev.h"

void if_cmd_set_xo2_prog_handle(XO2Handle_t *h);
#if defined(OW_FACTORY_PROGRAMMING)
/* Factory builds only; see if_fpga_prog.c. */
_Bool process_fpga_prog_command(UartPacket *uartResp, UartPacket *cmd);
#endif

extern ConsoleTemperatures consoleTemps;

#endif /* INC_IF_FPGA_PROG_COMMANDS_H_ */
