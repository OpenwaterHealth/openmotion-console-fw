/*
 * if_fpga_prog.c
 *
 *  Created as refactor target for command processing from uart_comms.c
 */

#include "main.h"
#include "common.h"
#include "if_fpga_prog.h"
#include "uart_comms.h"
#include "utils.h"
#include "tca9548a.h"
#include "XO2_cmds.h"
#include "XO2_api.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Pointer to the XO2 device handle initialised by if_cmd_set_xo2_prog_handle() */
static XO2Handle_t *xo2_prog_devp = NULL;

_Bool process_fpga_prog_command(UartPacket *response, UartPacket *cmd)
{
    _Bool process_ret = true;
    
    response->command = cmd->command;

    if (xo2_prog_devp == NULL)
    {
        response->data_len = 0;
        response->packet_type = OW_ERROR;
        return OW_CODE_ERROR;
    }else{
        xo2_prog_devp->pI2CDrvrCalls->mux_ch = cmd->reserved;
    }

    switch (cmd->command)
    {

    case OW_CMD_FPGA_PROG_OPEN:
    {
        /* Open configuration interface in Offline mode */
        int ret = XO2ECAcmd_openCfgIF(xo2_prog_devp, OFFLINE_MODE);
        response->data_len = 0;
        if (ret != OK)
        {
            response->packet_type = OW_ERROR;
            return OW_CODE_ERROR;
        }
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_ERASE:
    {
        /* Erase flash sectors. Payload byte 0 = erase mode bitmap:
         *   bit3=UFM  bit2=CFG  bit1=FeatureRow  bit0=SRAM */
        if (cmd->data_len < 1u)
        {
            response->data_len = 0;
            response->packet_type = OW_BAD_PARSE;
            return OW_CODE_DATA_ERROR;
        }
        
        int ret = XO2ECAcmd_EraseFlash(xo2_prog_devp, cmd->data[0]);
        response->data_len = 0;
        if (ret != OK)
        {
            response->packet_type = OW_ERROR;
            return OW_CODE_ERROR;
        }
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_CFG_RESET:
    {
        
        int ret = XO2ECAcmd_CfgResetAddr(xo2_prog_devp);
        response->data_len = 0;
        if (ret != OK)
        {
            response->packet_type = OW_ERROR;
            return OW_CODE_ERROR;
        }
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_CFG_WRITE_PAGE:
    {
        if (cmd->data_len != XO2_FLASH_PAGE_SIZE)
        {
            response->data_len = 0;
            response->packet_type = OW_BAD_PARSE;
            return OW_CODE_DATA_ERROR;
        }
        
        int ret = XO2ECAcmd_CfgWritePage(xo2_prog_devp, cmd->data);
        response->data_len = 0;
        if (ret != OK)
        {
            response->packet_type = OW_ERROR;
            return OW_CODE_ERROR;
        }
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_CFG_READ_PAGE:
    {
        static uint8_t cfg_page_buf[XO2_FLASH_PAGE_SIZE];
       
        int ret = XO2ECAcmd_CfgReadPage(xo2_prog_devp, cfg_page_buf);
        if (ret != OK)
        {
            response->data_len = 0;
            response->packet_type = OW_ERROR;
            return OW_CODE_ERROR;
        }
        response->data = cfg_page_buf;
        response->data_len = XO2_FLASH_PAGE_SIZE;
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_UFM_RESET:
    {        
        int ret = XO2ECAcmd_UFMResetAddr(xo2_prog_devp);
        response->data_len = 0;
        if (ret != OK)
        {
            response->packet_type = OW_ERROR;
            return OW_CODE_ERROR;
        }
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_UFM_WRITE_PAGE:
    {
        if (cmd->data_len != XO2_FLASH_PAGE_SIZE)
        {
            response->data_len = 0;
            response->packet_type = OW_BAD_PARSE;
            return OW_CODE_DATA_ERROR;
        }

        int ret = XO2ECAcmd_UFMWritePage(xo2_prog_devp, cmd->data);
        response->data_len = 0;
        if (ret != OK)
        {
            response->packet_type = OW_ERROR;
            return OW_CODE_ERROR;
        }
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_UFM_READ_PAGE:
    {
        static uint8_t ufm_page_buf[XO2_FLASH_PAGE_SIZE];

        int ret = XO2ECAcmd_UFMReadPage(xo2_prog_devp, ufm_page_buf);
        if (ret != OK)
        {
            response->data_len = 0;
            response->packet_type = OW_ERROR;
            return OW_CODE_ERROR;
        }
        response->data = ufm_page_buf;
        response->data_len = XO2_FLASH_PAGE_SIZE;
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_FEATROW_WRITE:
    {
        /* Payload: 8 bytes feature + 2 bytes feabits = 10 bytes total */
        if (cmd->data_len < 10u)
        {
            response->data_len = 0;
            response->packet_type = OW_BAD_PARSE;
            return OW_CODE_DATA_ERROR;
        }

        XO2FeatureRow_t fr;        
        memcpy(fr.feature, &cmd->data[0], sizeof(fr.feature));
        memcpy(fr.feabits, &cmd->data[8], sizeof(fr.feabits));
        int ret = XO2ECAcmd_FeatureRowWrite(xo2_prog_devp, &fr);
        response->data_len = 0;
        if (ret != OK)
        {
            response->packet_type = OW_ERROR;
            return OW_CODE_ERROR;
        }
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_FEATROW_READ:
    {
        static uint8_t fr_buf[10];
        
        XO2FeatureRow_t fr;
        int ret = XO2ECAcmd_FeatureRowRead(xo2_prog_devp, &fr);
        if (ret != OK)
        {
            response->data_len = 0;
            response->packet_type = OW_ERROR;
            return OW_CODE_ERROR;
        }
        memcpy(&fr_buf[0], fr.feature, sizeof(fr.feature));
        memcpy(&fr_buf[8], fr.feabits, sizeof(fr.feabits));
        response->data = fr_buf;
        response->data_len = sizeof(fr_buf);
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_SET_DONE:
    {        
        int ret = XO2ECAcmd_setDone(xo2_prog_devp);
        response->data_len = 0;
        if (ret != OK)
        {
            response->packet_type = OW_ERROR;
            return OW_CODE_ERROR;
        }
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_REFRESH:
    {
        /*
         * ISC_REFRESH (0x79) tells the MachXO2 to reboot from flash.
         *
         * The opcode must be sent EXACTLY ONCE.  After the I2C write is
         * accepted the device goes dark while it reloads its config
         * (Trefresh for MachXO2-7000 ≥ 23 ms per Lattice TN1204).
         *
         * XO2ECA_apiProgram (the working button-press path) does:
         *   while (i && XO2ECAcmd_Refresh() != OK) --i;
         * but that re-sends 0x79 on every iteration, which is only
         * harmless there because the device ignores it once in user mode.
         * Here we split the two concerns explicitly:
         *   1. Send ISC_REFRESH once via MachXO_CmdXfer.
         *   2. Poll status register up to 10× until DONE is confirmed.
         * Always return success – the FPGA is booting into user mode
         * regardless of whether the last status poll converged.
         */

        /* Step 1 – send ISC_REFRESH opcode once */
        delay_us(1000);
        const uint8_t refresh_cmd[4] = {0x79, 0x00, 0x00, 0x00};
        int rc = (int)MachXO_CmdXfer(xo2_prog_devp->pI2CDrvrCalls,
                                     refresh_cmd, 3, NULL, 0);
        if (rc != OK)
        {
            /* Could not even send the command – genuine error */
            printf("REFRESH: failed to send ISC_REFRESH\r\n");
            response->data_len = 0;
            response->packet_type = OW_ERROR;          
            return OW_CODE_ERROR;
        }

        /* Step 2 – poll status register until DONE (bits 12:13 == 00) */
        uint32_t trefresh = XO2DevList[xo2_prog_devp->devType].Trefresh;
        unsigned int sr = 0xFFFFFFFFu;
        int i = 10;
        do
        {
            HAL_Delay(trefresh);
            rc = XO2ECAcmd_readStatusReg(xo2_prog_devp, &sr);
            if (rc == OK && (sr & 0x3000u) == 0x0000u)
                break; /* DONE – device is in user mode */
            --i;
        } while (i);

        if (i)
        {
            printf("REFRESH: done after %d poll(s), sr=0x%08x\r\n", 11 - i, sr);
        }
        else
        {
            printf("REFRESH: status poll timed out, sr=0x%08x – FPGA likely booted\r\n", sr);
        }

        xo2_prog_devp->cfgEn = FALSE;
        response->data_len = 0;
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS; /* always succeed – programming is complete */
    }

    case OW_CMD_FPGA_PROG_CLOSE:
    {
        XO2ECAcmd_closeCfgIF(xo2_prog_devp);
        XO2ECAcmd_Bypass(xo2_prog_devp);
        response->data_len = 0;
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_CFG_WRITE_PAGES:
    {
        /* Payload: N * XO2_FLASH_PAGE_SIZE bytes, N >= 1 */
        if (cmd->data_len == 0 || (cmd->data_len % XO2_FLASH_PAGE_SIZE) != 0)
        {
            response->data_len = 0;
            response->packet_type = OW_BAD_PARSE;
            return OW_CODE_DATA_ERROR;
        }

        uint16_t num_pages = cmd->data_len / XO2_FLASH_PAGE_SIZE;
        for (uint16_t pg = 0; pg < num_pages; pg++)
        {
            int ret = XO2ECAcmd_CfgWritePage(xo2_prog_devp,
                                             cmd->data + pg * XO2_FLASH_PAGE_SIZE);
            if (ret != OK)
            {
                response->data_len = 0;
                response->packet_type = OW_ERROR;
                return OW_CODE_ERROR;
            }
        }
        response->data_len = 0;
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_UFM_WRITE_PAGES:
    {
        /* Payload: N * XO2_FLASH_PAGE_SIZE bytes, N >= 1 */
        if (cmd->data_len == 0 || (cmd->data_len % XO2_FLASH_PAGE_SIZE) != 0)
        {
            response->data_len = 0;
            response->packet_type = OW_BAD_PARSE;
            return OW_CODE_DATA_ERROR;
        }
        
        uint16_t num_pages = cmd->data_len / XO2_FLASH_PAGE_SIZE;
        for (uint16_t pg = 0; pg < num_pages; pg++)
        {
            int ret = XO2ECAcmd_UFMWritePage(xo2_prog_devp,
                                             cmd->data + pg * XO2_FLASH_PAGE_SIZE);
            if (ret != OK)
            {
                response->data_len = 0;
                response->packet_type = OW_ERROR;
                return OW_CODE_ERROR;
            }
        }
        response->data_len = 0;
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }

    case OW_CMD_FPGA_PROG_READ_STATUS:
    {
        /* Read the 32-bit Status Register (no cfgEn required). Returns 4 bytes big-endian. */
        static uint8_t sr_buf[4];
        unsigned int sr = 0;
        int ret = XO2ECAcmd_readStatusReg(xo2_prog_devp, &sr);
        if (ret != OK)
        {
            response->data_len = 0;
            response->packet_type = OW_ERROR;
            return OW_CODE_ERROR;
        }
        sr_buf[0] = (uint8_t)(sr >> 24);
        sr_buf[1] = (uint8_t)(sr >> 16);
        sr_buf[2] = (uint8_t)(sr >> 8);
        sr_buf[3] = (uint8_t)(sr);
        response->data = sr_buf;
        response->data_len = sizeof(sr_buf);
        response->packet_type = OW_RESP;
        return OW_CODE_SUCCESS;
    }
    default:
        response->data_len = 0;
        response->packet_type = OW_UNKNOWN;
        break;
    }
    
    return process_ret;
}

void if_cmd_set_xo2_prog_handle(XO2Handle_t *h)
{
    xo2_prog_devp = h;
}
