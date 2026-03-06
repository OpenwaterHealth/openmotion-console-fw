#include <stdio.h>

#include "XO2_cmds.h"
#include "tca9548a.h"


volatile uint8_t txComplete = 0;
volatile uint8_t rxComplete = 0;
volatile uint8_t i2cError = 0;

static inline uint16_t MachXO_Addr8(const MachXO_Handle_t *h)
{
    /* HAL expects 8-bit address = 7-bit << 1 */
    return (uint16_t)(h->addr7 << 1);
}

static int xi2c_write_bytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *data, uint16_t length) {
    return HAL_I2C_Master_Transmit(hi2c, DevAddress, data, length, HAL_MAX_DELAY);
}

static int xi2c_write_and_read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *wbuf, uint16_t wlen, uint8_t *rbuf, uint16_t rlen) {
    txComplete = 0;
    rxComplete = 0;
    i2cError = 0;

    if (HAL_I2C_Master_Seq_Transmit_IT(hi2c, DevAddress, wbuf, wlen, I2C_FIRST_FRAME) != HAL_OK)
        return -1;

    // Wait for the transmission to complete
    while (!txComplete && !i2cError) {}

    if (i2cError)
    {
        return HAL_ERROR;
    }


    if (HAL_I2C_Master_Seq_Receive_IT(hi2c, DevAddress, rbuf, rlen, I2C_LAST_FRAME) != HAL_OK)
        return -1;

    // Wait for the reception to complete
    while (!rxComplete && !i2cError) {}

    if (i2cError)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}


uint32_t MachXO_CmdXfer(MachXO_Handle_t *h,
                        const uint8_t *wbuf, int wcnt,
                        uint8_t *rbuf, int rcnt)
{
    if (!h || !h->hi2c) return ERROR;
    if ((wcnt > 0) && (wbuf == NULL)) return ERROR;
    if ((rcnt > 0) && (rbuf == NULL)) return ERROR;
    if (wcnt < 0 || rcnt < 0) return ERROR;

    const uint16_t addr8 = MachXO_Addr8(h);
	// printf("I2C Write MUX: %d CH: %d ADDR: 0x%02X \r\n", h->mux_idx, h->mux_ch, addr8);       
	TCA9548A_SelectChannel(h->mux_idx, h->mux_ch);
    if (rcnt > 0) {
    	if(xi2c_write_and_read(h->hi2c, addr8, (uint8_t*)wbuf, (uint16_t)wcnt, rbuf, rcnt) != HAL_OK) {
    		return ERROR;
    	}

    } else {
    	if(xi2c_write_bytes(h->hi2c, addr8, (uint8_t*)wbuf, (uint16_t)wcnt) != HAL_OK){
    		return ERROR;
    	}
        
    }
        
    return OK;
}


/**
 * Read the 4 byte Device ID from the XO2 Configuration logic block.
 * This function assembles the command sequence that allows reading the XO2 Device ID
 * from the configuration logic.  The command is first written to the XO2, then a read
 * of 4 bytes is perfromed to return the value.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param pVal pointer to the 32 bit integer to return the ID value in
 * @return OK if successful, ERROR if failed to read
 * 
 */
int XO2ECAcmd_readDevID(XO2Handle_t *pXO2, unsigned int *pVal) 
{
	unsigned char cmd[4];
	unsigned char data[4];
	int status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_readDevID()\r\n");
#endif

	cmd[0] = 0xE0;  // Read Device ID opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2
	
    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, data, 4);

#ifdef DEBUG_ECA
	printf("\tstatus=%d  data=%x %x %x %x\r\n", status, data[0], data[1], data[2], data[3]);
#endif
	if (status == OK)
	{
		*pVal = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}



/**
 * Read the 4 byte USERCODE from the XO2 Configuration logic block.
 * This function assembles the command sequence that allows reading the XO2 USERCODE
 * value from the configuration Flash sector.  The command is first written to the XO2, then a read
 * of 4 bytes is perfromed to return the value.
 * 
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param pVal pointer to the 32 bit integer to return the USERCODE value in
 * @return OK if successful, ERROR if failed to read
 * @note 
 * 
 */
int XO2ECAcmd_readUserCode(XO2Handle_t *pXO2, unsigned int *pVal) 
{
	unsigned char cmd[4];
	unsigned char data[4];
	int status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_readUserCode()\r\n");
#endif

	cmd[0] = 0xC0;  // Read USERCODE opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2
	
    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, data, 4);
	
#ifdef DEBUG_ECA
	printf("\tstatus=%d  data=%x %x %x %x\r\n", status, data[0], data[1], data[2], data[3]);
#endif
	if (status == OK)
	{
		*pVal = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}




/**
 * Set the 4 byte USERCODE in the XO2 Configuration logic block.
 * This function assembles the command sequence that allows programming the XO2 USERCODE
 * value.  The command is written to the XO2, along with 4 bytes to program into the USERCODE
 * area of flash.
 *  
 * @param pXO2 pointer to the XO2 device to access
 * @param val new USERCODE value
 * @return OK if successful, ERROR if failed to write
 * 
 * @note This command is only useful if the USERCODE contents is previously all 0's.
 * The USERCODE is cleared when the Cfg sector is erased.  So there is no way to 
 * individually clear just the USERCODE and reprogram with a new value using this command.
 * Its usefulness is questionable, seeing as the USERCODE is set when programming the
 * Cfg sector anyway.
 */
int XO2ECAcmd_setUserCode(XO2Handle_t *pXO2, unsigned int val) 
{
	unsigned char cmd[8];
	int status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_setUserCode()\r\n");
#endif

	if (pXO2->cfgEn == FALSE)
	{
		return(ERR_XO2_NOT_IN_CFG_MODE);
	}

	cmd[0] = 0xC2;  // program USERCODE opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

	cmd[4] = (unsigned char)(val>>24);
	cmd[5] = (unsigned char)(val>>16);
	cmd[6] = (unsigned char)(val>>8);
	cmd[7] = (unsigned char)(val);

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 8, NULL, 0);

#ifdef DEBUG_ECA
	printf("\tstatus=%d\r\n", status);
#endif
	if (status == OK)
	{
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}


/**
 * Read the 8 byte (64 bit) TraceID from the XO2 Feature Row.
 * This function assembles the command sequence that allows reading the XO2 TraceID
 * value from the Feature Row Flash sector.  The command is first written to the XO2, then a read
 * of 8 bytes is perfromed to return the value.
 * The TraceID is set in the Global Settings of Spreadsheet view.
 * The first byte read back (pVal[0]) can be set by the user in Spreadsheet view.
 * The remaining 7 bytes are unique for each silicon die.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param pVal pointer to the 8 byte array to return the TraceID value in
 * @return OK if successful, ERROR if failed to read
 * 
 */
int XO2ECAcmd_readTraceID(XO2Handle_t *pXO2, unsigned char *pVal) 
{
	unsigned char cmd[4];
	unsigned char data[8];
	int status;
	int i;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_readTraceID()\r\n");
#endif

	cmd[0] = 0x19;  // Read TraceID opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2
	
    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, data, 8);
	
#ifdef DEBUG_ECA
	printf("\tstatus=%d  data=", status);
	for (i = 0; i < 7; i++)
		printf("  %x", data[i]);
	printf("  %x\r\n", data[7]);
#endif
	if (status == OK)
	{
		for (i = 0; i < 8; i++)
			pVal[i] = data[i];
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}



/**
 * Enable access to Configuration Logic in Transparent mode or Offline mode.
 * This function issues one of the Enable Configuration Interface commands depending on
 * the value of mode.  Transparent mode allows the XO2 to continue operating in user mode
 * while access to the config logic is performed.  Offline mode halts all user logic, and
 * tri-states I/Os, while access is occurring.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param mode specify TRANSPARENT_MODE or OFFLINE_MODE 
 * @return OK if successful, ERROR if failed to read
 * 
 */
int XO2ECAcmd_openCfgIF(XO2Handle_t *pXO2, XO2CfgMode_t mode) 
{
	unsigned char cmd[4];
	int status;


#ifdef DEBUG_ECA
	if (mode == TRANSPARENT_MODE)
		printf("XO2ECAcmd_openCfgIF(Transparent_MODE)\r\n");
	else
		printf("XO2ECAcmd_openCfgIF(Offline_MODE)\r\n");	
#endif

	// if (mode == TRANSPARENT_MODE)
	cmd[0] = 0x74;  // Enable Config Interface in Transparent Mode opcode
	cmd[1] = 0x08;  // arg0
	cmd[2] = 0x00;  // arg1
//	cmd[3] = 0x00;  // arg2 - not used


	if (mode == OFFLINE_MODE)
	{
		cmd[0] = 0xC6;  // Enable Config  Interface in Offline Mode opcode		
	}

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 3, NULL, 0);
		
	// Wait till not busy - we have entered Config mode
	if (status == OK)
		status = XO2ECAcmd_waitStatusBusy(pXO2);
			
#ifdef DEBUG_ECA
		printf("\tstatus=%d\r\n", status);
#endif
	if (status == OK)
	{
		pXO2->cfgEn = TRUE;
		return(OK);
	}
	else
	{
		pXO2->cfgEn = FALSE;
		return(ERROR);
	}
}


/**
 * Disable access to Configuration Logic Interface.
 * This function issues the Disable Configuration Interface command and 
 * registers that the interface is no longer available for certain commands
 * that require it to be enabled.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @return OK if successful, ERROR if failed to read
 * 
 */
int XO2ECAcmd_closeCfgIF(XO2Handle_t *pXO2) 
{
	unsigned char cmd[4];
	int status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_closeCfgIF()\r\n");
#endif

	cmd[0] = 0x26;  // Disable Config  Interface opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
//	cmd[3] = 0x00;  // arg2  not used now

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 3, NULL, 0);

#ifdef DEBUG_ECA
	printf("\tstatus=%d\r\n", status);
#endif
	if (status == OK)
	{
		pXO2->cfgEn = FALSE;
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}


/**
 * Issue the Refresh command that updates the SRAM from Flash and boots the XO2.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @return OK if successful, ERROR code if failed to write
 * 
 */
int XO2ECAcmd_Refresh(XO2Handle_t *pXO2) 
{
	unsigned char cmd[4];
	int status;
	unsigned int sr;
	
#ifdef DEBUG_ECA
	printf("XO2ECAcmd_Refresh()\r\n");
#endif


	cmd[0] = 0x79;  // Refresh opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
//	cmd[3] = 0x00;  // arg2 - not used now

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, NULL, 0);

	if(status != OK) return status;

	// Must wait atleast Trefresh amount of time before allowing any other access to
	// the XO2 device.

	pXO2->pfmSecDelay(XO2DevList[pXO2->devType].Trefresh );
	sr = 0xFFFFFFFFu;  /* sentinel: treat unread register as error */
	status = XO2ECAcmd_readStatusReg(pXO2, &sr);

	if (status != OK)
		return(ERROR);

#ifdef DEBUG_ECA
	printf("\tstatus=%d   sr=%x\r\n", status, sr);
#endif
		
	/* Verify that only DONE bit is definitely set and not FAIL or BUSY or ISC_ENABLED */
	if ((sr & 0x3000) == 0x0000)
	{
		pXO2->cfgEn = FALSE;
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}


/**
 * Issue the Done command that updates the Program DONE bit.
 * Typically used after programming the Cfg Flash and before 
 * closing access to the configuration interface.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @return OK if successful, ERROR code if failed to write
 * 
 */
int XO2ECAcmd_setDone(XO2Handle_t *pXO2) 
{
	unsigned char cmd[4];
	int status;
	unsigned int sr;
	
#ifdef DEBUG_ECA
	printf("XO2ECAcmd_setDone()\r\n");
#endif

	if (pXO2->cfgEn == FALSE)
	{
		return(ERR_XO2_NOT_IN_CFG_MODE);
	}

	cmd[0] = 0x5E;  // Program DONE bit opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, NULL, 0);

// TODO: This delay time may be excessive

	if (status == OK)
	{
		// Wait 10 msec for Done
		pXO2->pfmSecDelay(10);
	}
	else 
	{
		return(ERROR);
	}
	
	
	
	if (XO2ECAcmd_readStatusReg(pXO2, &sr) != OK)
		return(ERROR);
		
	// Verify that DONE bit is definitely set and not FAIL or BUSY
	if ((sr & 0x3100) == 0x0100)
	{
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}





/**
 * Read the 4 byte Status Register from the XO2 Configuration logic block.
 * This function assembles the command sequence that allows reading the XO2 Status Register.
 * The command is first written to the XO2, then a read of 4 bytes is perfromed to return the value.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param pVal pointer to the 32 bit integer to return the Status Register value in.
 * @return OK if successful, ERROR if failed to read.
 * 
 */
int XO2ECAcmd_readStatusReg(XO2Handle_t *pXO2, unsigned int *pVal) 
{
	unsigned char cmd[4];
	unsigned char data[4];
	int status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_readStatusReg()\r\n");
#endif

	cmd[0] = 0x3C;  // Read Status Register opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, data, 4);

#ifdef DEBUG_ECA
	printf("\tstatus=%d  data=%x %x %x %x\r\n", status, data[0], data[1], data[2], data[3]);
#endif
	if (status == OK)
	{
		*pVal = (data[0]<<24) | (data[1]<<16) | (data[2]<<8) | data[3];
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}



/**
 * Wait for the Status register to report no longer busy.
 * Read the 4 byte Status Register from the XO2 Configuration logic block and check bit 12
 * to see if BUSY.  Also check bit 13 for FAIL indication.  Return error if an error
 * condition is detected.  Also return if exceed polling loop timeout.
 * This function assembles the command sequence that allows reading the XO2 Status Register.
 * The command is first written to the XO2, then a read of 4 bytes is perfromed to return the value.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @return OK if no longer Busy and can proceed, ERROR if failed to read.
 * 
 */
int XO2ECAcmd_waitStatusBusy(XO2Handle_t *pXO2) 
{
	unsigned char cmd[4];
	unsigned char data[4];
	int status;
	int loop;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_waitStatusBusy()\r\n");
#endif

	cmd[0] = 0x3C;  // Read Status Register opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

	loop = XO2ECA_CMD_LOOP_TIMEOUT;
	do
	{
		status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, data, 4);
		
		if (status != OK)
			return(ERROR);
			
		if (data[2] & 0x20)  // FAIL bit set
			return(ERROR);
			
		if (data[2] & 0x10)
		{
			// Still busy so wait another msec and loop again, if not timed out
			--loop;
			pXO2->pfmSecDelay(1);   // delay 1 msec
		}			
		
	} while(loop && (data[2] & 0x10)); 
		
	
	if (loop)
		return(OK);
	else
		return(ERROR);   // timed out waiting for BUSY to clear
}



/**
 * Read the Busy Flag bit from the XO2 Configuration logic block.
 * This function assembles the command sequence that allows reading the XO2 Busy Flag.
 * The command is first written to the XO2, then a read of 1 byte is perfromed to return the value.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param pVal pointer to the 8 bit integer to return the Busy Flag bit value in.
 * @return OK if successful, ERROR if failed to read.
 * 
 */
int XO2ECAcmd_readBusyFlag(XO2Handle_t *pXO2, unsigned char *pVal) 
{
	unsigned char cmd[4];
	unsigned char data;
	int status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_readBusyFlag()\r\n");
#endif

	cmd[0] = 0xF0;  // Read Status Register opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2
	
    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, &data, 1);

#ifdef DEBUG_ECA
	printf("\tstatus=%d  data=%x\r\n", status, data);
#endif
	if (status == OK)
	{
		*pVal = data;
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}




/**
 * Wait for the Busy Flag to be cleared.
 * Read the 1 byte from the Busy Flag resgister and check if not 0
 * to see if still BUSY.  Return error if an error
 * condition is detected.  Also return if exceed polling loop timeout.
 * This function assembles the command sequence that allows reading the XO2 Busy Flag Register.
 * The command is first written to the XO2, then a read of 1 byte is perfromed to obtain the value.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @return OK if no longer Busy and can proceed, ERROR if failed to read.
 * 
 */
int XO2ECAcmd_waitBusyFlag(XO2Handle_t *pXO2) 
{
	unsigned char cmd[4];
	unsigned char data[4];
	int status;
	int loop;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_waitBusyFlag()\r\n");
#endif

	cmd[0] = 0xF0;  // Check Busy Flag Register opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

	loop = XO2ECA_CMD_LOOP_TIMEOUT;
	do
	{		    	
		status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, data, 1);

		if (status != OK)
			return(ERROR);
		
		if (data[0])
		{
			// Still busy so wait another msec
			--loop;
			pXO2->pfmSecDelay(1);   // delay 1 msec
		}
		
	} while(loop && data[0]); 
	
	if (loop)
		return(OK);
	else
		return(ERROR);   // timed out waiting for BUSY to clear
}


/**
 * Send the Bypass command.
 * This function assembles the command sequence that allows writing the XO2 Bypass command.
 * This command is typically called after closing the Configuration Interface.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @return OK if successful, ERROR if failed to read.
 * 
 */
int XO2ECAcmd_Bypass(XO2Handle_t *pXO2) 
{
	unsigned char cmd[4];
	int status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_Bypass()\r\n");
#endif

	cmd[0] = 0xFF;  // Bypass opcode - supposedly does not have arguements, just command byte
//	cmd[1] = 0x00;  // arg0
//	cmd[2] = 0x00;  // arg1
//	cmd[3] = 0x00;  // arg2

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 1, NULL, 0);

#ifdef DEBUG_ECA
	printf("\tstatus=%d\r\n", status);
#endif
	if (status == OK)
	{
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}


/**
 * Set the current Page Address in either Cfg Flash or UFM.
 * The specific page is set for the next read/write operation.
 * This is probably only useful for the UFM since skipping around in the configuration
 * sector is very unlikely.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param mode which address to update: CFG_SECTOR or UFM_SECTOR
 * @param pageNum the page number to set address pointer to 
 * @return OK if successful, ERROR if failed to set address.
 * 
 */
int XO2ECAcmd_SetPage(XO2Handle_t *pXO2, XO2SectorMode_t mode, unsigned int pageNum) 
{
	unsigned char cmd[8];
	int status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_SetPage()\r\n");
#endif

	if (pXO2->cfgEn == FALSE)
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_NOT_IN_CFG_MODE\r\n");
#endif
		return(ERR_XO2_NOT_IN_CFG_MODE);
	}

	if ((mode == UFM_SECTOR) && (pageNum > XO2DevList[pXO2->devType].UFMpages))
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_EXCEEDS_UFM_SIZE\r\n");
#endif
		return(ERR_XO2_EXCEEDS_UFM_SIZE );
	}

	if ((mode == CFG_SECTOR) && (pageNum > XO2DevList[pXO2->devType].Cfgpages))
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_EXCEEDS_CFG_SIZE\r\n");
#endif
		return(ERR_XO2_EXCEEDS_CFG_SIZE );
	}


	cmd[0] = 0xB4;  // opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

	if (mode == CFG_SECTOR)
		cmd[4] = 0x00;  // page[0] = 0=Config, 1=UFM
	else
		cmd[4] = 0x40;  // page[0] = 0=Config, 1=UFM
	cmd[5] = 0x00;  // page[1]
	cmd[6] = (unsigned char)(pageNum>>8);  // page[2] = page number MSB
	cmd[7] = (unsigned char)pageNum;       // page[3] = page number LSB

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 8, NULL, 0);

#ifdef DEBUG_ECA
	printf("\tstatus=%d\r\n", status);
#endif
	if (status == OK)
	{
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}


/**
 * Erase any/all entire sectors of the XO2 Flash memory.
 * Erase sectors based on the bitmap of parameter mode passed in.
 * <ul>
 * <li> 8 = UFM
 * <li> 4 = CFG
 * <li> 2 = Feature Row
 * <li> 1 = SRAM
 * </ul>
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param mode bit map of what sector contents to erase
 * @return OK if successful, ERROR if failed.
 * 
 * @note Erases bits to a value of 0.  Any bit that is a 0 can then be programmed to a 1.
 * 
 */
int XO2ECAcmd_EraseFlash(XO2Handle_t *pXO2, unsigned char mode) 
{
	unsigned char cmd[4];
	int status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_EraseFlash()\r\n");
#endif

	if (pXO2->cfgEn == FALSE)
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_NOT_IN_CFG_MODE\r\n");
#endif
		return(ERR_XO2_NOT_IN_CFG_MODE);
	}

	mode = mode & 0x0f; 

	cmd[0] = 0x0E;  // Erase Flash opcode
	cmd[1] = mode;  // arg0 = which sectors to clear
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, NULL, 0);

	if (status == OK)
	{
		// Must wait an amount of time, based on device size, for largest flash sector to erase.
		if (mode & XO2ECA_CMD_ERASE_CFG)
			pXO2->pfmSecDelay(XO2DevList[pXO2->devType].CfgErase);  // longest
		else if (mode & XO2ECA_CMD_ERASE_UFM)
			pXO2->pfmSecDelay(XO2DevList[pXO2->devType].UFMErase);  // medium
		else		
			pXO2->pfmSecDelay(50);	// SRAM & Feature Row = shortest

		status = XO2ECAcmd_waitStatusBusy(pXO2);
	}
	
	
#ifdef DEBUG_ECA
	printf("\tstatus=%d\r\n", status);
#endif
	if (status == OK)
	{
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}


//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================
//                          C O N F I G     F L A S H      C O M M A N D S
//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================
/**
 * Reset the Address Regsiter to point to the first Config Flash page (sector 0, page 0).
 * @param pXO2 pointer to the XO2 device to operate on
 *
 * @return OK is address reset.  Error if failed.
 * 
 */
int XO2ECAcmd_CfgResetAddr(XO2Handle_t *pXO2) 
{
	unsigned char cmd[4];
	int status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_CfgResetAddr()\r\n");
#endif

	if (pXO2->cfgEn == FALSE)
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_NOT_IN_CFG_MODE\r\n");
#endif
		return(ERR_XO2_NOT_IN_CFG_MODE);
	}


	cmd[0] = 0x46;  // Reset CFG Address pointer to page 0 opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, NULL, 0);

#ifdef DEBUG_ECA
	printf("\tstatus=%d\r\n", status);
#endif
	if (status == OK)
	{
		return(OK);
	}
	else
	{
		return(ERROR);
	}

}


/**
 * Read the next page (16 bytes) from the Config Flash memory.
 * Page address can be set using SetAddress command.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param pBuf pointer to the 16 byte array to return the Config page bytes in.
 * @return OK if successful, ERROR if failed to read.
 * 
 * @note There is no advantage to reading multiple pages since the interface
 * is most likely so slow.  The I2C only runs at 100kHz.
 * 
 * @note The number of pages read is not comparted against the total pages in the
 * device.  Reading too far may have unexpected results.
 */
int XO2ECAcmd_CfgReadPage(XO2Handle_t *pXO2, unsigned char *pBuf) 
{
	unsigned char cmd[4];
	unsigned char data[XO2_FLASH_PAGE_SIZE];
	int status;
	int i;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_CfgReadPage()\r\n");
#endif

	if (pXO2->cfgEn == FALSE)
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_NOT_IN_CFG_MODE\r\n");
#endif
		return(ERR_XO2_NOT_IN_CFG_MODE);
	}


	cmd[0] = 0x73;  // Read UFM opcode
	cmd[1] = 0x00;  // arg0 = pad 4 bytes per page (to get around a readback problem)
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x01;  // arg2 = 1 page

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, data, XO2_FLASH_PAGE_SIZE);

#ifdef DEBUG_ECA
	printf("\tstatus=%d  data=", status);
	for (i = 0; i < XO2_FLASH_PAGE_SIZE-1; i++)
		printf("  %x", data[i]);
	printf("  %x\r\n", data[XO2_FLASH_PAGE_SIZE-1]);
#endif
	if (status == OK)
	{
		for (i = 0; i < XO2_FLASH_PAGE_SIZE; i++)
			pBuf[i] = data[i];
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}


/**
 * Write a page (16 bytes) into the current UFM memory page.
 * Page address can be set using SetAddress command.
 * Page address advances to next page after programming is completed.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param pBuf pointer to the 16 byte array to write into the UFM page.
 * @return OK if successful, ERROR if failed to read.
 * 
 * @note Programming must be done on a page basis.  Pages must be erased to 0's first.
 * @see XO2ECAcmd_UFMErase
 * 
 * @note The number of pages written is not compared against the total pages in the
 * device.  Writing too far may have unexpected results.
 */
int XO2ECAcmd_CfgWritePage(XO2Handle_t *pXO2, const unsigned char *pBuf) 
{
	unsigned char cmd[4 + 16];
	int status;
	int i;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_CfgWritePage()\r\n");
#endif

	if (pXO2->cfgEn == FALSE)
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_NOT_IN_CFG_MODE\r\n");
#endif
		return(ERR_XO2_NOT_IN_CFG_MODE);
	}


	cmd[0] = 0x70;  // Program Cfg opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x01;  // arg2

	for (i = 0; i < 16; i++)
		cmd[4 + i] = pBuf[i];

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4 + 16, NULL, 0);

	if (status == OK)
	{
		// Must wait 200 usec for a page to program.  This is a constant for all devices (see XO2 datasheet)
		pXO2->pfuSecDelay(200);
		status = XO2ECAcmd_waitStatusBusy(pXO2);
	}
	
#ifdef DEBUG_ECA
	printf("\tstatus=%d\r\n", status);
#endif
	if (status == OK)
	{
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}

/**
 * Erase the entire sector of the Configuration Flash memory.
 * This is a convience function to erase all Config contents to 0.  You can not erase on a page basis.
 * The entire sector is cleared.
 * The erase takes up to a few seconds.  The time to wait is device specific.
 * It is important that the correct device is selected in the pXO2 structure.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @return OK if successful, ERROR if failed.
 * 
 * @note Erases bits to a value of 0.  Any bit that is a 0 can then be programmed to a 1.
 * 
 */
int XO2ECAcmd_CfgErase(XO2Handle_t *pXO2) 
{

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_CfgErase()\r\n");
#endif

	return(XO2ECAcmd_EraseFlash(pXO2, XO2ECA_CMD_ERASE_CFG));
}

//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================
//                            U F M      C O M M A N D S
//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================
/**
 * Reset the Address Regsiter to point to the first UFM page (sector 1, page 0).
 * @param pXO2 pointer to the XO2 device to operate on
 *
 * @return OK if successful. ERROR if failed.
 * 
 */
int XO2ECAcmd_UFMResetAddr(XO2Handle_t *pXO2) 
{
	unsigned char cmd[4];
	int status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_UFMResetAddr()\r\n");
#endif

	if (pXO2->cfgEn == FALSE)
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_NOT_IN_CFG_MODE\r\n");
#endif
		return(ERR_XO2_NOT_IN_CFG_MODE);
	}

	if (pXO2->devType == MachXO2_256)
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_NO_UFM\r\n");
#endif
		return(ERR_XO2_NO_UFM);
	}


	cmd[0] = 0x47;  // Reset UFM Address pointer to page 0 opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, NULL, 0);

#ifdef DEBUG_ECA
	printf("\tstatus=%d\r\n", status);
#endif
	if (status == OK)
	{
		return(OK);
	}
	else
	{
		return(ERROR);
	}

}


/**
 * Read the next page (16 bytes) from the UFM memory.
 * Page address can be set using SetAddress command.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param pBuf pointer to the 16 byte array to return the UFM page bytes in.
 * @return OK if successful, ERROR if failed to read.
 * 
 * @note There is no advantage to reading multiple pages since the interface
 * is most likely so slow.  The I2C only runs at 100kHz.
 * 
 */
int XO2ECAcmd_UFMReadPage(XO2Handle_t *pXO2, unsigned char *pBuf) 
{
	unsigned char cmd[4];
	unsigned char data[16];
	int status;
	int i;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_UFMReadPage()\r\n");
#endif

	if (pXO2->cfgEn == FALSE)
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_NOT_IN_CFG_MODE\r\n");
#endif
		return(ERR_XO2_NOT_IN_CFG_MODE);
	}

	if (pXO2->devType == MachXO2_256)
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_NO_UFM\r\n");
#endif
		return(ERR_XO2_NO_UFM);
	}


	cmd[0] = 0xCA;  // Read UFM opcode
	cmd[1] = 0x00;  // arg0 = pad 4 bytes per page (to get around a readback problem)
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x01;  // arg2 = 1 page

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, data, 16);

#ifdef DEBUG_ECA
	printf("\tstatus=%d  data=", status);
	for (i = 0; i < 15; i++)
		printf("  %x", data[i]);
	printf("  %x\r\n", data[15]);
#endif
	if (status == OK)
	{
		for (i = 0; i < 16; i++)
			pBuf[i] = data[i];
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}

/**
 * Write a page (16 bytes) into the current UFM memory page.
 * Page address can be set using SetAddress command.
 * Page address advances to next page after programming is completed.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param pBuf pointer to the 16 byte array to write into the UFM page.
 * @return OK if successful, ERROR if failed to read.
 * 
 * @note Programming must be done on a page basis.  Pages must be erased to 0's first.
 * @see XO2ECAcmd_UFMErase
 * 
 */
int XO2ECAcmd_UFMWritePage(XO2Handle_t *pXO2, const unsigned char *pBuf) 
{
	unsigned char cmd[4 + 16];
	int status;
	int i;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_UFMWritePage()_1\r\n");
#endif

	if (pXO2->cfgEn == FALSE)
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_NOT_IN_CFG_MODE\r\n");
#endif
		return(ERR_XO2_NOT_IN_CFG_MODE);
	}

	if (pXO2->devType == MachXO2_256)
	{
#ifdef DEBUG_ECA
		printf("\tERR_XO2_NO_UFM\n");
#endif
		return(ERR_XO2_NO_UFM);
	}

	cmd[0] = 0xC9;  // Program UFM opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x01;  // arg2

	for (i = 0; i < 16; i++)
		cmd[4 + i] = pBuf[i];

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4 + 16, NULL, 0);

	if (status == OK)
	{
		// Must wait 200 usec for a page to program.  This is a constant for all devices (see XO2 datasheet)
		pXO2->pfuSecDelay(200);
		status = XO2ECAcmd_waitStatusBusy(pXO2);
	}

#ifdef DEBUG_ECA
	printf("\tstatus=%d\r\n", status);
#endif
	if (status == OK)
	{
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}

/**
 * Erase the entire sector of the UFM memory.
 * This is a convience function to erase all UFM contents to 0.  You can not erase on a page basis.
 * The entire sector is cleared.  Therefore save any data first, erase, then
 * reprogram, putting the saved data back along with any new data.
 * The erase takes up to a few hundered milliseconds.  The time to wait is device specific.
 * It is important that the correct device is selected in the pXO2 structure.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @return OK if successful, ERROR if failed.
 * 
 * @note Erases bits to a value of 0.  Any bit that is a 0 can then be programmed to a 1.
 * @note The routine does not poll for completion, but rather waits the maximum time
 * for an erase as specified in the data sheet.
 */
int XO2ECAcmd_UFMErase(XO2Handle_t *pXO2) 
{

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_UFMErase()\r\n");
#endif

	return(XO2ECAcmd_EraseFlash(pXO2, XO2ECA_CMD_ERASE_UFM));
}


//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================
//                            F E A T U R E    R O W     C O M M A N D S
//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================


/**
 * Erase the Feature Row contents.
 * This is a convience function to erase just the feature row bits to 0.
 * You must reprogram the Feature Row with FeatureWrite() or the
 * XO2 part may be in an unusable state.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @return OK if successful, ERROR if failed.
 * 
 * @note Erases bits to a value of 0.  Any bit that is a 0 can then be programmed to a 1.
 * 
 */
int XO2ECAcmd_FeatureRowErase(XO2Handle_t *pXO2) 
{

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_FeatureErase()\r\n");
#endif

	return(XO2ECAcmd_EraseFlash(pXO2, XO2ECA_CMD_ERASE_FTROW));
}


/**
 * Set the Feature Row.
 * This function assembles the command sequence that allows programming the XO2 FEATURE
 * bits and the FEABITS in the Feature Row.  The 8 FEATURE bytes and 2 FEABITS bytes 
 * must be properly formatted or possible
 * lock-up of the XO2 could occur.  Only the values obtained from properly parsing the JEDEC
 * file should be used. 
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @param pFeature pointer to the Feature Row structure containing the encoded data to write
 * into the Feature and FEABITS fields in the Feature Row.
 * @return OK if successful, ERROR if failed to write
 * 
 * @note The Feature Row must first be erased
 */
int XO2ECAcmd_FeatureRowWrite(XO2Handle_t *pXO2, XO2FeatureRow_t *pFeature) 
{
	unsigned char cmd[16];
	int i, status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_FeatureWrite()\r\n");
#endif

	if (pXO2->cfgEn == FALSE)
	{
		return(ERR_XO2_NOT_IN_CFG_MODE);
	}

	cmd[0] = 0xE4;  // program Feature opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

	for (i = 0; i < 8; i++)
		cmd[4 + i] = pFeature->feature[i];

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 12, NULL, 0);
	
	if (status != OK)
		return(ERROR);
	
	// Must wait 200 usec for a page to program.  This is a constant for all devices (see XO2 datasheet)
	pXO2->pfuSecDelay(200);


	cmd[0] = 0xF8;  // program FEABITS opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

	cmd[4] = pFeature->feabits[0];
	cmd[5] = pFeature->feabits[1];

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 6, NULL, 0);
	
	if (status == OK)
	{
		// Must wait 200 usec for a page to program.  This is a constant for all devices (see XO2 datasheet)
		pXO2->pfuSecDelay(200);
		status = XO2ECAcmd_waitStatusBusy(pXO2);
	}



	if (status == OK)
	{
		return(OK);
	}
	else
	{
		return(ERROR);
	}
}



/**
 * Read the Feature Row contents.
 * This function assembles the command sequence that allows reading back the Feature Row
 * contents.  The FEATURE bytes and FEABITS fields are returned in a XO2 specific stucture. 
 * Uses would be to verify successful XO2ECAcmd_FeatureWrite() programing. Or to read back
 * and preserve during an update.  
 * @param pXO2 pointer to the XO2 device to access
 * @param pFeature pointer to the Feature Row structure that will be loaded with the
 * feature row contents.
 * @return OK if successful, ERROR if failed to read
 * 
 */
int XO2ECAcmd_FeatureRowRead(XO2Handle_t *pXO2, XO2FeatureRow_t *pFeature) 
{
	unsigned char cmd[4];
	unsigned char data[8];

	int i, status;

#ifdef DEBUG_ECA
	printf("XO2ECAcmd_FeatureRead()\r\n");
#endif

	if (pXO2->cfgEn == FALSE)
	{
		return(ERR_XO2_NOT_IN_CFG_MODE);
	}

	cmd[0] = 0xE7;  // Read Feature opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

	status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, data, 8);

	if (status != OK)
		return(ERROR);

	for (i = 0; i < 8; i++)
		pFeature->feature[i] = data[i];


	cmd[0] = 0xFB;  // Read FEABITS opcode
	cmd[1] = 0x00;  // arg0
	cmd[2] = 0x00;  // arg1
	cmd[3] = 0x00;  // arg2

    status = MachXO_CmdXfer(pXO2->pI2CDrvrCalls, cmd, 4, data, 2);
	
	if (status != OK)
		return(ERROR);

	pFeature->feabits[0] = data[0];
	pFeature->feabits[1] = data[1];

	return(OK);
}

/**
 * Erase the SRAM, clearing the user design.
 * This is a convience function to erase just the SRAM.
 * 
 * @param pXO2 pointer to the XO2 device to access
 * @return OK if successful, ERROR if failed.
 */
int XO2ECAcmd_SRAMErase(XO2Handle_t *pXO2) 
{
#ifdef DEBUG_ECA
	printf("XO2ECAcmd_SRAMErase()\r\n");
#endif

	return(XO2ECAcmd_EraseFlash(pXO2, XO2ECA_CMD_ERASE_SRAM));
}


// Callback implementations
void MXO_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    txComplete = 1;
}

void MXO_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    rxComplete = 1;
}

void MXO_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    i2cError = 1;
}
