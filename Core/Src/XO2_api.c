/*
 *  COPYRIGHT (c) 2012 by Lattice Semiconductor Corporation
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

/** @file XO2_api.c 
 * The XO2 Embedded Configuration Access (ECA) API routines.
 * The routines provided here are for high-level operations.
 * They rely on calls to the routines in XO2_cmds.c to implement
 * the underlying register reads and writes via the communications
 * link to the XO2.
 */
#include <stdio.h>

#include "XO2_cmds.h"
#include "XO2_api.h"

/* Openwater local change (Core/Src/LOCAL_CHANGES.md): the programming entry
 * points are compiled only into FACTORY builds (OW_FACTORY_PROGRAMMING). The
 * read-only entry points (ReadBackCfg, ReadBackUFM, GetHdwInfo, GetHdwStatus,
 * JEDECinfo, JEDECverify) stay in every build for fpga_attest.c. */



/**
 * Erase and Program the Config, UFM and/or FeatureRow sectors of the XO2 Flash.
 * The caller can select to program individually any sector, and also perform
 * a verify of the operation. The program data is taken from the converted
 * JEDEC file.
 * 
 * Mode Values:
 * <UL> 
 *  <LI> 0x20 = Verify after programming
 *  <LI> 0x10 = program in Transparent (user logic runs), 0=Offline (design halts) mode
 *  <LI> 0x08 = Erase/program UFM sector
 *  <LI> 0x04 = Erase/program CFG sector
 * 	<LI> 0x02 = Erase/program Feature Row
 * </UL>
 * 
 * General rules for programming modes:
 * <UL>
 * <LI> Offline - recommended for reprogramming entire part, including Feature Row
 * <LI> Transparent - use to update UFM and/or Cfg sectors of a working design.
 * 		Feature row should not be erased in Transparent mode.
 * </UL>
 * @param pXO2dev reference to the XO2 device to access and program
 * @param pProgJED reference to the converted XO2 JEDEC file data 
 * @param mode bitmap of what to erase/program and whether to verify or not
 * 
 * @note If programming fails, an attempt is made to close confgiuration mode. 
 * The user may wish to retry or erase the Cfg and UFM flash sectors
 * in an attempt to return the device to a blank state.  
 * @see XO2ECA_apiClearXO2()
 * 
 */
#if defined(OW_FACTORY_PROGRAMMING)
int XO2ECA_apiProgram(XO2Handle_t *pXO2dev, const XO2_JEDEC_t *pProgJED, int mode)
{
	int status, ret;
	unsigned int	i, j;
	const unsigned char *p;
	unsigned char buf[XO2_FLASH_PAGE_SIZE];
	int numPgs;
	XO2FeatureRow_t featRow;
	
	ret = -99;  // initialize to unknown error value
	if (mode & XO2ECA_PROGRAM_TRANSPARENT)
	{
		status = XO2ECAcmd_openCfgIF(pXO2dev, TRANSPARENT_MODE); 
		
		// Prevent erasing the Feature Row in Transparent mode.  The user logic is running and
		// operating per the settings of the current Feature Row.  Erasing it can lead to
		// instability in the running design.  Use Offline mode (design halted) when re-
		// programming Feature Row (changing device behavior).
		mode = mode &  ~XO2ECA_ERASE_PROG_FEATROW;
	}
	else
	{
		status = XO2ECAcmd_openCfgIF(pXO2dev, OFFLINE_MODE);
	}
	
	if (status != OK)
		return(-1);	// Error. Could not open XO2 configuration 



	//=======================================================================================
	//=======================================================================================
	//=======================================================================================
	//             ERASE SECTORS: Config, UFM, and/or Feature Row
	//=======================================================================================
	//=======================================================================================
	//=======================================================================================
	status = XO2ECAcmd_EraseFlash(pXO2dev, mode);
	if (status != OK)
	{
		ret = -2;
		goto PROG_ABORT;
	}




	//=======================================================================================
	//=======================================================================================
	//=======================================================================================
	//             PROGRAM/VERIFY    Config SECTOR
	//=======================================================================================
	//=======================================================================================
	//=======================================================================================
	if (mode & XO2ECA_ERASE_PROG_CFG)
	{
#ifdef DEBUG_ECA
		printf("Cfg Sector Program/Verify\r\n");
#endif

		status = XO2ECAcmd_CfgResetAddr(pXO2dev); 
		if (status != OK)
		{
			ret = -11;
			goto PROG_ABORT;
		}
		
		numPgs = (pProgJED->CfgDataSize) / XO2_FLASH_PAGE_SIZE;

		p = pProgJED->pCfgData;
		for (j = 0; j < numPgs; ++j)
		{
#ifdef DEBUG_ECA
			printf("Cfg page: %u\r\n", j + 1);
#endif
			status = XO2ECAcmd_CfgWritePage(pXO2dev, p);

			if (status != OK)
			{
				ret = -12;
				goto PROG_ABORT;
			}
			p = p + XO2_FLASH_PAGE_SIZE; // next page
		}


		if (mode &  XO2ECA_PROGRAM_VERIFY)
		{
			// If verify then read back and make sure each page read back matches what is in JEDEC data struct
			status = XO2ECAcmd_CfgResetAddr(pXO2dev);
			if (status != OK)
			{
				ret = -13;
				goto PROG_ABORT;
			}
			
			p = pProgJED->pCfgData;  // reset back to beginning of Cfg data
			
			for (i = 0; i < numPgs; i++)
			{
#ifdef DEBUG_ECA
				printf("Verify CfgPage: %u\r\n", i + 1);
#endif
							
				// Read back the programmed page
				status = XO2ECAcmd_CfgReadPage(pXO2dev, buf);
				if (status != OK)
				{
#ifdef DEBUG_ECA
					printf("CfgReadPage(%u) ERR\r\n", i + 1);
#endif
					ret = -14;
					goto PROG_ABORT;
				}
				
				for (j = 0; j < XO2_FLASH_PAGE_SIZE; j++)
				{
					if (buf[j] != p[j])
					{
#ifdef DEBUG_ECA
						printf("Verify CfgPage(%u) ERR\r\n", i + 1);
#endif
						ret = -15;
						goto PROG_ABORT;
					}
				}
				p = p + XO2_FLASH_PAGE_SIZE;  // point to next page of cfg data for checking
			}
		}

	}

	//=======================================================================================
	//=======================================================================================
	//=======================================================================================
	//               PROGRAM/VERIFY      UFM SECTOR
	//=======================================================================================
	//=======================================================================================
	//=======================================================================================
	if (mode & XO2ECA_ERASE_PROG_UFM)
	{
#ifdef DEBUG_ECA
		printf("UFM Sector Program/Verify\r\n");
#endif

		status = XO2ECAcmd_UFMResetAddr(pXO2dev); 
		if (status != OK)
		{
			ret = -21;
			goto PROG_ABORT;
		}
			
		numPgs = (pProgJED->UFMDataSize) / XO2_FLASH_PAGE_SIZE;

		p = pProgJED->pUFMData;
		for (j = 0; j < numPgs; ++j)
		{
#ifdef DEBUG_ECA
			printf("UFM page: %u\r\n", j + 1);
#endif
			status = XO2ECAcmd_UFMWritePage(pXO2dev, p);
			if (status != OK)
			{
				ret = -22;
				goto PROG_ABORT;
			}

			p = p + XO2_FLASH_PAGE_SIZE; // next page
		}
		

		if (mode &  XO2ECA_PROGRAM_VERIFY)
		{
			// If verify then read back and make sure each page read back matches what is in JEDEC data struct
			status = XO2ECAcmd_UFMResetAddr(pXO2dev); 
			if (status != OK)
			{
				ret = -23;
				goto PROG_ABORT;
			}

			
			p = pProgJED->pUFMData;  // reset back to beginning of UFM data
			
			for (i = 0; i < numPgs; i++)
			{
#ifdef DEBUG_ECA
				printf("Verify UFMPage: %u\r\n", i + 1);
#endif
				
				// Readback the programmed page
				status = XO2ECAcmd_UFMReadPage(pXO2dev, buf);
				if (status != OK)
				{
#ifdef DEBUG_ECA
					printf("UFMReadPage(%u) ERR\r\n", i + 1);
#endif
					ret = -24;
					goto PROG_ABORT;
				}
				
				for (j = 0; j < XO2_FLASH_PAGE_SIZE; j++)
				{
					if (buf[j] != p[j])
					{
#ifdef DEBUG_ECA
						printf("Verify UFMPage(%u) ERR\r\n", i + 1);
#endif
						ret = -25;
						goto PROG_ABORT;
					}
				}
				p = p + XO2_FLASH_PAGE_SIZE;  // point to next page of UFM data for checking
			}
		
		}

	}


	//=======================================================================================
	//=======================================================================================
	//=======================================================================================
	//               PROGRAM/VERIFY      FEATURE ROW
	//=======================================================================================
	//=======================================================================================
	//=======================================================================================

	if (mode & XO2ECA_ERASE_PROG_FEATROW)
	{
#ifdef DEBUG_ECA
		printf("Feature Row Program/Verify\r\n");
#endif

		status = XO2ECAcmd_FeatureRowWrite(pXO2dev, pProgJED->pFeatureRow);
		if (status != OK)
		{
			ret = -31;
			goto PROG_ABORT;
		}
		
		
		if (mode &  XO2ECA_PROGRAM_VERIFY)
		{
	
			status = XO2ECAcmd_FeatureRowRead(pXO2dev, &featRow);
			
#ifdef DEBUG_ECA
			printf("Feature Contents: %x %x %x %x %x %x %x %x\r\n", featRow.feature[0],
			featRow.feature[1],featRow.feature[2],featRow.feature[3],featRow.feature[4],featRow.feature[5],
			featRow.feature[6],featRow.feature[7]);
			printf("FEABITS: %x %x\r\n", featRow.feabits[0], featRow.feabits[1]);
#endif

			for (i = 0; i < 8; i++)
			{
				if (featRow.feature[i] != pProgJED->pFeatureRow->feature[i])
				{
#ifdef DEBUG_ECA
					printf("FeatureRow Verify ERR @ feature[%u]\r\n", i);
#endif
					ret = -32;
					goto PROG_ABORT;
				}
			}
			for (i = 0; i < 2; i++)
			{
				if (featRow.feabits[i] != pProgJED->pFeatureRow->feabits[i])
				{
#ifdef DEBUG_ECA
					printf("FeatureRow Verify ERR @ feabits[%u]\r\n", i);
#endif
					ret = -33;
					goto PROG_ABORT;
				}
			}
			
		}
	}
	
	
	
	//=======================================================================================
	//=======================================================================================
	//=======================================================================================
	//               FINALIZE and CLOSE CONFIG INTERFACE
	//=======================================================================================
	//=======================================================================================
	//=======================================================================================
	
	// Set DONE bit indicating valid design loaded into flash
	status = XO2ECAcmd_setDone(pXO2dev);
	if (status != OK)
	{
		ret = -40;
		goto PROG_ABORT;
	}

	// Boot design to user mode (i.e. like hitting PROGRAM pin)	
	// Refresh command will clear SRAM, load from Flash, set Done, exit config mode.
	// Sometimes it needs to be called more than once.
	// But eventually it boots and Done goes high.	
	i = 10;
	while (i && (XO2ECAcmd_Refresh(pXO2dev) != OK))
	{
		--i;
	}

	if (i)
		return(OK);  // Done got set before timeout
	else
		return(-42);  // failed exiting config mode and/or refreshing


	
	// This is clean-up from aborting.  Close, but don't set done or refresh
	// User may wish to erase sectors they attempted to program and/or
	// put part into a blank state.
	// See XO2ECA_apiClearXO2() for clearing XO2 to a blank state.
PROG_ABORT:
	XO2ECAcmd_closeCfgIF(pXO2dev); 
	XO2ECAcmd_Bypass(pXO2dev); 
	return(ret);
}


/**
 * Clear a failed programming attempt.
 * Call when programming fails.  This erases the Config, UFM, Feature Row sectors
 * of the XO2 Flash and clears the SRAM by doing a Refresh with the now blank flash
 * contents.  This will put the part into a factory blank state.
 * Factory blank state has I2C and SPI enabled for configuration, so a new programming
 * cycle can be re-attempted.
 * 
 * @param pXO2dev reference to the XO2 device to access and program
 */
int XO2ECA_apiClearXO2(XO2Handle_t *pXO2dev)
{
	int status;

	status = XO2ECAcmd_openCfgIF(pXO2dev, OFFLINE_MODE);
	
	if (status != OK)
		return(-1);	// Error. Could not open XO2 configuration 
	
	status = XO2ECAcmd_EraseFlash(pXO2dev, 0x0e);   // Erase Cfg, UFM, FR

	
	status = status | XO2ECAcmd_Refresh(pXO2dev);

	return(status);
}



/**
 * Erase the Config and/or the UFM sectors of the XO2 Flash.
 * The caller can select to erase either the Config or UFM or both sectors.
 * This would typically be used during testing and debugging to restore the XO2
 * part to factory fresh state, before programming with a new image.
 * 
 * <ul> Mode Values
*  <li> 0x01 = program in Transparent (user logic runs) or Offline (design halts) mode
*  <li> 0x02 = Erase CFG sector
 * <li> 0x04 =  Erase UFM sector
 * <li> 0x08 = Erase Feature Row
 * </ul>
 * @param pXO2dev reference to the XO2 device to access and program
  * @param mode bitmap of features
 */
int XO2ECA_apiEraseFlash(XO2Handle_t *pXO2dev,  int mode)
{
	return(XO2ECAcmd_EraseFlash(pXO2dev, mode)); 

}



#endif /* OW_FACTORY_PROGRAMMING: XO2ECA_apiProgram, XO2ECA_apiClearXO2, XO2ECA_apiEraseFlash */

/**
 * Display info about JEDEC data structure.
 * @param pXO2dev reference to the XO2 device to access and program
 */
void XO2ECA_apiJEDECinfo(XO2Handle_t *pXO2dev, const XO2_JEDEC_t *pProgJED)
{

	printf("JEDEC Data Structure:\r\n");
	printf("DeviceID = %s (%d)\r\n", XO2DevList[pProgJED->devID].pName, pProgJED->devID);
	printf("PageCount = %d\r\n", pProgJED->pageCnt);
	printf("CfgDataSize = %d bytes (%d pages)\r\n", pProgJED->CfgDataSize, pProgJED->CfgDataSize / XO2_FLASH_PAGE_SIZE);
	printf("UFMDataSize = %d bytes (%d pages)\r\n", pProgJED->UFMDataSize, pProgJED->UFMDataSize / XO2_FLASH_PAGE_SIZE);
	printf("USERCODE = 0x%08x\r\n", pProgJED->UserCode);
	printf("Security = 0x%08x\r\n", pProgJED->SecurityFuses);

}


/**
 * Verify JEDEC data structure.
 * Compare JEDEC device type and Cfg/UFM sizes to the device listed by the pXO2dev.
 *  @param pXO2dev reference to the XO2 device to access and program
 * @return OK if compatible.  ERROR if JEDEC file for different device, not compatible.
 */
int XO2ECA_apiJEDECverify(XO2Handle_t *pXO2dev, const XO2_JEDEC_t *pProgJED)
{


	return(NOT_IMPLEMENTED_ERR);
}



/**
 * Readback and save the Configuration FLash area.
 * This would be used to compare what was written, or save current device design before erasing.
 *  @param pXO2dev reference to the XO2 device to access and program
 */
int XO2ECA_apiReadBackCfg(XO2Handle_t *pXO2dev, unsigned char *pBuf)
{
	return(NOT_IMPLEMENTED_ERR);

}


/**
 * Readback and save the User FLash Memory (UFM) area.
 * This would be used to read all, or just some pages, of the UFM to either save it before
 * programming (i.e. update a section with new data and reprogram) or to retrieve info
 * stored there previously, perhaps via CPU in XO2 or other method.
 * Example usage:
 @code
   XO2ECA_apiReadBackUFM(pXO2dev, 0, -1, storeUFM);  // read all pages of UFM
   XO2ECA_apiReadBackUFM(pXO2dev, newRecord, 1, );   // read a page (perhaps written by LM8 on XO2) from UFM
 @endcode
 * @param pXO2dev reference to the XO2 device to access and read
 * @param startPg starting page to read from.  Numbering starts with page 0.
 * @param numPgs how many to read back.  -1 means read all
 * @param pBuf pointer to storage for bytes read from UFM, must be multiple of page size, and big enough.
 */
int XO2ECA_apiReadBackUFM(XO2Handle_t *pXO2dev, int startPg, int numPgs, unsigned char *pBuf)
{
	int status;
	int devIndex;	
	int i;
	int ret;
	
	ret = OK;
	
		
	devIndex = pXO2dev->devType;
	
	if (numPgs == -1)
		numPgs =  XO2DevList[devIndex].UFMpages;
	
	// first check if pages are in the range supported by this device
	if (startPg + numPgs > XO2DevList[devIndex].UFMpages)
	{
#ifdef DEBUG_ECA
		printf("Page Range ERR\r\n");
#endif
		return(-1);
	}
	
	status = XO2ECAcmd_openCfgIF(pXO2dev, TRANSPARENT_MODE); 
	if (status != OK)
	{
#ifdef DEBUG_ECA
		printf("XO2ECAcmd_openCfgIF() ERR\r\n");
#endif
		return(-2);	
	}
	
	// Set the UFM beginning page to write to
	status = XO2ECAcmd_SetPage(pXO2dev, UFM_SECTOR, startPg);
	if (status != OK)
	{
#ifdef DEBUG_ECA
		printf("XO2ECAcmd_SetPage(%d) ERR\r\n", startPg);
#endif
		return(-3);
	}
	
	for (i = 0; i < numPgs; i++)
	{
		// Write the next page with the new pattern
		status = XO2ECAcmd_CfgReadPage(pXO2dev, pBuf);
		if (status != OK)
		{
#ifdef DEBUG_ECA
			printf("XO2ECAcmd_CfgReadPage(%d) ERR\r\n", startPg + i);
#endif
			ret = -11;
			break;
		}
		pBuf = pBuf + 16;  // point to next page of data
	}
	
	(void)XO2ECAcmd_closeCfgIF(pXO2dev); 

	(void)XO2ECAcmd_Bypass(pXO2dev); 
	
	return(ret);

}


#if defined(OW_FACTORY_PROGRAMMING)
/**
 * Program the UFM area with raw data.
 * Specify the page range to write over.  checking is done to validate the page range.
 * @param pXO2dev reference to the XO2 device to access and write
 * @param startPg starting page to read from.  Numbering starts with page 0.
 * @param numPgs specifies the amount of valid data bytes (numPg * 16) in pBuf.  
 * Can be set to 0 to just erase the UFM contents and not do any programming at this time.
 * @param pBuf pointer to bytes to program into UFM, must be multiple of page size.
 * @param erase if set, then the entire UFM area is erased first.  Otherwise writing expects the pages
 * in the range to have already been erased (cleared to 0's).
 */
int XO2ECA_apiWriteUFM(XO2Handle_t *pXO2dev, int startPg, int numPgs, unsigned char *pBuf, int erase)
{
	int status;
	int devIndex;	
	int i;
	int ret;
	
	ret = OK;
	
	devIndex = pXO2dev->devType;
	// first check if pages are in the range supported by this device
	if (startPg + numPgs > XO2DevList[devIndex].UFMpages)
	{
#ifdef DEBUG_ECA
		printf("Page Range ERR\r\n");
#endif
		return(-1);
	}
	
	status = XO2ECAcmd_openCfgIF(pXO2dev, TRANSPARENT_MODE); 
	if (status != OK)
	{
#ifdef DEBUG_ECA
		printf("XO2ECAcmd_openCfgIF() ERR\r\n");
#endif
		return(-2);	
	}
	
	
	if (erase)
	{
		status = XO2ECAcmd_UFMErase(pXO2dev); 
		if (status != OK)
			return(-5);	

		status = XO2ECAcmd_UFMResetAddr(pXO2dev); 
		if (status != OK)
			return(-6);	
		
	}
	
	// Set the UFM beginning page to write to
	status = XO2ECAcmd_SetPage(pXO2dev, UFM_SECTOR, startPg);
	if (status != OK)
	{
#ifdef DEBUG_ECA
		printf("XO2ECAcmd_SetPage(%d) ERR\r\n", startPg);
#endif
		return(-3);
	}
	
	for (i = 0; i < numPgs; i++)
	{
		// Write the next page with the new pattern
		status = XO2ECAcmd_CfgWritePage(pXO2dev, pBuf);
		if (status != OK)
		{
#ifdef DEBUG_ECA
			printf("XO2ECAcmd_CfgWritePage(%d) ERR\r\n", startPg + i);
#endif
			ret = -11;
			break;
		}
		pBuf = pBuf + 16;  // point to next page of data
	}
	
	(void)XO2ECAcmd_closeCfgIF(pXO2dev); 

	(void)XO2ECAcmd_Bypass(pXO2dev); 
	
	return(ret);
}



#endif /* OW_FACTORY_PROGRAMMING: XO2ECA_apiWriteUFM */

/**
 * Read the DeviceID, USERCODE and TraceID registers in the hardware device.
 * Return them in the structure.
 * Also looks up the corresponding index into the XO2 Device Features database.
 *  @param pXO2dev reference to the XO2 device to access
 */
int XO2ECA_apiGetHdwInfo(XO2Handle_t *pXO2dev, XO2RegInfo_t *pInfo)
{
	int status;

	// Read Device ID
	status = XO2ECAcmd_readDevID(pXO2dev, &(pInfo->devID));
	if(status != OK) return status;

	// Read USERCODE
	status = XO2ECAcmd_readUserCode(pXO2dev, &(pInfo->UserCode));
	if(status != OK) return status;

	// Read TraceID
	status = XO2ECAcmd_readTraceID(pXO2dev, pInfo->TraceID);
	if(status != OK) return status;

// TODO: fix this
	pInfo->devInfoIndex = 3;  // hard code for now

	return(OK);
}


/**
 * Read the status register and decode the various bits.
 * Return in a more compact format.
 * <p>
 * Format is:
 * <p>
 * <b><code>xEEE_xFBD</code></b>: 
 * upper nibble is Flash Check status, lower nibble is general status.
 * <ul>
 * <li>EEE = flash check error code: 000 = No Err
 * <li>F = Fail Flag
 * <li>B = Busy Flag
 * <li>D = Done Flag
 * </ul>
 *  @param pXO2dev reference to the XO2 device to access
 */
int XO2ECA_apiGetHdwStatus(XO2Handle_t *pXO2dev, unsigned int *pVal)
{
	int status;
	unsigned int regVal;

//	status = XO2ECAcmd_FlashCheck(&xo2_dev);

	status = XO2ECAcmd_readStatusReg(pXO2dev, &regVal);
	if (status == OK)
	{
		printf("XO2 Status Register = %x\r\n", regVal);
		*pVal = 0;
		if (regVal & 0x00000100u)
			*pVal |= 1u;
		if (regVal & 0x00001000u)
			*pVal |= 2u;
		if (regVal & 0x00002000u)
			*pVal |= 4u;
		*pVal |= ((regVal >> 19) & 0x7U) << 4;
	}
	else
	{
		printf("ERROR!!!\r\n");
	}

	
	return(status);

}
