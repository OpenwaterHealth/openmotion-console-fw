/*
 *  COPYRIGHT (c) 2012 by Lattice Semiconductor Corporation
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

/** @file XO2_cmds.h */
 
#ifndef LATTICE_XO2_CMDS_H
#define LATTICE_XO2_CMDS_H

#include "XO2_dev.h"
#include "stm32h7xx_hal.h"

/* Openwater local change (Core/Src/LOCAL_CHANGES.md): the erase/program
 * commands are declared and compiled in FACTORY builds only. */

//--------------------------------------------
//      E R R O R   C O D E S
//--------------------------------------------
#define ERR_XO2_NOT_IN_CFG_MODE (-100)
#define ERR_XO2_NO_UFM          (-101)
#define ERR_XO2_EXCEEDS_UFM_SIZE (-102)
#define ERR_XO2_EXCEEDS_CFG_SIZE (-103)



#define XO2ECA_CMD_LOOP_TIMEOUT    25000 // number of times to poll in a loop before aborting
#define XO2ECA_CMD_ERASE_UFM   8
#define XO2ECA_CMD_ERASE_CFG   4
#define XO2ECA_CMD_ERASE_FTROW 2
#define XO2ECA_CMD_ERASE_SRAM  1



//--------------------------------------------
//      L o w - l e v e l   T r a n s p o r t
//--------------------------------------------
/** Send wbuf[0..wcnt-1] over I2C, optionally read rcnt bytes into rbuf. */
uint32_t MachXO_CmdXfer(MachXO_Handle_t *h,
                        const uint8_t *wbuf, int wcnt,
                        uint8_t *rbuf,       int rcnt);

//--------------------------------------------
//      G e n e r a l   C o m m a n d s
//--------------------------------------------
int XO2ECAcmd_readDevID(XO2Handle_t *pXO2, unsigned int *pVal) ;
int XO2ECAcmd_readUserCode(XO2Handle_t *pXO2, unsigned int *pVal) ;
#if defined(OW_FACTORY_PROGRAMMING)
int XO2ECAcmd_setUserCode(XO2Handle_t *pXO2, unsigned int val) ;
#endif
int XO2ECAcmd_readTraceID(XO2Handle_t *pXO2, unsigned char *pVal) ;

int XO2ECAcmd_Bypass(XO2Handle_t *pXO2) ;

int XO2ECAcmd_closeCfgIF(XO2Handle_t *pXO2); 
int XO2ECAcmd_openCfgIF(XO2Handle_t *pXO2, XO2CfgMode_t mode); 

int XO2ECAcmd_readStatusReg(XO2Handle_t *pXO2, unsigned int *pVal) ;
int XO2ECAcmd_readBusyFlag(XO2Handle_t *pXO2, unsigned char *pVal) ;
int XO2ECAcmd_waitStatusBusy(XO2Handle_t *pXO2) ;
int XO2ECAcmd_waitBusyFlag(XO2Handle_t *pXO2) ;


int XO2ECAcmd_SetPage(XO2Handle_t *pXO2, XO2SectorMode_t mode, unsigned int pageNum) ;

#if defined(OW_FACTORY_PROGRAMMING)
int XO2ECAcmd_EraseFlash(XO2Handle_t *pXO2, unsigned char mode) ;
int XO2ECAcmd_SRAMErase(XO2Handle_t *pXO2) ;
#endif


//--------------------------------------------
//      C F G   C o m m a n d s
//--------------------------------------------

#if defined(OW_FACTORY_PROGRAMMING)
int XO2ECAcmd_setDone(XO2Handle_t *pXO2) ;
#endif
int XO2ECAcmd_Refresh(XO2Handle_t *pXO2); 

#if defined(OW_FACTORY_PROGRAMMING)
int XO2ECAcmd_CfgErase(XO2Handle_t *pXO2) ;
#endif
int XO2ECAcmd_CfgResetAddr(XO2Handle_t *pXO2) ;
int XO2ECAcmd_CfgReadPage(XO2Handle_t *pXO2, unsigned char *pBuf) ;
#if defined(OW_FACTORY_PROGRAMMING)
int XO2ECAcmd_CfgWritePage(XO2Handle_t *pXO2, const unsigned char *pBuf) ;
#endif



//--------------------------------------------
//      U F M   C o m m a n d s
//--------------------------------------------
#if defined(OW_FACTORY_PROGRAMMING)
int XO2ECAcmd_UFMErase(XO2Handle_t *pXO2) ;
#endif
int XO2ECAcmd_UFMResetAddr(XO2Handle_t *pXO2);
#if defined(OW_FACTORY_PROGRAMMING)
int XO2ECAcmd_UFMWritePage(XO2Handle_t *pXO2, const unsigned char *pBuf) ;
#endif
int XO2ECAcmd_UFMReadPage(XO2Handle_t *pXO2, unsigned char *pBuf) ;



//--------------------------------------------
//     F E A T U R E  RO W   C o m m a n d s
//--------------------------------------------
#if defined(OW_FACTORY_PROGRAMMING)
int XO2ECAcmd_FeatureRowErase(XO2Handle_t *pXO2);
int XO2ECAcmd_FeatureRowWrite(XO2Handle_t *pXO2, XO2FeatureRow_t *pFeature) ;
#endif
int XO2ECAcmd_FeatureRowRead(XO2Handle_t *pXO2, XO2FeatureRow_t *pFeature) ;


void MXO_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void MXO_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void MXO_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);


#endif
