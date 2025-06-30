/*
 * PMTControl.h
 *
 *  Created on: 2019年11月4日
 *      Author: Administrator
 */

#ifndef SRC_OPTICALCONTROL_PMTCONTROL_H_
#define SRC_OPTICALCONTROL_PMTCONTROL_H_

#include "stm32f4xx.h"
#include "Common/Types.h"

typedef struct
{
    Uint32 refValue;
    Uint32 meaValue;
}PmtResult;

typedef enum
{
        PMT_IDLE  = 0,
        PMT_BUSY = 1,
}PMT_STATUS;

#define PMT_CHN_REF    0
#define PMT_CHN_MEA   1

void PMTControl_Init(void);
Bool PMTControl_StartAcquire(Uint32 timeout, Bool isSendEvent);
void PMTControl_StopAcquire(Bool isSendEvent);
Bool PMTControl_GetNewResult(PmtResult* result);
Uint8 PMTControl_GetStatus(void);
void PMTControl_PowerOn(void);
void PMTControl_PowerOff(void);
Bool PMTControl_PowerStatus(void);
void PMTControl_Start(void);
void PMTControl_ReadResult(void);
Uint16 PMTControl_ReadWord(Uint8 chn);
Bool PMTControl_WriteWord(Uint8 chn, Uint16 cmd);
Uint32 PMTControl_ReadValue(Uint8 chn);
Bool PMTControl_WriteByte(Uint8 chn, Uint8 cmd);
Bool PMTControl_SetTime(Uint8 chn, Uint16 time);
Uint16 PMTControl_GetTime(Uint8 chn);
Bool PMTControl_StartCount(Uint8 chn);
Uint32 PMTControl_ReadData(Uint8 chn);
Bool PMTControl_SetPower(Uint8 chn, Bool state);

#endif /* SRC_OPTICALCONTROL_PMTCONTROL_H_ */
