/*
 * PMTDriver.h
 *
 *  Created on: 2019年11月4日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_OPTICALDRIVER_PMTDRIVER_H_
#define SRC_DRIVER_OPTICALDRIVER_PMTDRIVER_H_

#include "stm32f4xx.h"
#include "Common/Types.h"

void PMTDriver_Init(void);
void PMTDriver_PowerPinInit(void);

#endif /* SRC_DRIVER_OPTICALDRIVER_PMTDRIVER_H_ */
