/*
 * PMTDriver.c
 *
 *  Created on: 2019年11月4日
 *      Author: Administrator
 */
#include "PMTDriver.h"
#include "Common/Types.h"
#include "SystemConfig.h"
#include "Tracer/Trace.h"
#include "FreeRTOS.h"
#include "task.h"

/*************PMT_POWER******************/
#define PMT_POWER_PLUS_PIN              GPIO_Pin_4
#define PMT_POWER_PLUS_PORT             GPIOC
#define PMT_POWER_PLUS_RCC              RCC_AHB1Periph_GPIOC
#define PMT_POWER_PLUS_ON()                GPIO_SetBits(PMT_POWER_PLUS_PORT, PMT_POWER_PLUS_PIN)
#define PMT_POWER_PLUS_OFF()              GPIO_ResetBits(PMT_POWER_PLUS_PORT, PMT_POWER_PLUS_PIN)

#define PMT_POWER_MINUS_PIN              GPIO_Pin_5
#define PMT_POWER_MINUS_PORT             GPIOC
#define PMT_POWER_MINUS_RCC              RCC_AHB1Periph_GPIOC
#define PMT_POWER_MINUS_ON()                GPIO_SetBits(PMT_POWER_MINUS_PORT, PMT_POWER_MINUS_PIN)
#define PMT_POWER_MINUS_OFF()              GPIO_ResetBits(PMT_POWER_MINUS_PORT, PMT_POWER_MINUS_PIN)

/**
 * @brief PMT驱动初始化
 */
void PMTDriver_Init(void)
{
    PMTDriver_PowerPinInit();
}

/**
 * @brief 电源控制引脚初始化
 */
void PMTDriver_PowerPinInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(PMT_POWER_PLUS_RCC | PMT_POWER_MINUS_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = PMT_POWER_PLUS_PIN;
    GPIO_Init(PMT_POWER_PLUS_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PMT_POWER_MINUS_PIN;
    GPIO_Init(PMT_POWER_MINUS_PORT, &GPIO_InitStructure);

    PMT_POWER_PLUS_ON();
    PMT_POWER_MINUS_ON();
}

/**
 * @brief 电源使能
 */
void PMTDriver_PowerOn(void)
{
    PMT_POWER_PLUS_ON();
    PMT_POWER_MINUS_ON();
}

/**
 * @brief 电源关闭
 */
void PMTDriver_PowerOff(void)
{
    PMT_POWER_PLUS_OFF();
    PMT_POWER_MINUS_OFF();
}



