/*
 * PMTControl.c
 *
 *  Created on: 2019年11月4日
 *      Author: Administrator
 */

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"
#include "PMTControl.h"
#include "LuipApi/OpticalAcquireInterface.h"
#include "Driver/OpticalDriver/PMTDriver.h"
#include "Driver/OpticalDriver/AD5175Driver.h"
#include "DncpStack/DncpStack.h"
#include "Common/Types.h"
#include "SystemConfig.h"
#include "Tracer/Trace.h"
#include "semphr.h"
#include "System.h"

typedef enum
{
    AcquireFailed = 0,
    AcquireSucceed = 1,
    AcquireStopped = 2,
}PmtEvent;

/********命令表**********/
#define PMT_REG_STATUS  0x00   ///<状态寄存器 8位 只读  0空闲  1忙碌
#define PMT_REG_START   0x01   ///<启动寄存器 8位  只写   写1开始
#define PMT_REG_DATA    0x02   ///<数据寄存器 32位 只读  PMT数据
#define PMT_REG_POWER    0x03   ///<电源寄存器 8位 只写  写1PMT开 写0PMT关
#define PMT_REG_TIME_W  0x05   ///<时间寄存器 16位 只写  单位:毫秒
#define PMT_REG_TIME_R   0x06   ///<时间寄存器 16位 只读  单位:毫秒

#define I2C_SLAVE_ADDR 0x08

//最大定时时间(ms)
#define PMT_CONTROL_MAX_TIME  12000

static Uint32 s_acquireTime = 0;   //ms
static Bool s_isSendEvent = FALSE;
static Bool s_isRequestStop = FALSE;

static PmtResult s_pmtResult = {0,0};
static Bool s_newResult = FALSE;
static AcquiredResult s_resultCode = ACQUIRED_RESULT_FAILED;

static PMT_STATUS s_pmtStatus = PMT_IDLE;
static Bool s_pmtPower = FALSE;

static TickType_t s_startTime;
static TickType_t s_realTime;

static Uint32 s_powerTimes = 0;

static Bool s_isAcquireFinished = FALSE;

static AD5175Driver s_ref_i2c;
static AD5175Driver s_mea_i2c;

/* ----------------------- Task ----------------------------------------*/
TaskHandle_t s_pmtAcquireTask;
static void PMTAcquire_Handle(TaskHandle_t argument);

void PMTControl_InitI2C(void)
{
    s_ref_i2c.pinSCL = GPIO_Pin_14;
    s_ref_i2c.portSCL = GPIOE;
    s_ref_i2c.rccSCL  = RCC_AHB1Periph_GPIOE;
    s_ref_i2c.pinSDA = GPIO_Pin_15;
    s_ref_i2c.portSDA = GPIOE;
    s_ref_i2c.rccSDA  = RCC_AHB1Periph_GPIOE;

    s_mea_i2c.pinSCL = GPIO_Pin_12;
    s_mea_i2c.portSCL = GPIOE;
    s_mea_i2c.rccSCL  = RCC_AHB1Periph_GPIOE;
    s_mea_i2c.pinSDA = GPIO_Pin_13;
    s_mea_i2c.portSDA = GPIOE;
    s_mea_i2c.rccSDA  = RCC_AHB1Periph_GPIOE;
}

Uint16 PMTControl_ReadWord(Uint8 chn)
{
    TRACE_INFO("\nPMT Read chn = %d,", chn);
    if(chn == PMT_CHN_REF)
    {
        return AD5175_PmtReadWord(&s_ref_i2c, I2C_SLAVE_ADDR);
    }
    else if(chn == PMT_CHN_MEA)
    {
        return AD5175_PmtReadWord(&s_mea_i2c, I2C_SLAVE_ADDR);
    }
    else
    {
        TRACE_WARN("Invalid PMT channel");
        return FALSE;
    }
}

Bool PMTControl_WriteWord(Uint8 chn, Uint16 cmd)
{
    TRACE_INFO("\nPMT Write chn = %d, word = 0x%04x", chn, cmd);
    if(chn == PMT_CHN_REF)
    {
        return AD5175_PmtWriteWord(&s_ref_i2c, I2C_SLAVE_ADDR, cmd);
    }
    else if(chn == PMT_CHN_MEA)
    {
        return AD5175_PmtWriteWord(&s_mea_i2c, I2C_SLAVE_ADDR, cmd);
    }
    else
    {
        TRACE_WARN("Invalid PMT channel");
        return FALSE;
    }
}

Uint32 PMTControl_ReadValue(Uint8 chn)
{
    Uint32 value = 0;
    Uint8 buff[4] = {0};

    TRACE_INFO("\nPMT Read chn = %d,", chn);
    if(chn == PMT_CHN_REF)
    {
        AD5175_PmtReadBytes(&s_ref_i2c, I2C_SLAVE_ADDR, buff, 4);
        value = (buff[0]<<24) + (buff[1]<<16) + (buff[2]<<8) + buff[3];
        TRACE_INFO("\nRead d0:0x%02x, d1:0x%02x, d2:0x%02x, d3:0x%02x,  ret:0x%04x", buff[0], buff[1], buff[2], buff[3],  value);
    }
    else if(chn == PMT_CHN_MEA)
    {
        AD5175_PmtReadBytes(&s_mea_i2c, I2C_SLAVE_ADDR, buff, 4);
        value = (buff[0]<<24) + (buff[1]<<16) + (buff[2]<<8) + buff[3];
        TRACE_INFO("\nRead d0:0x%02x, d1:0x%02x, d2:0x%02x, d3:0x%02x,  ret:0x%04x", buff[0], buff[1], buff[2], buff[3],  value);
    }
    else
    {
        TRACE_WARN("Invalid PMT channel");
        return FALSE;
    }

    return value;
}

Bool PMTControl_WriteByte(Uint8 chn, Uint8 cmd)
{
    Uint8 buff[1] = {cmd};

    TRACE_INFO("\nPMT Write chn = %d, byte = 0x%02x", chn, cmd);
    if(chn == PMT_CHN_REF)
    {
        return AD5175_PmtWriteBytes(&s_ref_i2c, I2C_SLAVE_ADDR, buff,  1);
    }
    else if(chn == PMT_CHN_MEA)
    {
        return AD5175_PmtWriteBytes(&s_mea_i2c, I2C_SLAVE_ADDR, buff,  1);
    }
    else
    {
        TRACE_WARN("Invalid PMT channel");
        return FALSE;
    }
}

void PMTControl_Start(void)
{
    PMTControl_StartCount(PMT_CHN_REF);
    PMTControl_StartCount(PMT_CHN_MEA);
}

void PMTControl_ReadResult()
{
    s_pmtResult.refValue = PMTControl_ReadData(PMT_CHN_REF);
    s_pmtResult.meaValue = PMTControl_ReadData(PMT_CHN_MEA);
}

Bool PMTControl_SetTime(Uint8 chn, Uint16 time)
{
    Uint8 cmdData[3] = {0};
    cmdData[0] = PMT_REG_TIME_W;
    cmdData[1] = (time >> 8) & 0xFF;
    cmdData[2] = time & 0xFF;

    if(chn == PMT_CHN_REF)
    {
        return AD5175_PmtWriteBytes(&s_ref_i2c, I2C_SLAVE_ADDR, cmdData, 3);
    }
    else if(chn == PMT_CHN_MEA)
    {
        return AD5175_PmtWriteBytes(&s_mea_i2c, I2C_SLAVE_ADDR, cmdData, 3);
    }
    else
    {
        TRACE_WARN("\nPMT Set Time Invalid Channel");
        return FALSE;
    }
}

Uint16 PMTControl_GetTime(Uint8 chn)
{
    Uint16 value = 0;
    Uint8 retData[2] = {0};
    Uint8 cmdData[1] = {0};
    cmdData[0] = PMT_REG_TIME_R;

    if(chn == PMT_CHN_REF)
    {
        if(AD5175_PmtWriteBytes(&s_ref_i2c, I2C_SLAVE_ADDR, cmdData, 1))
        {
            AD5175_PmtReadBytes(&s_ref_i2c, I2C_SLAVE_ADDR, retData, 4);
            value = (retData[0]<<8) + retData[1];
            TRACE_INFO("\nPMT Read Ref Channel Time = %d(0x%04x)", value, value);
        }
        else
        {
            TRACE_WARN("\nPMT Read Channel %d Fail", chn);
        }
    }
    else if(chn == PMT_CHN_MEA)
    {
        if(AD5175_PmtWriteBytes(&s_mea_i2c, I2C_SLAVE_ADDR, cmdData, 1))
        {
            AD5175_PmtReadBytes(&s_mea_i2c, I2C_SLAVE_ADDR, retData, 4);
            value = (retData[0]<<8) + retData[1];
            TRACE_INFO("\nPMT Read Mea Channel Time = %d(0x%04x)", value, value);
        }
        else
        {
            TRACE_WARN("\nPMT Read Channel %d Fail", chn);
        }
    }
    else
    {
        TRACE_WARN("\nPMT Read Invalid Channel %d", chn);
    }

    return value;
}

Bool PMTControl_SetPower(Uint8 chn, Bool state)
{
    Uint8 cmdData[2] = {0};
    cmdData[0] = PMT_REG_POWER;
    cmdData[1] = state;

    if(chn == PMT_CHN_REF)
    {
        if(AD5175_PmtWriteBytes(&s_ref_i2c, I2C_SLAVE_ADDR, cmdData, 2))
        {
            TRACE_INFO("\nPMT Ref Power  = %d", state);
            return TRUE;
        }
        else
        {
            return FALSE;
        }
    }
    else if(chn == PMT_CHN_MEA)
    {
        if(AD5175_PmtWriteBytes(&s_mea_i2c, I2C_SLAVE_ADDR, cmdData, 2))
        {
            TRACE_INFO("\nPMT Mea Power  = %d", state);
            return TRUE;
        }
        else
        {
            return FALSE;
        }
    }
    else
    {
        TRACE_WARN("\nPMT Start Invalid Channel");
        return FALSE;
    }
}

Bool PMTControl_StartCount(Uint8 chn)
{
    Uint8 cmdData[2] = {0};
    cmdData[0] = PMT_REG_START;
    cmdData[1] = TRUE;

    if(chn == PMT_CHN_REF)
    {
        return AD5175_PmtWriteBytes(&s_ref_i2c, I2C_SLAVE_ADDR, cmdData, 2);
    }
    else if(chn == PMT_CHN_MEA)
    {
        return AD5175_PmtWriteBytes(&s_mea_i2c, I2C_SLAVE_ADDR, cmdData, 2);
    }
    else
    {
        TRACE_WARN("\nPMT Start Invalid Channel");
        return FALSE;
    }
}

Uint32 PMTControl_ReadData(Uint8 chn)
{
    Uint32 value = 0;
    Uint8 retData[4] = {0};
    Uint8 cmdData[1] = {0};
    cmdData[0] = PMT_REG_DATA;


    if(chn == PMT_CHN_REF)
    {
        if(AD5175_PmtWriteBytes(&s_ref_i2c, I2C_SLAVE_ADDR, cmdData, 1))
        {
            AD5175_PmtReadBytes(&s_ref_i2c, I2C_SLAVE_ADDR, retData, 4);
            value = (retData[0]<<24) + (retData[1]<<16) + (retData[2]<<8) + retData[3];
            TRACE_INFO("\nPMT Read Ref Channel Data = %d(0x%08x)", value, value);
        }
        else
        {
            TRACE_WARN("\nPMT Read Channel %d Fail", chn);
        }
    }
    else if(chn == PMT_CHN_MEA)
    {
        if(AD5175_PmtWriteBytes(&s_mea_i2c, I2C_SLAVE_ADDR, cmdData, 1))
        {
            AD5175_PmtReadBytes(&s_mea_i2c, I2C_SLAVE_ADDR, retData, 4);
            value = (retData[0]<<24) + (retData[1]<<16) + (retData[2]<<8) + retData[3];
            TRACE_INFO("\nPMT Read Mea Channel Data = %d(0x%08x)", value, value);
        }
        else
        {
            TRACE_WARN("\nPMT Read Channel %d Fail", chn);
        }
    }
    else
    {
        TRACE_WARN("\nPMT Read Invalid Channel %d", chn);
    }

    return value;
}

/**
 * @brief PMT控制器初始化
 */
void PMTControl_Init(void)
{
    PMTControl_InitI2C();
    PMTDriver_Init();

    s_isAcquireFinished = FALSE;
    s_pmtPower = TRUE;

    xTaskCreate(PMTAcquire_Handle, "PMTAcquire", PMT_ACQUIRE_STK_SIZE, NULL, PMT_ACQUIRE_TASK_PRIO, &s_pmtAcquireTask);

    s_pmtStatus = PMT_IDLE;
}


/**
 * @brief PMT信号采集
 * @param[in] timeout 超时时间, 单位 ms
 */
Bool PMTControl_StartAcquire(Uint32 timeout, Bool isSendEvent)
{
    if(s_pmtStatus == PMT_IDLE)
    {
        if(timeout > PMT_CONTROL_MAX_TIME)
        {
            TRACE_ERROR("\nPMT Once Max Acquire Time is %d ms", PMT_CONTROL_MAX_TIME);
            return FALSE;
        }
        else if(timeout <= 0)
        {
            TRACE_ERROR("\nAcquire Time %d ms is invalid", timeout);
            return FALSE;
        }

        s_acquireTime = timeout;
        s_isSendEvent = isSendEvent;
        s_isRequestStop = FALSE;
        s_resultCode = ACQUIRED_RESULT_FAILED;
        s_pmtResult.refValue = 0;
        s_pmtResult.meaValue = 0;
        s_newResult = FALSE;

        s_isAcquireFinished = FALSE;

        TRACE_ERROR("\nPMT Acquire Start total time = %d ms", timeout);

        s_pmtStatus = PMT_BUSY;
        s_startTime = xTaskGetTickCount();
        vTaskResume(s_pmtAcquireTask);

        taskENTER_CRITICAL();

        //PMT板开始采集
        PMTControl_Start();

        taskEXIT_CRITICAL();

        return TRUE;
    }
    else
    {
        TRACE_ERROR("\nPMT Control is acquiring");
        return FALSE;
    }
}

/**
 * @brief 停止信号采集
 */
void PMTControl_StopAcquire(Bool isSendEvent)
{
    s_isSendEvent = isSendEvent;
    s_isRequestStop = TRUE;
    s_pmtResult.refValue = 0;
    s_pmtResult.meaValue = 0;

    s_acquireTime = 1000;
    s_resultCode = ACQUIRED_RESULT_STOPPED;

    s_isAcquireFinished = TRUE;

    TRACE_INFO("\nPMT Acquire Stop");
}

/**
 * @brief 获取最新PMT采集数据
 */
Bool PMTControl_GetNewResult(PmtResult* result)
{
    if(s_newResult == TRUE)
    {
        result->refValue = s_pmtResult.refValue;
        result->meaValue = s_pmtResult.meaValue;
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/**
 * @brief PMT是否空闲
 */
Uint8 PMTControl_GetStatus(void)
{
    return (Uint8)s_pmtStatus;
}

/**
 * @brief PMT探头电源开启
 */
void PMTControl_PowerOn()
{
    PMTControl_SetPower(PMT_CHN_REF, TRUE);
    PMTControl_SetPower(PMT_CHN_MEA, TRUE);
    s_pmtPower = TRUE;
}

/**
 * @brief PMT探头电源关闭
 */
void PMTControl_PowerOff()
{
    PMTControl_SetPower(PMT_CHN_REF, FALSE);
    PMTControl_SetPower(PMT_CHN_MEA, FALSE);
    s_pmtPower = FALSE;
}

/**
 * @brief PMT电源状态
 */
Bool PMTControl_PowerStatus()
{
    return s_pmtPower;
}

/**
  * @brief PMT测试
  */
void PMTAcquire_Handle(TaskHandle_t argument)
{
    vTaskSuspend(NULL);
    while(1)
    {
        switch(s_pmtStatus)
        {
            case PMT_IDLE:
                s_startTime = xTaskGetTickCount();
                break;

            case PMT_BUSY:
                s_realTime = xTaskGetTickCount() - s_startTime;

                if(s_realTime >= 1000 + s_acquireTime*portTICK_RATE_MS)  //
                {
                    if(FALSE == s_isRequestStop)
                    {
                        s_isAcquireFinished = TRUE;
                        s_resultCode = ACQUIRED_RESULT_FINISHED;
                    }

                    if ( TRUE == s_isAcquireFinished)
                    {
                        TRACE_INFO("\nGet PMT RESULT CODE = %d", s_resultCode);

                        s_isAcquireFinished = FALSE;

                        PMTControl_ReadResult();

                        TRACE_INFO("\nPMT Acuire Result: Ref = %d, Mea = %d, Acquire Time = %d ms, Real Time = %d ms", s_pmtResult.refValue, s_pmtResult.meaValue, s_acquireTime, s_realTime);

                        s_newResult = TRUE;

                        if (TRUE == s_isSendEvent)
                        {
                            Uint8 data[9] = {0};
                            memcpy(data, &s_pmtResult, sizeof(s_pmtResult));
                            data[8] = s_resultCode;
                            DncpStack_SendEvent(DSCP_EVENT_OAI_AD_ACQUIRED, (void *)data, sizeof(data));
                            DncpStack_BufferEvent(DSCP_EVENT_OAI_AD_ACQUIRED, (void *)data, sizeof(data));
                        }

                        s_isRequestStop = FALSE;
                        s_isSendEvent = FALSE;
                        s_pmtStatus = PMT_IDLE;
                    }
                }
                break;

            default:
                break;
        }
        vTaskDelay(10);
    }
}
