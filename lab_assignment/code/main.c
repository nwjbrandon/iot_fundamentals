/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2ctmp007.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/I2C.h>


/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>


/* Example/Board Header files */
#include "Board.h"
#include "SensorOpt3001.h"
#include "SensorHdc1000.h"
#include "SensorUtil.h"
#include "SensorI2C.h"
#include "SensorMpu9250.h"
#define TASKSTACKSIZE       2048
#define TMP007_OBJ_TEMP     0x0003  /* Object Temp Result Register */


/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config ledPinTable[] = {
Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                             Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW
                                     | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                             PIN_TERMINATE };




/*
 *  ======== echoFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
#define HDC1000_REG_TEMP           0x00 // Temperature
#define HDC1000_REG_HUM            0x01 // Humidity
#define HDC1000_REG_CONFIG         0x02 // Configuration
#define HDC1000_REG_SERID_H        0xFB // Serial ID high
#define HDC1000_REG_SERID_M        0xFC // Serial ID middle
#define HDC1000_REG_SERID_L        0xFD // Serial ID low
#define HDC1000_REG_MANF_ID        0xFE // Manufacturer ID
#define HDC1000_REG_DEV_ID         0xFF // Device ID

// Fixed values
#define HDC1000_VAL_MANF_ID        0x5449
#define HDC1000_VAL_DEV_ID         0x1000
#define HDC1000_VAL_CONFIG         0x1000 // 14 bit, acquired in sequence
#define SENSOR_DESELECT()   SensorI2C_deselect()


//Task_Struct tsk0Struct;
//Task_Handle task;
//Char task0Stack[TASKSTACKSIZE];

Task_Struct tsk0Struct;
UInt8 tsk0Stack[TASKSTACKSIZE];
Task_Handle task;

/* Initializers for debugging */
Task_Struct tDebugStruct;
UInt8 tDebugStack[TASKSTACKSIZE];
Task_Handle tDebugTask;

/* Initializers for PWM */
Task_Struct tPwmStruct;
UInt8 tPwmStack[TASKSTACKSIZE];
Task_Handle tPwmTask;

/* Initializers for OPT */
Task_Struct tOptStruct;
UInt8 tOptStack[TASKSTACKSIZE];
Task_Handle tOptTask;

/* Initializers for MPU */
Task_Struct tMpuStruct;
UInt8 tMpuStack[TASKSTACKSIZE];
Task_Handle tMpuTask;

volatile float mpuDc = 0;
volatile float optDc = 0;

void readOPT3001(UArg arg0, UArg arg1)
{
    SensorOpt3001_init();
    SensorOpt3001_enable(true);
    uint16_t rawData = 0;
    while (!SensorOpt3001_test()) {
        System_printf("SensorOpt3001 did not pass test!\n");
        System_flush();
    }
    while(1){
        if (SensorOpt3001_read(&rawData)) {
            float data = SensorOpt3001_convert(rawData);
            float tmp = (data-100) / 1000.0;
            if (tmp<0) {
                optDc = 0;
            } else if (tmp>1) {
                optDc = 1;
            } else {
                optDc = tmp;
            }
            System_printf("SensorOpt3001 Raw: %d Converted: %f Resize: %f OptDc %f \n", rawData, data, tmp, optDc);
            System_flush();
        } else {
            System_printf("SensorOpt3001 I2C fault!\n");
            System_flush();
        }
        Task_sleep((UInt) 10000);
    }
}

void readMpu9250()
{
    uint16_t rawData = 0;
    SensorMpu9250_powerOn();
    while (!SensorMpu9250_init()) {
        System_printf("SensorMPU9250_ cannot init!\n");
        System_flush();
    }
    SensorMpu9250_accSetRange(ACC_RANGE_2G);
    SensorMpu9250_enable(9);
    SensorMpu9250_enableWom(1);
    while (!SensorMpu9250_test()) {
        System_printf("SensorMPU9250_ did not pass test!\n");
        System_flush();
    }
    while (1) {
        if (SensorMpu9250_accRead(&rawData)) {
            float data = SensorMpu9250_accConvert(rawData);
            float tmp = data>0 ? data : -data;
            tmp = (tmp-0.34)/0.2;
            if (tmp<0) {
                mpuDc=0;
            } else if (tmp>1){
                mpuDc=1;
            } else {
                mpuDc=tmp;
            }
            System_printf("readMpu9250 Raw: %d Converted: %f Resize: %f MpuDc: %f \n", rawData, data, tmp, mpuDc);
            System_flush();
        } else {
            System_printf("SensorMPU9250_ I2C fault!\n");
            System_flush();
        }
        Task_sleep((UInt) 10000);
    }
}

void PWMLED(UArg arg0, UArg arg1)
{
    PWM_Handle pwm1;
    PWM_Params params;
    uint16_t   pwmPeriod = 3000;      // Period and duty in microseconds

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;
    pwm1 = PWM_open(Board_PWM0, &params);

    if (pwm1 == NULL) {
        System_abort("Board_PWM0 did not open");
    }
    PWM_start(pwm1);
    while (1) {
        float dc = mpuDc > optDc ? mpuDc : optDc;
        System_printf("optDc: %f mpuDc: %f dc: %f \n", optDc, mpuDc, dc);
        System_flush();
        PWM_setDuty(pwm1, dc * pwmPeriod);
        Task_sleep((UInt) 10000);
    }
}

int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();
    Board_initGPIO();
    Board_initPWM();
    SensorI2C_open();

    /* Initialize Light Sensor */
    Task_Params tOptParams;
    Task_Params_init(&tOptParams);
    tOptParams.stackSize = TASKSTACKSIZE;
    tOptParams.stack = &tOptStack;
    tOptParams.arg0 = 50;
    Task_construct(&tOptStruct, (Task_FuncPtr)readOPT3001, &tOptParams, NULL);
    tOptTask = Task_handle(&tOptStruct);

    /* Construct MPU Task thread */
    Task_Params tMpuParams;
    Task_Params_init(&tMpuParams);
    tMpuParams.stackSize = TASKSTACKSIZE;
    tMpuParams.stack = &tMpuStack;
    tMpuParams.arg0 = 50;
    Task_construct(&tMpuStruct, (Task_FuncPtr)readMpu9250, &tMpuParams, NULL);
    tMpuTask = Task_handle(&tMpuStruct);

    /* Construct LED Task thread */
    Task_Params tPwmParams;
    Task_Params_init(&tPwmParams);
    tPwmParams.stackSize = TASKSTACKSIZE;
    tPwmParams.stack = &tPwmStack;
    tPwmParams.arg0 = 50;
    Task_construct(&tPwmStruct, (Task_FuncPtr)PWMLED, &tPwmParams, NULL);
    tPwmTask = Task_handle(&tPwmStruct);

    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the I2C example\nSystem provider is set to SysMin."
                  " Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
