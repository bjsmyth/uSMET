/*
 * Copyright (c) 2015, Texas Instruments Incorporated
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

#include <stdio.h>
#include <string.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/PWM.h>

#include <ti/mw/fatfs/ff.h>

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>

#include <sensorlib/i2cm_drv.h>
#include <sensorlib/ak8975.h>
#include <sensorlib/mpu9150.h>


/* Board Header files */
#include "Board.h"

#include "MPU9150.h"
#include "six_axis_comp_filter.h"

#define TASKSTACKSIZE       4096


Task_Struct task0Struct;
Char task0Stack[512];

Task_Struct task1Struct;
Char task1Stack[TASKSTACKSIZE];

Task_Struct task2Struct;
Char task2Stack[512];

static MPU9150_Handle mpu;
static SixAxis compFilter;

void gpioMPU9150DataReady(unsigned int index) {
    GPIO_clearInt(MPU9150_INT_PIN);
    Semaphore_post(imuDataReady);

}

/*
 *  ======== taskFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void taskFxn(UArg arg0, UArg arg1)
{
    MPU9150_Data accel, gyro;
    mpu = MPU9150_init(0, Board_I2C0, 0x68);
    if(!mpu) {
        GPIO_write(Board_LED2, Board_LED_ON);
        System_abort("Could not init IMU");
    }
    Task_sleep(100); //Wait for IMU to be ready

    CompInit(&compFilter, 0.05f, 0.5f);

    //Sample once
    if(!MPU9150_read(mpu)) {
        GPIO_write(Board_LED2, Board_LED_ON);
        System_abort("Could not extract data registers");
    }

    MPU9150_getAccelFloat(mpu, &accel);

    CompAccelUpdate(&compFilter, accel.xFloat, accel.yFloat, accel.zFloat);
    CompStart(&compFilter);

    for (;;) {
        Task_sleep(50);
        //Semaphore_pend(imuDataReady, BIOS_WAIT_FOREVER);
        if(!MPU9150_read(mpu)) {
            GPIO_write(Board_LED2, Board_LED_ON);
            //System_abort("Could not extract data registers");
        }

        MPU9150_getGyroFloat(mpu, &gyro);
        MPU9150_getAccelFloat(mpu, &accel);

        CompGyroUpdate(&compFilter, gyro.xFloat, gyro.yFloat, gyro.zFloat);
        CompAccelUpdate(&compFilter, accel.xFloat, accel.yFloat, accel.zFloat);
        CompUpdate(&compFilter);
    }
}

Void printFxn(UArg arg0, UArg arg1) {

    char outputfile[20];
    SDSPI_Handle sdspiHandle;
    SDSPI_Params sdspiParams;

    FIL src;
    unsigned int bytesWritten = 0;

    SDSPI_Params_init(&sdspiParams);
    sdspiHandle = SDSPI_open(Board_SDSPI0, 0, &sdspiParams);
    if (sdspiHandle == NULL) {
        System_abort("Error starting the SD card\n");
    }
    else {
        System_printf("Drive 0 is mounted\n");
    }
    unsigned int fileNum = 0;
    FRESULT res;
    do {
        sprintf(outputfile, "0:log_%u.csv", fileNum++);
        res = f_open(&src, outputfile, FA_CREATE_NEW | FA_WRITE);
        if(res == FR_NOT_READY) {
            GPIO_write(Board_LED2, Board_LED_ON);
            System_abort("SD Card reported not ready, try reseating card");
        }
    }while(res != FR_OK);

    System_printf("Successfully opened log file: %s\n", outputfile);
    System_flush();
    const char csvStr[] = "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,roll\n";
    f_write(&src, csvStr, strlen(csvStr), &bytesWritten);

    MPU9150_Data accel, gyro;

    char accelStr[100];
    uint32_t length;
    float roll, rollOffset;
    Task_sleep(5000);

    CompAnglesGet(&compFilter, NULL, &rollOffset); //Offset
    GPIO_write(Board_LED0, Board_LED_OFF);
    GPIO_write(Board_LED1, Board_LED_ON);
    for(;;) {
        MPU9150_getGyroFloat(mpu, &gyro);
        MPU9150_getAccelFloat(mpu, &accel);
        CompAnglesGet(&compFilter, NULL, &roll);
        roll -= rollOffset;
        roll = CompRadiansToDegrees(roll);
        if(roll >= 180.0f) {
            roll -= 360.0f;
        }

        length = sprintf(accelStr, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                         accel.xFloat, accel.yFloat, accel.zFloat,
                         gyro.xFloat, gyro.yFloat, gyro.zFloat, roll);

        System_printf("%s", accelStr);
        System_flush();

        f_write(&src, accelStr, length, &bytesWritten);
        f_sync(&src);
        Task_sleep(100);
    }
}

void pwmLEDFxn(UArg arg0, UArg arg1)
{
    PWM_Handle pwm1;
    PWM_Handle pwm2 = NULL;
    PWM_Params params;
    uint16_t   pwmPeriod = 3000;      // Period and duty in microseconds
    uint16_t   duty = 0;
    uint16_t   dutyInc = 100;

    PWM_Params_init(&params);
    params.period = pwmPeriod;
    pwm1 = PWM_open(Board_PWM0, &params);
    if (pwm1 == NULL) {
        System_abort("Board_PWM0 did not open");
    }

    if (Board_PWM1 != Board_PWM0) {
        params.polarity = PWM_POL_ACTIVE_LOW;
        pwm2 = PWM_open(Board_PWM1, &params);
        if (pwm2 == NULL) {
            System_abort("Board_PWM1 did not open");
        }
    }

    /* Loop forever incrementing the PWM duty */
    while (1) {
        PWM_setDuty(pwm1, duty);

        if (pwm2) {
            PWM_setDuty(pwm2, duty);
        }

        duty = (duty + dutyInc);
        if (duty == pwmPeriod || (!duty)) {
            dutyInc = - dutyInc;
        }

        Task_sleep((UInt) 100);
    }
}


/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams, task1, task2;

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initSDSPI();
    Board_initI2C();
    Board_initPWM();



    /* Construct MPU9150 Task thread */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 512;
    taskParams.stack = &task0Stack;
    taskParams.priority = 15; //Highest priority, must read on time
    Task_construct(&task0Struct, (Task_FuncPtr)taskFxn, &taskParams, NULL);

    Task_Params_init(&task1);
    task1.stackSize = TASKSTACKSIZE;
    task1.stack = &task1Stack;
    task1.priority = 1;
    Task_construct(&task1Struct, (Task_FuncPtr)printFxn, &task1, NULL);

    Task_Params_init(&task2);
    task1.stackSize = 512;
    task1.stack = &task2Stack;
    task1.priority = 2;
    Task_construct(&task2Struct, (Task_FuncPtr)pwmLEDFxn, &task2, NULL);

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);
    GPIO_setCallback(MPU9150_INT_PIN, gpioMPU9150DataReady);
    GPIO_enableInt(MPU9150_INT_PIN);

    System_printf("Starting the I2C example\nSystem provider is set to SysMin."
                  " Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
