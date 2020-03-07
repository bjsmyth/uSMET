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
#include <math.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/gates/GateMutex.h>

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

#include <driverlib/adc.h>


/* Board Header files */
#include "Board.h"
#include "config.h"
#include "utilities.h"

#include "IMU/MPU9150.h"
#include "IMU/six_axis_comp_filter.h"
#include "IMU/kalman.h"

#include "vesc_uart/comm_uart.h"
#include "vesc_uart/bldc_interface.h"
#include "vesc_uart/datatypes.h"

#include "communication/communication.h"
#include "steering/steering.h"


Task_Struct task0Struct;
uint8_t task0Stack[IMUPROCESS_STACK];

Task_Struct task1Struct;
uint8_t task1Stack[DATALOG_STACK];

Task_Struct task2Struct;
uint8_t task2Stack[PWMCTRL_STACK];

static MPU9150_Handle mpu;
Kalman kalmanRoll, kalmanPitch;


Void imuProc(UArg arg0, UArg arg1)
{
    MPU9150_Data accel, gyro;
    mpu = MPU9150_init(0, Board_I2C0, 0x68);
    if(!mpu) {
        GPIO_write(Board_LED2, Board_LED_ON);
        System_abort("Could not init IMU");
    }
    Task_sleep(100); //Wait for IMU to be ready

    Kalman_init(&kalmanRoll);
    Kalman_init(&kalmanPitch);

    //Sample once
    if(!MPU9150_read(mpu)) {
        GPIO_write(Board_LED2, Board_LED_ON);
        System_abort("Could not extract data registers");
    }

    MPU9150_getAccelFloat(mpu, &accel);

    float roll = atan2f(accel.yFloat, accel.zFloat);
    roll = CompRadiansToDegrees(roll);
    float pitch = atanf(-accel.xFloat / sqrtf(accel.yFloat * accel.yFloat + accel.zFloat * accel.zFloat));
    pitch = CompRadiansToDegrees(pitch);
    Kalman_setAngle(&kalmanRoll, roll);
    Kalman_setAngle(&kalmanPitch, pitch);

    float prevRoll;
    for (;;) {
        Task_sleep(25);
        //Semaphore_pend(imuDataReady, BIOS_WAIT_FOREVER);
        if(!MPU9150_read(mpu)) {
            GPIO_write(Board_LED2, Board_LED_ON);
            //System_abort("Could not extract data registers");
        }

        MPU9150_getGyroFloat(mpu, &gyro);
        MPU9150_getAccelFloat(mpu, &accel);

        //Kalman
        float roll = atan2f(accel.yFloat, accel.zFloat);
        roll = RadiansToDegrees(roll);
        float pitch = atanf(-accel.xFloat / sqrtf(accel.yFloat * accel.yFloat + accel.zFloat * accel.zFloat));
        pitch = RadiansToDegrees(pitch);

        if((roll < -90.0f && prevRoll > 90.0f) || (roll > 90.0f && prevRoll < -90.0f)) {
            Kalman_setAngle(&kalmanRoll, roll);
            prevRoll = roll;
        } else {
            float gyroXDeg = RadiansToDegrees(gyro.xFloat);
            prevRoll = Kalman_updateFilter(&kalmanRoll, roll, gyroXDeg, 0.05f);
        }

        if(fabs(prevRoll) > 90.0f) {
            gyro.yFloat = -gyro.yFloat;
        }

        float gyroYDeg = RadiansToDegrees(gyro.yFloat);
        Kalman_updateFilter(&kalmanPitch, pitch, gyroYDeg, 0.05f);
    }
}

Void printFxn(UArg arg0, UArg arg1) {

    bool vescMboxRes;
    mc_values vescFeedback = { 0 };

    MPU9150_Data accel, gyro;

    tx_packet txPackt = { 0 };
    rx_packet rxPackt;
    uint32_t startTick, endTick;

    Task_sleep(1000);

    //char logStr[128];
    float roll, rollOffset;
    GPIO_write(Board_LED0, Board_LED_OFF);
    GPIO_write(Board_LED1, Board_LED_ON);

    rollOffset = Kalman_getAngle(&kalmanRoll);

    int32_t startTimer = Clock_getTicks();

    for(;;) {
        startTick = Clock_getTicks();

        MPU9150_getGyroFloat(mpu, &gyro);
        MPU9150_getAccelFloat(mpu, &accel);

        vescMboxRes = Mailbox_pend(vescTelemetry, &vescFeedback, BIOS_NO_WAIT);
        if(vescMboxRes) {
            vescFeedback.rpm *= (1.0f / 7.0f);
        }

        roll = Kalman_getAngle(&kalmanRoll);
        roll -= rollOffset;

        txPackt.steering_current_pos = steer_getCurrentAngle();
        txPackt.steering_motor_rpm = steer_getControlRpm();
        txPackt.vehicle_roll = roll;
        txPackt.vehicle_speed = vescFeedback.rpm;
        txPackt.accel = accel;
        txPackt.gyro = gyro;
        rxPackt = communication_uart_send(txPackt);

        steer_setAngle(rxPackt.steering_set_pos);
        //bldc_interface_set_rpm_true(VESC_UART_DRIVE, 1);
        bldc_interface_get_values(VESC_UART_STEERING);

        /*int32_t currentTime = Clock_getTicks();
        int rpm;
        if(30000 - currentTime - startTimer < 0) {
            if(31000 - currentTime - startTimer > 0) {
                rpm = -1500;
                test.flags = 1;

            } else {
                rpm = 0;
                test.flags = 0;
                if(31500 - currentTime - startTimer > 0) {
                    test.flags = 1;
                }
            }

        } else if(29500 - currentTime - startTimer < 0) {
            test.flags = 1;
        }

        else {
            rpm = 0;
            test.flags = 0;
        }*/
        //bldc_interface_set_duty_cycle(VESC_UART_DRIVE, duty);
        //System_printf("%d %f\n", remVal, duty);
        //System_flush();

        endTick = Clock_getTicks();
        int sleepTime = 67 - (endTick - startTick);
        if(sleepTime < 0) {
            sleepTime = 0;
        }
        Task_sleep(sleepTime);
    }
}

void pwmCtrlFxn(UArg arg0, UArg arg1)
{
    PWM_Handle pwm1;
    PWM_Params params;
    uint16_t   pwmPeriod = 50000;      // Period and duty in microseconds
    uint16_t   duty = 1500;
    int16_t   dutyInc = 5;

    PWM_Params_init(&params);
    params.period = pwmPeriod;
    pwm1 = PWM_open(Board_PWM0, &params);
    if (pwm1 == NULL) {
        System_abort("Board_PWM0 did not open");
    }

    /* Loop forever incrementing the PWM duty */
    while (1) {
        if(duty == 1500) {
            //Task_sleep(2500);
        }
        PWM_setDuty(pwm1, duty);

        duty += dutyInc;
        if (duty == 2000 || duty == 1000) {
            dutyInc = - dutyInc;
            //Task_sleep(10000);
        }

        Task_sleep(100);
    }
}

void Init_tasks()
{
    Task_Params taskParams, dataLogging, pwmTask;
    /* Construct MPU9150 Task thread */
    Task_Params_init(&taskParams);
    taskParams.stackSize = IMUPROCESS_STACK;
    taskParams.stack = &task0Stack;
    taskParams.priority = IMUPROCESS_PRIORITY; //Highest priority, must read on time
    Task_construct(&task0Struct, (Task_FuncPtr)imuProc, &taskParams, NULL);

    Task_Params_init(&dataLogging);
    dataLogging.stackSize = DATALOG_STACK;
    dataLogging.stack = &task1Stack;
    dataLogging.priority = DATALOG_PRIORITY;
    Task_construct(&task1Struct, (Task_FuncPtr)printFxn, &dataLogging, NULL);

    /*Task_Params_init(&pwmTask);
    pwmTask.stackSize = PWMCTRL_STACK;
    pwmTask.stack = &task2Stack;
    pwmTask.priority = PWMCTRL_PRIORITY;
    Task_construct(&task2Struct, (Task_FuncPtr)pwmCtrlFxn, &pwmTask, NULL);*/
}


/*
 *  ======== main ========
 */
int main(void)
{

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    //Board_initSDSPI();
    Board_initI2C();
    //Board_initPWM();
    Board_initUART();
    Board_initADC();


    Init_tasks();
    comm_uart_init();
    communication_uart_init();
    steering_init();
    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    //USBCDCD_init();

    System_printf("Starting uSMET software\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return 0;
}
