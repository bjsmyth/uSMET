/*
 * steering.c
 *
 *  Created on: Mar 5, 2020
 *      Author: brandon
 */

#include "steering.h"

#include <math.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Timer.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Clock.h>

#include <driverlib/adc.h>
#include <inc/hw_memmap.h>

#include "utilities.h"
#include "config.h"
#include "vesc_uart/bldc_interface.h"


static Task_Struct steering_pi_task_struct;
static uint8_t steering_pi_task_stack[STEERING_STACK];

extern Semaphore_Handle steerBlk;

static struct {
    float current_steerAngle;
    float set_steerAngle;
    int32_t rpm;
    GateMutex_Handle dataAccess;
} steer_set;

void steer_setAngle(float angle)
{
    //Clamp input
    if(angle > STEERING_INPUT_ANGLE_CLAMP) {
        angle = STEERING_INPUT_ANGLE_CLAMP;
    } else if(angle < -STEERING_INPUT_ANGLE_CLAMP) {
        angle = -STEERING_INPUT_ANGLE_CLAMP;
    }

    uint32_t key = GateMutex_enter(steer_set.dataAccess);

    steer_set.set_steerAngle = angle;

    GateMutex_leave(steer_set.dataAccess, key);
}

static float steer_getSetAngle()
{
    float ret;
    uint32_t key = GateMutex_enter(steer_set.dataAccess);

    ret = steer_set.set_steerAngle;

    GateMutex_leave(steer_set.dataAccess, key);

    return ret;
}

float steer_getCurrentAngle()
{
    float ret;
    uint32_t key = GateMutex_enter(steer_set.dataAccess);

    ret = steer_set.current_steerAngle;

    GateMutex_leave(steer_set.dataAccess, key);

    return ret;
}

static void steer_setCurrentAngle(float angle)
{
    uint32_t key = GateMutex_enter(steer_set.dataAccess);

    steer_set.current_steerAngle = angle;

    GateMutex_leave(steer_set.dataAccess, key);
}

static void steer_setControlRpm(int32_t rpm)
{
    steer_set.rpm = rpm;
}

int32_t steer_getControlRpm()
{
    return steer_set.rpm;
}

static uint32_t steer_GetADC()
{
    uint32_t ret;
    ADCProcessorTrigger(ADC0_BASE, 0);

    while(!ADCIntStatus(ADC0_BASE, 0, false));

    ADCSequenceDataGet(ADC0_BASE, 0, &ret);

    return ret;
}

static inline float steer_mapADCAngle(uint32_t adc)
{
    return map_float((float)adc, 0.0f, 4096.0f, \
                     -STEERING_ANGLE_SENSE_RANGE * 0.5f, STEERING_ANGLE_SENSE_RANGE * 0.5f);
}

static int32_t pi_controller_rpm(float currentAngle)
{
    static float prevError = 0, output = 0;

    float currError = steer_getSetAngle() - currentAngle;

    currError = fabs(currError) < 1.0f ? 0 : currError;

    output += (STEERING_KP + STEERING_KI * (STEERING_DT * 0.001f)) * currError
             - STEERING_KP * prevError;

    prevError = currError;

    // Clamp output
    output = output > STEERING_RPM_CLAMP ? STEERING_RPM_CLAMP : output;
    output = output < -STEERING_RPM_CLAMP ? -STEERING_RPM_CLAMP : output;

#ifdef STEERING_INVERT
    return (int32_t)-output;
#else
    return (int32_t)output;
#endif
}

static void steering_pi_proc()
{
    uint32_t adcVal;
    int32_t rpm;
    float steer_currentAngle;

    { //Allow ADC to settle
        int i;
        for(i = 0; i < 10; i++) {
            adcVal = steer_GetADC();
            steer_currentAngle = steer_mapADCAngle(adcVal);
        }
    }

    uint32_t startTick, endTick;
    Task_sleep(1000);
    for(;;) {
        startTick = Clock_getTicks();

        adcVal = steer_GetADC();

        steer_currentAngle = steer_mapADCAngle(adcVal);
        steer_setCurrentAngle(steer_currentAngle);

        rpm = pi_controller_rpm(steer_currentAngle);

        bldc_interface_set_rpm(VESC_UART_STEERING, rpm);

        steer_setControlRpm(rpm);

        endTick = Clock_getTicks();
        int sleepTime = STEERING_DT - (endTick - startTick);
        if(sleepTime < 0) {
            sleepTime = 0;
        }
        Task_sleep(sleepTime);
    }
}


void steering_init()
{
    Task_Params steeringParams;

    Task_Params_init(&steeringParams);
    steeringParams.stackSize = STEERING_STACK;
    steeringParams.stack = &steering_pi_task_stack;
    steeringParams.priority = STEERING_PRIORITY;
    Task_construct(&steering_pi_task_struct, (Task_FuncPtr)steering_pi_proc, &steeringParams, NULL);

    Error_Block eb;

    Error_init(&eb);

    steer_set.current_steerAngle = 0;
    steer_set.set_steerAngle = 0;
    steer_set.dataAccess = GateMutex_create(NULL, &eb);
}
