/*
 * config.h
 *
 *  Created on: Jan 29, 2020
 *      Author: Brandon Smyth
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "Board.h"

/* Control Params */

//#define STEERING_INVERT //Define if steering motor spins the wrong way for control loop
#define STEERING_KP (55.0f)
#define STEERING_KI (20.4f)
#define STEERING_ANGLE_SENSE_RANGE (250.0f) //Degrees
#define STEERING_INPUT_ANGLE_CLAMP (90.0f) //Degrees
#define STEERING_RPM_CLAMP (6000.0f)

/* Task Stack Sizes */
#define IMUPROCESS_STACK (512)
#define DATALOG_STACK (4096)

#define VESCPROCESS_STACK (1024)
#define VESCTIMEOUT_STACK (512)

#define STEERING_STACK (512)

/* Task Priority (Higher Number == Higher Priority) */
#define IMUPROCESS_PRIORITY (14)
#define STEERING_PRIORITY (15)
#define DATALOG_PRIORITY (5)
#define PWMCTRL_PRIORITY (3)

#define VESCPROCESS_PRIORITY (1)
#define VESCTIMEOUT_PRIORITY (1)

/* UART peripherals */

typedef enum VESC_UARTName {
    VESC_UART_STEERING = 0,
    VESC_UART_REARTRACK,
    VESC_UART_THROTTLE_REARLEFT,
    VESC_UART_THROTTLE_REARRIGHT,
    VESC_UART_THROTTLE_FRONT,
    VESC_UART_COUNT
} VESC_UARTName;
#define VESC_UART_STEERING_DRV Board_UART3
#define VESC_UART_REARTRACK_DRV Board_UART7

#define COMMUNICATION_UART Board_UART0
#define COMMUNICATION_UART_BAUD (1000000)

#endif /* CONFIG_H_ */
