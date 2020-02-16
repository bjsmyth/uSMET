/*
 * config.h
 *
 *  Created on: Jan 29, 2020
 *      Author: Brandon Smyth
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "Board.h"

/* Task Stack Sizes */
#define IMUPROCESS_STACK (512)
#define DATALOG_STACK (4096)
#define PWMCTRL_STACK (512)
#define FLUSH_STACK (512)

#define VESCPROCESS_STACK (1024)
#define VESCTIMEOUT_STACK (384)

/* Task Priority (Higher Number == Higher Priority) */
#define IMUPROCESS_PRIORITY (15)
#define DATALOG_PRIORITY (3)
#define PWMCTRL_PRIORITY (5)
#define FLUSH_PRIORITY (2)

#define VESCPROCESS_PRIORITY (1)
#define VESCTIMEOUT_PRIORITY (1)

/* UART peripherals */

typedef enum VESC_UARTName {
    VESC_UART_DRIVE = 0,
    VESC_UART_COUNT
} VESC_UARTName;
#define VESC_UART_DRV Board_UART7
#define COMMUNICATION_UART Board_UART3

#endif /* CONFIG_H_ */
