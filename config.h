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
#define VESCTIMEOUT_STACK (512)

/* Task Priority (Higher Number == Higher Priority) */
#define IMUPROCESS_PRIORITY (15)
#define DATALOG_PRIORITY (5)
#define PWMCTRL_PRIORITY (3)

#define VESCPROCESS_PRIORITY (1)
#define VESCTIMEOUT_PRIORITY (1)

/* UART peripherals */

typedef enum VESC_UARTName {
    VESC_UART_DRIVE = 0,
    VESC_UART_COUNT
} VESC_UARTName;
#define VESC_UART_DRV Board_UART3

#define COMMUNICATION_UART Board_UART0
#define COMMUNICATION_UART_BAUD (1000000)

#define BLUETOOTH_UART Board_UART7
#define BLUETOOTH_UART_BAUD (115200)

#endif /* CONFIG_H_ */
