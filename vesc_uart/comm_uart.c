/*
	Copyright 2015 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * comm_uart.c
 *
 *  Created on: 17 aug 2015
 *      Author: benjamin
 */

#include "../config.h"

#include "comm_uart.h"
#include "bldc_interface_uart.h"
#include "bldc_interface.h"
#include "Board.h"

#include <string.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>


#include <ti/drivers/UART.h>
#include <string.h>

static UART_Handle uart;
static Mailbox_Handle uartRxReady;
static Mailbox_Struct uartRxReadyStct;

// Settings

// Private functions
static void send_packet(unsigned int uart_index, unsigned char *data, unsigned int len);

// Threads
static Task_Struct periodicRefresh;
static uint8_t periodicRefreshStack[VESCTIMEOUT_STACK];

static Task_Struct uartProcess;
static uint8_t uartProcessStack[VESCPROCESS_STACK];

static void uartCallbackFxn(UART_Handle handle, void *buf, size_t count)
{
    unsigned char *buffer = buf;
    unsigned int i;
    for(i = 0; i < count; i++) {
        Mailbox_post(uartRxReady, &buffer[i], BIOS_NO_WAIT);
    }
}

static void periodicUpdate(UArg arg0, UArg arg1) {
	for(;;) {
	    bldc_interface_uart_run_timer();
        Task_sleep(1);
	}
}

static void uartProcessFxn(UArg arg0, UArg arg1) {
    uint8_t uartValue;
    for(;;) {
        UART_read(uart, &uartValue, 1);
        Mailbox_pend(uartRxReady, &uartValue, BIOS_WAIT_FOREVER);

        bldc_interface_uart_process_byte(VESC_UART_STEERING, uartValue);
    }
}

static void send_packet(unsigned int uart_index, unsigned char *data, unsigned int len) {
	if (len > (PACKET_MAX_PL_LEN + 5)) {
		return;
	}

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
	memcpy(buffer, data, len);

	UART_write(uart, &buffer, len); //Blocks with semaphore until write complete
}

void bldc_val_received(unsigned int uart_index, mc_values *val)
{
    Mailbox_post(vescTelemetry, val, BIOS_NO_WAIT);
}

void comm_uart_init(void) {
    UART_Params uartParams;

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = &uartCallbackFxn;
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uart = UART_open(VESC_UART_STEERING_DRV, &uartParams);

    Mailbox_Params uartMboxParams;
    Mailbox_Params_init(&uartMboxParams);
    Mailbox_construct(&uartRxReadyStct, sizeof(uint8_t), 64, &uartMboxParams, NULL);
    uartRxReady = Mailbox_handle(&uartRxReadyStct);

    bldc_interface_uart_init(VESC_UART_STEERING, send_packet);
    bldc_interface_set_rx_value_func(bldc_val_received);

    Task_Params periodicTaskParams, processingTaskParams;

    Task_Params_init(&periodicTaskParams);
    periodicTaskParams.stackSize = VESCTIMEOUT_STACK;
    periodicTaskParams.stack = &periodicRefreshStack;
    periodicTaskParams.priority = VESCTIMEOUT_PRIORITY;
    Task_construct(&periodicRefresh, (Task_FuncPtr)periodicUpdate, &periodicTaskParams, NULL);

    Task_Params_init(&processingTaskParams);
    processingTaskParams.stackSize = VESCPROCESS_STACK;
    processingTaskParams.stack = uartProcessStack;
    processingTaskParams.priority = VESCPROCESS_PRIORITY;
    Task_construct(&uartProcess, (Task_FuncPtr)uartProcessFxn, &processingTaskParams, NULL);
}
