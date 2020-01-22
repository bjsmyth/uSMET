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


#include <ti/drivers/UART.h>


mc_values vescFeedback;

static UART_Handle uart;
// Settings

// Private functions
static void send_packet(unsigned char *data, unsigned int len);

// Threads
Task_Struct periodicRefresh;
Char periodicRefreshStack[512];

Task_Struct uartProcess;
Char uartProcessStack[1024];

static void uartCallbackFxn(UART_Handle handle, void *buf, size_t count)
{
    unsigned char *buffer = buf;
    unsigned int i;
    for(i = 0; i < count; i++) {
        Mailbox_post(vescDataRx, &buffer[i], BIOS_NO_WAIT);
    }
}

static void periodicUpdate(UArg arg0, UArg arg1) {
	for(;;) {
	    bldc_interface_uart_run_timer();
	    Task_sleep(1);
	}
}

static void uartProcessFxn(UArg arg0, UArg arg1) {
    unsigned char uartValue;
    for(;;) {
        UART_read(uart, &uartValue, 1);
        Mailbox_pend(vescDataRx, &uartValue, BIOS_WAIT_FOREVER);

        bldc_interface_uart_process_byte(uartValue);
    }
}

static void send_packet(unsigned char *data, unsigned int len) {
	if (len > (PACKET_MAX_PL_LEN + 5)) {
		return;
	}

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
	memcpy(buffer, data, len);

	UART_write(uart, &buffer, len);

	// Send the data over UART
	//uartStartSend(&UART_DEV, len, buffer);
}

void bldc_val_received(mc_values *val)
{
    memcpy(&vescFeedback, val, sizeof(mc_values));
}

void comm_uart_init(void) {
	// Initialize UART
    Board_initUART();

    UART_Params uartParams;

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = &uartCallbackFxn;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uart = UART_open(Board_UART0, &uartParams);

    bldc_interface_uart_init(send_packet);
    bldc_interface_set_rx_value_func(bldc_val_received);

    Task_Params task1, task2;

    Task_Params_init(&task1);
    task1.stackSize = 512;
    task1.stack = &periodicRefreshStack;
    task1.priority = 2;
    Task_construct(&periodicRefresh, (Task_FuncPtr)periodicUpdate, &task1, NULL);

    Task_Params_init(&task2);
    task2.stackSize = 1024;
    task2.stack = uartProcessStack;
    task2.priority = 2;
    Task_construct(&uartProcess, (Task_FuncPtr)uartProcessFxn, &task2, NULL);
}
