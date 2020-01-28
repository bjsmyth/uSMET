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
Char periodicRefreshStack[2048];

Task_Struct uartProcess;
Char uartProcessStack[2048];

static void uartCallbackFxn(UART_Handle handle, void *buf, size_t count)
{
    unsigned char *buffer = buf;
    unsigned int i;
    for(i = 0; i < count; i++) {
        Mailbox_post(vescDataRx, &buffer[i], BIOS_NO_WAIT);
    }
}

static void periodicUpdate(UArg arg0, UArg arg1) {
    uint32_t i = 0;
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
}

void bldc_val_received(mc_values *val)
{
    memcpy(&vescFeedback, val, sizeof(mc_values));

    System_printf("Input voltage: %.2f V\n", val->v_in);
    System_printf("Temp:          %.2f degC\n", val->temp_mos);
    System_printf("Current motor: %.2f A\n", val->current_motor);
    System_printf("Current in:    %.2f A\n", val->current_in);
    System_printf("RPM:           %.1f RPM\n", val->rpm);
    System_printf("Duty cycle:    %.1f %%\n", val->duty_now * 100.0);
    System_printf("Ah Drawn:      %.4f Ah\n", val->amp_hours);
    System_printf("Ah Regen:      %.4f Ah\n", val->amp_hours_charged);
    System_printf("Wh Drawn:      %.4f Wh\n", val->watt_hours);
    System_printf("Wh Regen:      %.4f Wh\n", val->watt_hours_charged);
    System_printf("Tacho:         %i counts\n", val->tachometer);
    System_printf("Tacho ABS:     %i counts\n", val->tachometer_abs);
    System_printf("Fault Code:    %s\n", bldc_interface_fault_to_string(val->fault_code));

    System_flush();
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
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uart = UART_open(Board_UART0, &uartParams);

    bldc_interface_uart_init(send_packet);
    bldc_interface_set_rx_value_func(bldc_val_received);

    Task_Params periodicTaskParams, processingTaskParams;

    Task_Params_init(&periodicTaskParams);
    periodicTaskParams.stackSize = 512;
    periodicTaskParams.stack = &periodicRefreshStack;
    periodicTaskParams.priority = 1;
    Task_construct(&periodicRefresh, (Task_FuncPtr)periodicUpdate, &periodicTaskParams, NULL);

    Task_Params_init(&processingTaskParams);
    processingTaskParams.stackSize = 2048;
    processingTaskParams.stack = uartProcessStack;
    processingTaskParams.priority = 2;
    Task_construct(&uartProcess, (Task_FuncPtr)uartProcessFxn, &processingTaskParams, NULL);
}
