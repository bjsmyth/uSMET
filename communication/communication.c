/*
 * communication.c
 *
 *  Created on: Feb 4, 2020
 *      Author: brandon
 */

#include <stdint.h>
#include <ti/drivers/UART.h>
#include <xdc/runtime/System.h>


#include "config.h"
#include "communication.h"
#include "checksum/checksum.h"
#include "vesc_uart/buffer.h"

static UART_Handle uart;


communication_status communication_uart_decode(uint8_t *packt, uint32_t len, packet *p)
{
    if(*packt != CTS_BYTE) {
        return FAILURE;
    }
    packet tmp;
    int32_t idx = 0;
    if(crc_8(packt, len)) { // CRC non-zero, invalid
        return CRC_INVALID;
    }

    tmp.steering_pos = buffer_get_uint16(packt, &idx);
    tmp.reartrack_pos = buffer_get_uint16(packt, &idx);
    tmp.reartrack_duty = buffer_get_float32(packt, SIG_FIG, &idx);
    tmp.vehicle_speed = buffer_get_float32(packt, SIG_FIG, &idx);
    tmp.vehicle_roll = buffer_get_float32(packt, SIG_FIG, &idx);

    memcpy(p, &tmp, sizeof(packet));

    return SUCCESS;
}

void communication_uart_send(packet p)
{
    static uint8_t buffer[128];
    int32_t len = 0;
    buffer[len] = CTS_BYTE;
    len++;
    //buffer_append_uint16(buffer, CTS_BYTE, &len);
    buffer_append_uint16(buffer, p.steering_pos, &len);
    buffer_append_uint16(buffer, p.reartrack_pos, &len);
    buffer_append_float32(buffer, p.reartrack_duty, SIG_FIG, &len);
    buffer_append_float32(buffer, p.vehicle_speed, SIG_FIG, &len);
    buffer_append_float32(buffer, p.vehicle_roll, SIG_FIG, &len);
    buffer[len] = crc_8(buffer, len);
    len++;

    UART_write(uart, buffer, len); //Blocks with semaphore until write complete
}

void communication_uart_init()
{
    UART_Params uartParams;

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uart = UART_open(COMMUNICATION_UART, &uartParams);
}
