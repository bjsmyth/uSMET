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
#include "vesc_uart/crc.h"
#include "vesc_uart/buffer.h"

static UART_Handle comm_uart;


communication_status communication_uart_decode(uint8_t *packt, uint32_t len, tx_packet *p)
{
    if(*packt != CTS_BYTE) {
        return FAILURE;
    }

    return SUCCESS;
}

rx_packet communication_uart_send(tx_packet p)
{
    static uint8_t buffer[256];
    uint8_t readVar;
    rx_packet ret = { 0 };
    int readReturn, timeout;
    int32_t len = 0;
    buffer[len++] = CTS_BYTE;
    buffer[len++] = p.flags;
    //buffer_append_uint16(buffer, CTS_BYTE, &len);
    buffer_append_float32_true(buffer, p.steering_current_pos, &len);
    buffer_append_int32(buffer, p.steering_motor_set_rpm, &len);
    buffer_append_float32_true(buffer, p.steering_motor_current_rpm, &len);
    buffer_append_uint16(buffer, p.reartrack_pos, &len);
    buffer_append_float32_true(buffer, p.reartrack_duty, &len);
    buffer_append_float32_true(buffer, p.vehicle_speed, &len);
    buffer_append_float32_true(buffer, p.accel.xFloat, &len);
    buffer_append_float32_true(buffer, p.accel.yFloat, &len);
    buffer_append_float32_true(buffer, p.accel.zFloat, &len);
    buffer_append_float32_true(buffer, p.gyro.xFloat, &len);
    buffer_append_float32_true(buffer, p.gyro.yFloat, &len);
    buffer_append_float32_true(buffer, p.gyro.zFloat, &len);
    buffer_append_float32_true(buffer, p.vehicle_roll, &len);
    buffer_append_uint16(buffer, crc16(buffer, len), &len);

    timeout = 0;
    do {
        UART_write(comm_uart, &len, 1);
        readReturn = UART_read(comm_uart, &readVar, 1);
        if((++timeout) == 5) {
            return ret;
        }
    }while(readReturn == 0);

    UART_write(comm_uart, buffer, len); //Blocks with semaphore until write complete
    UART_read(comm_uart, buffer, 1);
    UART_write(comm_uart, buffer, 1);
    UART_read(comm_uart, buffer, 14);
    len = 0;
    uint16_t crc = crc16(buffer, 14);

    if(crc != 0) {
        return ret;
    }

    ret.steering_set_pos = buffer_get_float32_true(buffer, &len);
    ret.vehicle_set_speed = buffer_get_float32_true(buffer, &len);
    ret.reartrack_set_angle = buffer_get_float32_true(buffer, &len);

    return ret;
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
    uartParams.readTimeout = 50;
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = COMMUNICATION_UART_BAUD;
    comm_uart = UART_open(COMMUNICATION_UART, &uartParams);
}
