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


communication_status communication_uart_decode(uint8_t *packt, uint32_t len, packet *p)
{
    if(*packt != CTS_BYTE) {
        return FAILURE;
    }
    packet tmp;
    int32_t idx = 0;

    tmp.steering_pos = buffer_get_uint16(packt, &idx);
    tmp.reartrack_pos = buffer_get_uint16(packt, &idx);
    tmp.reartrack_duty = buffer_get_float32(packt, SIG_FIG, &idx);
    tmp.vehicle_speed = buffer_get_float32(packt, SIG_FIG, &idx);
    tmp.vehicle_roll = buffer_get_float32(packt, SIG_FIG, &idx);

    memcpy(p, &tmp, sizeof(packet));

    return SUCCESS;
}

int16_t communication_uart_send(packet p)
{
    static uint8_t buffer[256];
    uint8_t readVar;
    int16_t testRet;
    int readReturn, timeout;
    int32_t len = 0;
    buffer[len++] = CTS_BYTE;
    buffer[len++] = p.flags;
    //buffer_append_uint16(buffer, CTS_BYTE, &len);
    buffer_append_uint16(buffer, p.steering_pos, &len);
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
            return 0;
        }
    }while(readReturn == 0);

    UART_write(comm_uart, buffer, len); //Blocks with semaphore until write complete
    UART_read(comm_uart, buffer, 1);
    UART_write(comm_uart, buffer, 1);
    UART_read(comm_uart, buffer, 4);
    len = 0;
    uint16_t crc = crc16(buffer, 4);

    if(crc != 0) {
        return 0;
    }

    testRet = buffer_get_int16(buffer, &len);

    return testRet;
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
    uartParams.readTimeout = 10;
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = COMMUNICATION_UART_BAUD;
    comm_uart = UART_open(COMMUNICATION_UART, &uartParams);
}
