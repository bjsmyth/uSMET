/*
 * communication.h
 *
 *  Created on: Feb 4, 2020
 *      Author: brandon
 */

#ifndef COMMUNICATION_COMMUNICATION_H_
#define COMMUNICATION_COMMUNICATION_H_

#include "IMU/MPU9150.h"

#define CTS_BYTE (0xA5)
#define SIG_FIG (1e4)

typedef enum {
    SUCCESS,
    FAILURE,
    CRC_INVALID
} communication_status;

typedef struct {
    uint8_t flags;
    uint16_t steering_pos;
    uint16_t reartrack_pos;
    float reartrack_duty;
    float vehicle_speed;
    float vehicle_roll;
    MPU9150_Data accel;
    MPU9150_Data gyro;
} packet;


void communication_uart_init();

communication_status communication_uart_decode(uint8_t *packt, uint32_t len, packet *p);

int16_t communication_uart_send(packet p);

void communication_process();

#endif /* COMMUNICATION_COMMUNICATION_H_ */
