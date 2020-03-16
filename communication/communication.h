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
    float steering_current_pos;
    int32_t steering_motor_set_rpm;
    float steering_motor_current_rpm;
    uint16_t reartrack_pos;
    float reartrack_duty;
    float vehicle_speed;
    float vehicle_roll;
    MPU9150_Data accel;
    MPU9150_Data gyro;
} tx_packet;

typedef struct {
    float steering_set_pos;
    float vehicle_set_speed;
    float reartrack_set_angle;
} rx_packet;

void communication_uart_init();

communication_status communication_uart_decode(uint8_t *packt, uint32_t len, tx_packet *p);

rx_packet communication_uart_send(tx_packet p);

void communication_process();

#endif /* COMMUNICATION_COMMUNICATION_H_ */
