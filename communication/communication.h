/*
 * communication.h
 *
 *  Created on: Feb 4, 2020
 *      Author: brandon
 */

#ifndef COMMUNICATION_COMMUNICATION_H_
#define COMMUNICATION_COMMUNICATION_H_

#define CTS_BYTE (0xA5)
#define SIG_FIG (1e4)

typedef enum {
    SUCCESS,
    FAILURE,
    CRC_INVALID
} communication_status;

typedef struct {
    uint16_t steering_pos;
    uint16_t reartrack_pos;
    float reartrack_duty;
    float vehicle_speed;
    float vehicle_roll;
} packet;

void communication_uart_init();

communication_status communication_uart_decode(uint8_t *packt, uint32_t len, packet *p);

void communication_uart_send(packet p);

void communication_process();


#endif /* COMMUNICATION_COMMUNICATION_H_ */
