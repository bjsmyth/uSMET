/*
 * steering.h
 *
 *  Created on: Mar 5, 2020
 *      Author: brandon
 */

#ifndef STEERING_STEERING_H_
#define STEERING_STEERING_H_

#include <stdint.h>

#define STEERING_PRIO (15)
#define STEERING_STACK (512)

#define STEERING_DT (50)

void steer_setAngle(float angle);
int32_t steer_getControlRpm();
float steer_getCurrentAngle();

void steering_init();

#endif /* STEERING_STEERING_H_ */
