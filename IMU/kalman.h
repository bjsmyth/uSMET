/*
 * kalman.h
 *
 *  Created on: Mar 1, 2020
 *      Author: brandon
 */

#ifndef IMU_KALMAN_H_
#define IMU_KALMAN_H_

#include <ti/sysbios/gates/GateMutex.h>

typedef struct Kalman {
    float Q_angle;
    float Q_bias;
    float R_measure;

    float angle;
    float bias;
    float rate;

    float P[2][2];

    GateMutex_Handle dataAccess;
} Kalman;

void Kalman_init(Kalman *filter);
float Kalman_updateFilter(Kalman *filter, float newAngle, float newRate, float dt);
float Kalman_getAngle(Kalman *filter);
void Kalman_setAngle(Kalman *filter, float angle);

#endif /* IMU_KALMAN_H_ */
