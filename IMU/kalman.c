/*
 * kalman.c
 *
 *  Created on: Mar 1, 2020
 *      Author: brandon
 */


#include "kalman.h"

#include <xdc/runtime/Error.h>

/*
 * Kalman filter initialization
 */
void Kalman_init(Kalman *filter)
{
    filter->Q_angle = 0.001f;
    filter->Q_bias = 0.003f;
    filter->R_measure = 0.03f;

    filter->angle = 0.0f;
    filter->bias = 0.0f;

    filter->P[0][0] = 0.0f;
    filter->P[0][1] = 0.0f;
    filter->P[1][0] = 0.0f;
    filter->P[1][1] = 0.0f;

    Error_Block eb;
    Error_init(&eb);
    filter->dataAccess = GateMutex_create(NULL, &eb);
}

/*
 * Kalman filter update process
 */
float Kalman_updateFilter(Kalman *filter, float newAngle, float newRate, float dt)
{
    uint32_t key = GateMutex_enter(filter->dataAccess);

    //Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    filter->rate = newRate - filter->bias;
    filter->angle += dt * filter->rate;

    //Update estimation error covariance - Project the error covariance ahead
    filter->P[0][0] += dt * (dt*filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->Q_angle);
    filter->P[0][1] -= dt * filter->P[1][1];
    filter->P[1][0] -= dt * filter->P[1][1];
    filter->P[1][1] += dt * filter->Q_bias;

    //Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    //Calculate Kalman gain - Compute the Kalman gain
    float S = filter->P[0][0] + filter->R_measure;

    float K[2];
    K[0] = filter->P[0][0] / S;
    K[1] = filter->P[1][0] / S;

    //Calculate angle and bias - Update estimate with measurement zk (newAngle)
    float y = newAngle - filter->angle;
    filter->angle += K[0] * y;
    filter->bias += K[1] * y;

    //Calculate estimation error covariance - Update the error covariance
    float P00_temp = filter->P[0][0];
    float P01_temp = filter->P[0][1];

    filter->P[0][0] -= K[0] * P00_temp;
    filter->P[0][1] -= K[0] * P01_temp;
    filter->P[1][0] -= K[1] * P00_temp;
    filter->P[1][1] -= K[1] * P01_temp;

    float ret = filter->angle;
    GateMutex_leave(filter->dataAccess, key);

    return ret;
}

/*
 * Kalman filter return filtered angle
 */
float Kalman_getAngle(Kalman *filter)
{
    uint32_t key = GateMutex_enter(filter->dataAccess);

    float ret = filter->angle;

    GateMutex_leave(filter->dataAccess, key);

    return ret;
}

/*
 * Write initial angle to filter
 */
void Kalman_setAngle(Kalman *filter, float angle)
{
    uint32_t key = GateMutex_enter(filter->dataAccess);

    filter->angle = angle;

    GateMutex_leave(filter->dataAccess, key);
}
