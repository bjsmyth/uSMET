/*
 * utilities.h
 *
 *  Created on: Feb 23, 2020
 *      Author: brandon
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#define DEG_TO_RAD_RATIO    0.0174532f
#define RAD_TO_DEG_RATIO    57.2957795f


inline float map_float(float x, float in_min, float in_max, float out_min, float out_max);
inline int map_int(int x, int in_min, int in_max, int out_min, int out_max);
inline float RadiansToDegrees(float radians){ return radians*RAD_TO_DEG_RATIO; }
inline float DegreesToRadians(float degrees){ return degrees*DEG_TO_RAD_RATIO; }

#endif /* UTILITIES_H_ */
