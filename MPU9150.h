/*
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== mpu9150.h ========
 */

#ifndef MPU9150_H_
#define MPU9150_H_

#include <stdint.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/drivers/I2C.h>

#define MPU9150_COUNT	                    1
#define MPU9150_SENSOR_REGISTER_SET_SIZE    22

/*
 * MPU9150 data structure is used by the MPU9150_get* APIs to extract the
 * requested data from the previously captured data read using MPU9150_read()
 */
typedef struct MPU9150_Data {
	union {
		struct {
			int16_t	x;
			int16_t	y;
			int16_t	z;
		};
		struct {
			float	xFloat;
			float	yFloat;
			float	zFloat;
		};
		struct {
			int16_t temperature;
		};
		struct {
			float   temperatureCFloat;
			float   temperatureFFloat;
		};
	};
} MPU9150_Data;

typedef struct MPU9150_Object {
	I2C_Handle       i2c;
	uint8_t	         i2cAddr;
	GateMutex_Handle dataAccess;
	uint8_t          data[MPU9150_SENSOR_REGISTER_SET_SIZE];
} MPU9150_Object, *MPU9150_Handle;

/*
 *  ======== MPU9150_init ========
 *  Function opens the I2C controller and initializes the MPU9150
 *  It returns a non-zero MPU9150 handle if the MPU9150 initialized
 *  successfully.
 *  This is a one time call
 */
MPU9150_Handle MPU9150_init(unsigned int mpu9105Index,
                            unsigned int i2cIndex,
                            uint8_t i2cAddr);

/*
 *  ======== MPU9150_read ========
 *  Function reads all the MPU9150 data registers.
 *  The data registers read are: Accelerometer, Gyroscope, Magnetometer, and
 *  Temperature.
 *  This function should be called when the MPU9150 has sensor data ready;
 *  which is typically notified via a GPIO interrupt.
 */
bool MPU9150_read(MPU9150_Handle handle);

/*
 *  ======== MPU9150_getAccelRaw ========
 *  Function returns the raw Acceleration register values for X,Y,and Z axes.
 *  Returns true if successful.
 */
bool MPU9150_getAccelRaw(MPU9150_Handle handle, MPU9150_Data *data);

/*
 *  ======== MPU9150_getAccelFloat ========
 *  Function returns a processed float values of the accelerometer in (m/s^2).
 *  Returns true if successful.
 */
bool MPU9150_getAccelFloat(MPU9150_Handle handle, MPU9150_Data *data);

/*
 *  ======== MPU9150_getGyroRaw ========
 *  Function returns the raw Gyroscope register values for X,Y,and Z axes.
 *  Returns true if successful.
 */
bool MPU9150_getGyroRaw(MPU9150_Handle handle, MPU9150_Data *data);

/*
 *  ======== MPU9150_getGyroFloat ========
 *  Function returns a processed float values of the gyroscope in (rad/s).
 *  Returns true if successful.
 */
bool MPU9150_getGyroFloat(MPU9150_Handle handle, MPU9150_Data *data);

/*
 *  ======== MPU9150_getMagnetoRaw ========
 *  Function returns the raw Magnetometer register values for X,Y,and Z axes.
 *  Returns true if successful.
 */
bool MPU9150_getMagnetoRaw(MPU9150_Handle handle, MPU9150_Data *data);

/*
 *  ======== MPU9150_getMagnetoFloat ========
 *  Function returns a processed float values of the Magnetometer in (uT).
 *  Returns true if successful.
 */
bool MPU9150_getMagnetoFloat(MPU9150_Handle handle, MPU9150_Data *data);

/*
 *  ======== MPU9150_getTemperatureRaw ========
 *  Function returns the raw Temperature register values.
 *  Returns true if successful.
 */
bool MPU9150_getTemperatureRaw(MPU9150_Handle handle, MPU9150_Data *data);

/*
 *  ======== MPU9150_getTemperatureFloat ========
 *  Function returns a processed float values of the Temperature in (C) & (F).
 *  Returns true if successful.
 */
bool MPU9150_getTemperatureFloat(MPU9150_Handle handle, MPU9150_Data *data);

#endif /* MPU9150_H_ */
