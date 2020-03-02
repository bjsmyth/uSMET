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
 *    ======== mpu9150.c ========
 */

#include "MPU9150.h"
#include <sensorlib/hw_mpu9150.h>

#include <xdc/runtime/Error.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

static MPU9150_Object object[MPU9150_COUNT] = {0};

MPU9150_Handle MPU9150_init(unsigned int mpu9105Index,
                            unsigned int i2cIndex,
                            uint8_t i2cAddr)
{
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;
    Error_Block     eb;
    bool			transferOK;
    uint8_t		    writeBuffer[5];
    uint8_t		    readBuffer[1];
    MPU9150_Handle	handle = &object[mpu9105Index];

    /* Check if the handle was already opened */
    if((mpu9105Index > MPU9150_COUNT) || (handle->i2c != NULL)) {
    	return (NULL);
    }

    /*
     * Create GateMutex to guarantee that data received from the MPU9150
     * retains its coherency.
     */
    Error_init(&eb);
    handle->dataAccess = GateMutex_create(NULL, &eb);
    if (!handle->dataAccess) {
    	return (NULL);
    }

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    handle->i2c = I2C_open(i2cIndex, &i2cParams);
    handle->i2cAddr =  i2cAddr;

    /* If the I2C controller opened properly continue */
    if (handle->i2c) {

    	/* Used by all I2C transfers */
    	i2cTransaction.slaveAddress = i2cAddr;
    	i2cTransaction.writeBuf = writeBuffer;
    	i2cTransaction.readBuf = readBuffer;


    	/* Put the MPU9150 into reset state */
    	writeBuffer[0] = MPU9150_O_PWR_MGMT_1;
    	writeBuffer[1] = MPU9150_PWR_MGMT_1_DEVICE_RESET;
    	i2cTransaction.writeCount = 2;
		i2cTransaction.readCount = 0;
       	if (!I2C_transfer(handle->i2c, &i2cTransaction)) {
			return (NULL);
		}

       	/*
		 * Check the value read back from status to determine if device
		 * is still in reset or if it is ready.  Reset state for this
		 * register is 0x40, which has sleep bit set. Device may also
		 * respond with an address NACK during very early stages of the
		 * its internal reset.  Keep polling until we verify device is
		 * ready.
		 */
       	writeBuffer[0] = MPU9150_O_PWR_MGMT_1;
       	i2cTransaction.writeCount = 1;
    	i2cTransaction.readCount = 1;
    	do {
    		transferOK = I2C_transfer(handle->i2c, &i2cTransaction);
    	} while ((readBuffer[0] != MPU9150_PWR_MGMT_1_SLEEP) || (!transferOK));

    	/* Take the device out of reset and enable the clock */
       	writeBuffer[0] = MPU9150_O_PWR_MGMT_1;
       	writeBuffer[1] = MPU9150_PWR_MGMT_1_CLKSEL_XG;
       	i2cTransaction.writeCount = 2;
    	i2cTransaction.readCount = 0;
       	if (!I2C_transfer(handle->i2c, &i2cTransaction)) {
			return (NULL);
		}

    	/* Enable A I2C Master mode on the MPU9150 */
       	writeBuffer[0] = MPU9150_O_USER_CTRL;
       	writeBuffer[1] = MPU9150_USER_CTRL_I2C_MST_EN;
       	i2cTransaction.writeCount = 2;
    	i2cTransaction.readCount = 0;
       	if (!I2C_transfer(handle->i2c, &i2cTransaction)) {
			return (NULL);
		}

       	/*
       	 * Change to power mode complete, device is ready for configuration.
       	 * Set MPU9150's sampling rate
       	 * Set sample rate to 50 hertz.  1000 hz / (1 + 19)
       	 */
       	writeBuffer[0] = MPU9150_O_SMPLRT_DIV;
       	writeBuffer[1] = 9;
       	i2cTransaction.writeCount = 2;
    	i2cTransaction.readCount = 0;
       	if (!I2C_transfer(handle->i2c, &i2cTransaction)) {
			return (NULL);
		}

       	/*
		 * Write the I2C Master delay control so we only sample the AK
		 * every 5th time that we sample accel/gyro.  Delay Count itself
		 * handled in next state.
		 */
       	writeBuffer[0] = MPU9150_O_I2C_MST_DELAY_CTRL;
       	writeBuffer[1] = (MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN |
                          MPU9150_I2C_MST_DELAY_CTRL_I2C_SLV4_DLY_EN);
       	i2cTransaction.writeCount = 2;
    	i2cTransaction.readCount = 0;
       	if (!I2C_transfer(handle->i2c, &i2cTransaction)) {
			return (NULL);
		}

       	/*
		 * Write the configuration for I2C master control clock 400khz
		 * and wait for external sensor before asserting data ready
		 */
       	writeBuffer[0] = MPU9150_O_I2C_MST_CTRL;
		writeBuffer[1] = (MPU9150_I2C_MST_CTRL_I2C_MST_CLK_400 |
                          MPU9150_I2C_MST_CTRL_WAIT_FOR_ES);

		/*
		 * Configure I2C Slave 0 for read of AK8975 (I2C Address 0x0C)
		 * Start at AK8975 register status 1 (0x02)
		 */
		writeBuffer[2] = MPU9150_I2C_SLV0_ADDR_RW | 0x0C;
		writeBuffer[3] = 0x02;
		writeBuffer[4] = MPU9150_I2C_SLV0_CTRL_EN | 0x08;
       	i2cTransaction.writeCount = 5;
    	i2cTransaction.readCount = 0;
       	if (!I2C_transfer(handle->i2c, &i2cTransaction)) {
			return (NULL);
		}

       	/*
		 * Write the configuration for I2C Slave 4 transaction to AK8975
		 * 0x0c is the AK8975 address on i2c bus.
		 * we want to write the control register with the value for a
		 * starting a single measurement.
		 */
       	writeBuffer[0] = MPU9150_O_I2C_SLV4_ADDR;
		writeBuffer[1] = 0x0C;
		writeBuffer[2] = 0x0A; //AK8975_O_CNTL
		writeBuffer[3] = 0x01; //AK8975_CNTL_MODE_SINGLE
		writeBuffer[4] = MPU9150_I2C_SLV4_CTRL_EN | 0x04;
       	i2cTransaction.writeCount = 5;
    	i2cTransaction.readCount = 0;
       	if (!I2C_transfer(handle->i2c, &i2cTransaction)) {
			return (NULL);
		}

        /*
         * Write application specific sensor configuration such as filter
         * settings and sensor range settings.
         */
       	writeBuffer[0] = MPU9150_O_CONFIG;
    	writeBuffer[1] = MPU9150_CONFIG_DLPF_CFG_94_98;
		writeBuffer[2] = MPU9150_GYRO_CONFIG_FS_SEL_250;
		writeBuffer[3] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ |
                          MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
       	i2cTransaction.writeCount = 4;
    	i2cTransaction.readCount = 0;
       	if (!I2C_transfer(handle->i2c, &i2cTransaction)) {
			return (NULL);
		}

       	/*
		 * Configure the data ready interrupt pin output of the MPU9150.
		 */
       	writeBuffer[0] = MPU9150_O_INT_PIN_CFG;
    	writeBuffer[1] = MPU9150_INT_PIN_CFG_INT_LEVEL |
                         MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
                         MPU9150_INT_PIN_CFG_LATCH_INT_EN;;
		writeBuffer[2] = MPU9150_INT_ENABLE_DATA_RDY_EN;
       	i2cTransaction.writeCount = 3;
    	i2cTransaction.readCount = 0;
       	if (!I2C_transfer(handle->i2c, &i2cTransaction)) {
			return (NULL);
		}

    	return (handle);
    }
    else {
    	return (NULL);
    }
}

/*
 * ======== getDataRegisters ========
 */
bool MPU9150_read(MPU9150_Handle handle)
{
    I2C_Transaction i2cTransaction;
    uint8_t         writeBuffer[1];
    uint8_t			data[MPU9150_SENSOR_REGISTER_SET_SIZE];
    unsigned int	key;

	if (handle->i2c) {
	    i2cTransaction.slaveAddress = handle->i2cAddr;
	    i2cTransaction.writeBuf = writeBuffer;
	    i2cTransaction.writeCount = 1;
	    i2cTransaction.readBuf = data;
	    i2cTransaction.readCount = MPU9150_SENSOR_REGISTER_SET_SIZE;

	    /*
	     * I2C peripherals generally have an internal burst counter.
	     * Therefore, start reading from the first data register and just
	     * keep on reading until we have all the data.
	     * Starting with MPU9150_O_ACCEL_XOUT_H, we can read the accelerometer,
	     * temperature, gyroscope, and magnetometer data registers.
	     *
	     * (ACCEL_XOUT_H(0x3B) -> GYRO_ZOUT_L(0x48) = 14 bytes
         * Grab Ext Sens Data as well for another 8 bytes.  ST1 + Mag Data + ST2
	     */
	    writeBuffer[0] = MPU9150_O_ACCEL_XOUT_H;

        if (I2C_transfer(handle->i2c, &i2cTransaction)) {
    	    /*
    	     * To ensure coherency of the data received we copy the data
    	     * atomically
    	     */
    	    key = GateMutex_enter(handle->dataAccess);
    	    memcpy(handle->data, data, MPU9150_SENSOR_REGISTER_SET_SIZE);
    	    GateMutex_leave(handle->dataAccess, key);
        	return (true);
        }
	}

	return (false);
}

/*
 * ======== MPU9150_getAccelRaw ========
 */
bool MPU9150_getAccelRaw(MPU9150_Handle handle, MPU9150_Data *data)
{
    unsigned int    key;

    if (handle) {
        /* A gate is used to ensure data coherency */
		key = GateMutex_enter(handle->dataAccess);
		data->x = (handle->data[0] << 8) | handle->data[1];
		data->y = (handle->data[2] << 8) | handle->data[3];
		data->z = (handle->data[4] << 8) | handle->data[5];
		GateMutex_leave(handle->dataAccess, key);

		return (true);
    }
    return (false);
}

/*
 *  ======== MPU9150_getAccelFloat ========
 */
bool MPU9150_getAccelFloat(MPU9150_Handle handle, MPU9150_Data *data)
{
	MPU9150_Data    tmp;

    /* 
     * Values are obtained by taking the g conversion factors from the data
     * sheet and multiplying by 9.81 (1 g = 9.81 m/s^2).
     * Range = +/- 2 g (16384 lsb/g)
     */
	if (MPU9150_getAccelRaw(handle, &tmp)) {
		data->xFloat = tmp.x * 0.0005985482;
		data->yFloat = tmp.y * 0.0005985482;
		data->zFloat = tmp.z * 0.0005985482;

		return (true);
	}
	return (false);
}

/*
 *  ======== MPU9150_getGyroRaw ========
 */
bool MPU9150_getGyroRaw(MPU9150_Handle handle, MPU9150_Data *data)
{
    unsigned int    key;

    if (handle) {
        /* A gate is used to ensure data coherency */
		key = GateMutex_enter(handle->dataAccess);
		data->x = (handle->data[8]  << 8) | handle->data[9];
		data->y = (handle->data[10] << 8) | handle->data[11];
		data->z = (handle->data[12] << 8) | handle->data[13];
		GateMutex_leave(handle->dataAccess, key);

		return (true);
    }
    return (false);
}

/*
 *  ======== MPU9150_getGyroFloat ========
 */
bool MPU9150_getGyroFloat(MPU9150_Handle handle, MPU9150_Data *data)
{
	MPU9150_Data    tmp;

    /*
     * Values are obtained by taking the degree per second conversion factors
     * from the data sheet and then converting to radians per sec (1 degree =
     * 0.0174532925 radians).
     * Range = +/- 250 dps (131.0)
     */
	if (MPU9150_getGyroRaw(handle, &tmp)) {
		data->xFloat = tmp.x * 1.3323124e-4;
		data->yFloat = tmp.y * 1.3323124e-4;
		data->zFloat = tmp.z * 1.3323124e-4;

		return (true);
	}
	return (false);
}

/*
 *  ======== MPU9150_getMagnetoRaw ========
 */
bool MPU9150_getMagnetoRaw(MPU9150_Handle handle, MPU9150_Data *data)
{
    unsigned int	key;

    /*
     * The registers from the magnetometer are stored as Low Byte and followed
     * by the High Byte. This allows for easy 16-bit typecasting to extract
     * the data.
     */
    if (handle) {
        /* A gate is used to ensure data coherency */
		key = GateMutex_enter(handle->dataAccess);
		data->x = *((int16_t*)(handle->data + 15));
		data->y = *((int16_t*)(handle->data + 17));
		data->z = *((int16_t*)(handle->data + 19));
		GateMutex_leave(handle->dataAccess, key);

		return (true);
    }
    return (false);
}

/*
 *  ======== MPU9150_getMagnetoFloat ========
 */
bool MPU9150_getMagnetoFloat(MPU9150_Handle handle, MPU9150_Data *data)
{
	MPU9150_Data    tmp;

	/*
	 * 1 LSB = 0.3 Magnetic Flux density in uT.
	 */
	if (MPU9150_getMagnetoRaw(handle, &tmp)) {
		data->xFloat = (float)tmp.x * 0.3;
		data->yFloat = (float)tmp.y * 0.3;
		data->zFloat = (float)tmp.z * 0.3;

		return (true);
	}
	return (false);
}

/*
 *  ======== MPU9150_getTemperatureRaw ========
 */
bool MPU9150_getTemperatureRaw(MPU9150_Handle handle, MPU9150_Data *data)
{
    unsigned int	key;

    if (handle) {
        /* A gate is used to ensure data coherency */
		key = GateMutex_enter(handle->dataAccess);
		data->temperature = (handle->data[6]  << 8) | handle->data[7];
		GateMutex_leave(handle->dataAccess, key);

		return (true);
    }
    return (false);
}

/*
 *  ======== MPU9150_getTemperatureFloat ========
 */
bool MPU9150_getTemperatureFloat(MPU9150_Handle handle, MPU9150_Data *data)
{
	MPU9150_Data tmp;

	if (MPU9150_getTemperatureRaw(handle, &tmp)) {
		data->temperatureCFloat = (tmp.temperature / 340.f) + 35.f;
		data->temperatureFFloat = data->temperatureCFloat * 1.8f + 32.f;
		return (true);
	}
	return (false);
}
