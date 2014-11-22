#ifndef _MPU9150
#define _MPU9150
#define DEBUG(X) printf(X)
#include "MPU9150.h"
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <i2cmaster.h>

uint8_t _i2c_addr = 0;

int _i2c_write(uint8_t dest, uint8_t val){
	i2c_start_wait(_i2c_addr << 1);
	if(i2c_write(dest)){
		return 1;
	}
	if(i2c_write(val)){
		return 1;
	}
	i2c_stop();
	return 0;
}

uint8_t _i2c_read(uint8_t reg){
	i2c_start_wait(_i2c_addr << 1);
	if(i2c_write(reg)){
		return 255;
	}
	if(i2c_rep_start((_i2c_addr << 1) + 1)){
		return 255;
	}
	uint8_t ret = i2c_readNak();
	i2c_stop();
	return ret;
}

int MPU9150_init(){
	DEBUG("Initializing MPU9150...\n");
	// Initialize TWI
	i2c_init();

	DEBUG("Turning on MPU9150...");
	_i2c_addr = ADDR1;
	_i2c_write(MPU9150_PWR_MGMT_1, 1 << 0);
	DEBUG("done\n");

	// Configure gyro to +- 250 deg / sec
	DEBUG("Configuring gyroscope rate...");
	//	temp = _i2c_read(MPU9150_GYRO_CONFIG);
	_i2c_write(MPU9150_GYRO_CONFIG, 0);
	DEBUG("done\n");

	// configure acc to +- 2g
	DEBUG("Configuring accelerometer sensitivity...");
	uint8_t temp = _i2c_read(MPU9150_ACCEL_CONFIG);
	_i2c_write(MPU9150_ACCEL_CONFIG, temp & ~(1 << 3 | 1 << 4));
	DEBUG("done\n");

	// enable mag passthrough
	DEBUG("Enabling magnetometer passthrough...");
	_i2c_write(MPU9150_I2C_INT_PIN_CFG, 1 << 1);
	DEBUG("done\n");

	// configure mag
	DEBUG("Configuring Magnetometer...");
	_i2c_addr = ADDR2;
	_i2c_write(MPU9150_MAG_CNTL, 1 << 0);
	_i2c_addr = ADDR1;
	DEBUG("done\n");

/*	// configure i2c master
	DEBUG("Configuring I2C Master...")
	_i2c_write(MPU9150_I2C_MST_CTRL, 1 << 4 | 8);

	// configure i2c slave 0
	_i2c_write(MPU9150_I2C_SLV0_ADDR, ADDR2 | 1 << 7);
	_i2c_write(MPU9150_I2C_SLV0_REG, 0x03);
	_i2c_write(MPU9150_I2C_SLV0_CTRL, 6 | 1 << 7);
	DEBUG("done\n");*/

	// configure interrupt enable
	_i2c_write(MPU9150_INT_ENABLE, 1);

	// Configure controls
	_i2c_write(MPU9150_USER_CTRL, 1 << 6 | 1 << 5);
	return 0;
}

int MPU9150_Read(){
	uint8_t byte_H;
	uint8_t byte_L;

	_i2c_addr = ADDR1;

	// accelX
	i2c_start_wait(_i2c_addr << 1);	// write SLA+W
	i2c_write(MPU9150_ACCEL_XOUT_H);	// write start register
	i2c_rep_start((_i2c_addr << 1) + 1);	// write SLA+R
	byte_H = i2c_readAck();
	byte_L = i2c_readAck();
	accelX = byte_H << 8 | byte_L;

	// accelY
	byte_H = i2c_readAck();
	byte_L = i2c_readAck();
	accelY = byte_H << 8 | byte_L;

	// accelZ
	byte_H = i2c_readAck();
	byte_L = i2c_readAck();
	accelZ = byte_H << 8 | byte_L;

	// Temp
	byte_H = i2c_readAck();
	byte_L = i2c_readAck();
	temp = byte_H << 8 | byte_L;

	// gyroX
	byte_H = i2c_readAck();
	byte_L = i2c_readAck();
	gyroX = byte_H << 8 | byte_L;

	// gyroY
	byte_H = i2c_readAck();
	byte_L = i2c_readAck();
	gyroY = byte_H << 8 | byte_L;

	// gyroZ
	byte_H = i2c_readAck();
	byte_L = i2c_readNak();
	gyroZ = byte_H << 8 | byte_L;
	i2c_stop();

	// Read from Mag
	_i2c_addr = ADDR2;
	// Activate measurement
	_i2c_write(MPU9150_MAG_CNTL, 1);
	while(!_i2c_read(MPU9150_MAG_ST1)){
		continue;
	}
	// magX
	i2c_start_wait(_i2c_addr << 1);	// SLA+W
	i2c_write(MPU9150_MAG_HXL);	// write start reg
	i2c_rep_start((_i2c_addr << 1) + 1);
	byte_L = i2c_readAck();
	byte_H = i2c_readAck();
	magX = byte_H << 8 | byte_L;

	// magY
	byte_L = i2c_readAck();
	byte_H = i2c_readAck();
	magY = byte_H << 8 | byte_L;

	// magZ
	byte_L = i2c_readAck();
	byte_H = i2c_readNak();
	i2c_stop();
	magZ = byte_H << 8 | byte_L;
	_i2c_addr = ADDR1;
	return 0;
}
#endif
