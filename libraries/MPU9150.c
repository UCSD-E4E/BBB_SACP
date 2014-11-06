#include "MPU9150.h"
#include <stdint.h>
#include <avr/io.h>

uint8_t _i2c_addr = 0;

int _i2c_write(uint8_t dest, uint8_t val){
	// Set Start condition
	TWCR = 1 << 7 | 1 << 5 | 1 << 2;
	// Check for master control
	while(!(TWCR & (1 << 7))){
		// wait for completion of I2C operation
	}
	if((TWSR & 0xf8) != 0x08 && (TWSR & 0xf8) != 0x10){
		return -1;
	}
	// Load _i2c_addr and send
	TWDR = _i2c_addr << 1;
	TWCR = 1 << 7 | 1 << 2;
	// Check for ACK
	while(!(TWCR & (1 << 7))){
		// wait for i2c operation
	}
	if(!(TWSR & 0x08 && TWSR ^ 0x30)){
		return -1;
	}
	// Load dest and send
	TWDR = dest;
	TWCR = 1 << 7 | 1 << 2;
	// check for ACK
	while(!(TWCR & (1 << 7))){
		// wait for i2c op
	}
	if(!(TWSR & 0x08 && TWSR ^ 0x30)){
		return -1;
	}
	// Load val and send
	TWDR = val;
	TWCR = 1 << 7 | 1 << 2;
	// check for ack
	while(!(TWCR & (1 << 7))){
		// wait for i2c op
	}
	if(!(TWSR & 0x08 && TWSR ^ 0x30)){
		return -1;
	}
	// set STOP cond
	TWCR = 1 << 7 | 1 << 2 | 1 << 4;
	return 0;
}
uint8_t _i2c_read(uint8_t reg){
	// Set Start condition
	TWCR = 1 << 7 | 1 << 5 | 1 << 2;
	// Check for master control
	while(!(TWCR & (1 << 7))){
		// wait for completion of I2C operation
	}
	if((TWSR & 0xf8) != 0x08 && (TWSR & 0xf8) != 0x10){
		return -1;
	}
	// Load _i2c_addr and send
	TWDR = _i2c_addr << 1;
	TWCR = 1 << 7 | 1 << 2;
	// Check for ACK
	while(!(TWCR & (1 << 7))){
		// wait for i2c operation
	}
	if(!(TWSR & 0x08 && TWSR ^ 0x30)){
		return -1;
	}
	// Load reg and send
	TWDR = reg;
	TWCR = 1 << 7 | 1 << 2;
	// check for ACK
	while(!(TWCR & (1 << 7))){
		// wait for i2c op
	}
	if(!(TWSR & 0x08 && TWSR ^ 0x30)){
		return -1;
	}
	// send start
	TWCR = 1 << 7 | 1 << 5 | 1 << 2;
	// Check for master ctrl
	while(!(TWCR & (1 << 7))){
		// wait for i2c op
	}
	if((TWSR & 0xf8) != 0x08 && (TWSR & 0xf8) != 0x10){
		return -1;
	}
	// load _i2c_addr + read bit and send
	TWDR = _i2c_addr << 1 | 0x01;
	TWCR = 1 << 7 | 1 << 2;
	// wait for ack
	while(!(TWCR & (1 << 7))){
		// wait for i2c op
	}
	if(!((TWSR & 0xf8) == 0x40)){
		return -1;
	}
	// clear 7
	TWCR = 1 << 7 | 1 << 2;
	// wait for nack
	while(!(TWCR & (1 << 7))){
		// wait for i2c op
	}
	if(!((TWSR & 0xf8) == 0x58)){
		return -1;
	}
	uint8_t temp = TWDR;
	// send stop
	TWCR = 1 << 7 | 1 << 4 | 1 << 2;
	return temp;
}
int MPU9150_init(){
	_i2c_addr = ADDR1;
	_i2c_write(MPU9150_PWR_MGMT_1, 1 << 0);
	// Configure gyro to +- 250 deg / sec
	uint8_t temp = _i2c_read(MPU9150_GYRO_CONFIG);
	_i2c_write(MPU9150_GYRO_CONFIG, temp & ~(1 << 3 | 1 << 4));
	
	// configure acc to +- 2g
	temp = _i2c_read(MPU9150_ACCEL_CONFIG);
	_i2c_write(MPU9150_ACCEL_CONFIG, temp & ~(1 << 3 | 1 << 4));

	// enable mag passthrough
	_i2c_write(MPU9150_I2C_INT_PIN_CFG, 1 << 1);

	// configure mag
	_i2c_addr = ADDR2;
	_i2c_write(MPU9150_MAG_CNTL, 1 << 0);
	_i2c_addr = ADDR1;

	// configure i2c master
	_i2c_write(MPU9150_I2C_MST_CTRL, 1 << 4 | 8);

	// configure i2c slave 0
	_i2c_write(MPU9150_I2C_SLV0_ADDR, ADDR2 | 1 << 7);
	_i2c_write(MPU9150_I2C_SLV0_REG, 0x03);
	_i2c_write(MPU9150_I2C_SLV0_CTRL, 6 | 1 << 7);

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
	byte_H = _i2c_read(MPU9150_ACCEL_XOUT_H);
	byte_L = _i2c_read(MPU9150_ACCEL_XOUT_L);
	accelX = byte_H << 8 | byte_L;

	// accelY
	byte_H = _i2c_read(MPU9150_ACCEL_YOUT_H);
	byte_L = _i2c_read(MPU9150_ACCEL_YOUT_L);
	accelY = byte_H << 8 | byte_L;

	// accelZ
	byte_H = _i2c_read(MPU9150_ACCEL_ZOUT_H);
	byte_L = _i2c_read(MPU9150_ACCEL_ZOUT_L);
	accelZ = byte_H << 8 | byte_L;

	// gyroX
	byte_H = _i2c_read(MPU9150_GYRO_XOUT_H);
	byte_L = _i2c_read(MPU9150_GYRO_XOUT_L);
	gyroX = byte_H << 8 | byte_L;

	// gyroY
	byte_H = _i2c_read(MPU9150_GYRO_YOUT_H);
	byte_L = _i2c_read(MPU9150_GYRO_YOUT_L);
	gyroY = byte_H << 8 | byte_L;

	// gyroZ
	byte_H = _i2c_read(MPU9150_GYRO_ZOUT_H);
	byte_L = _i2c_read(MPU9150_GYRO_ZOUT_L);
	gyroZ = byte_H << 8 | byte_L;

	// magX
	byte_L = _i2c_read(MPU9150_EXT_SENS_DATA_00);
	byte_H = _i2c_read(MPU9150_EXT_SENS_DATA_01);
	magX = byte_H << 8 | byte_L;

	// magY
	byte_L = _i2c_read(MPU9150_EXT_SENS_DATA_02);
	byte_H = _i2c_read(MPU9150_EXT_SENS_DATA_03);
	magY = byte_H << 8 | byte_L;

	// magZ
	byte_L = _i2c_read(MPU9150_EXT_SENS_DATA_04);
	byte_H = _i2c_read(MPU9150_EXT_SENS_DATA_05);
	magZ = byte_H << 8 | byte_L;
	return 0;
}
