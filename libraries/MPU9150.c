#ifndef _MPU9150
#define _MPU9150
#define DEBUG(X) printf(X)
#include "MPU9150.h"
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>

uint8_t _i2c_addr = 0;

int _i2c_write(uint8_t dest, uint8_t val){
	// Set Start condition
	TWCR |= (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
	// Check for master control
	while((TWSR & 0xf8) != 0x08){
		// wait for completion of I2C operation
	}
	// Load _i2c_addr and send
	uint8_t twi_slarw = _i2c_addr << 1;
	TWDR = twi_slarw;
	printf("TWDR = 0x%x\n", TWDR);
	TWCR = 1 << TWINT | 1 << TWEN | 1 << TWIE | 1 << TWEA;
	DEBUG("Sending slaw\n");
	// Check for ACK
	while((TWSR & 0xf8) != 0x18){
		// wait for i2c operation
	}
	DEBUG("done\n");
	printf("0x%x", TWSR);
	// Load dest and send
	TWDR = dest;
	TWCR = 1 << TWEN | 1 << TWIE | 1 << TWINT | 1 << TWEA;
	// check for ACK
	while(!(TWCR & 0x28)){
		// wait for i2c op
	}
	// Load val and send
	TWDR = val;
	TWCR = 1 << TWEN | 1 << TWIE | 1 << TWINT | 1 << TWEA;
	// check for ack
	while(!(TWCR & 0x28)){
		// wait for i2c op
	}
	// set STOP cond
	TWCR = 1 << TWEN | 1 << TWIE | 1 << TWEA | 1 << TWINT | 1 << TWSTO;
	return 0;
}

uint8_t _i2c_read(uint8_t reg){
	printf("I2C Slave Address = 0x%x\n", _i2c_addr);
	// Set Start condition
	TWCR = 1 << TWEN | 1 << TWIE | 1 << TWEA | 1 << TWINT | 1 << TWSTA;
	// Check for master control
	while(!(TWSR & 0x08)){
		// wait for completion of I2C operation
	}
	// Load _i2c_addr and send
	TWDR = _i2c_addr << 1;
	TWCR = 1 << TWEN | 1 << TWIE | 1 << TWINT | 1 << TWEA;
	// Check for ACK
	while(!(TWCR & 0x18)){
		// wait for i2c operation
	}
	// Load reg and send
	TWDR = reg;
	TWCR = 1 << TWEN | 1 << TWIE | 1 << TWINT | 1 << TWEA;
	// check for ACK
	while(!(TWCR & 0x28)){
		// wait for i2c op
	}
	// send start
	TWCR = 1 << TWEN | 1 << TWIE | 1 << TWINT | 1 << TWEA | 1 << TWSTA;
	// Check for master ctrl
	while(!(TWCR & 0x08)){
		// wait for i2c op
	}
	// load _i2c_addr + read bit and send
	TWDR = _i2c_addr << 1 | 1;
	TWCR = 1 << TWEN | 1 << TWIE | 1 << TWINT | 1 << TWEA;
	// wait for ack
	while(!(TWCR & 0x40)){
		// wait for i2c op
	}
	// ack, prepare for nack
	TWCR =1 << TWEN | 1 << TWIE | 1 << TWINT;
	// wait for nack
	while(!(TWCR & 0x58)){
		// wait for i2c op
	}
	uint8_t temp = TWDR;
	// send stop
	TWCR = 1 << TWEN | 1 << TWIE | 1 << TWEA | 1 << TWINT | 1 << TWSTO;
	return temp;
}

int MPU9150_init(){
	DEBUG("Initializing MPU9150...\n");
	// Initialize TWI
	DDRC |= (1 << DDC5) | (1 << DDC4);	// set C5 and C4 as output pins
	PORTC |= (1 << PORTC5) | (1 << PORTC4);	// activate pullup resistors
	TWSR &= ~(1 << TWPS0) & ~(1 << TWPS1);
	TWBR = 72;
	TWCR = (1 << TWEN) | 1 << TWIE | 1 << TWEA;

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
#endif
