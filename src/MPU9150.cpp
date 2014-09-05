/**
 * MPU9150 Class Implementation
 * 
 */

// Includes
#include <cstdio>
#include <iostream>
#include <sys/ioctl.h>
#include <iomanip>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include "MPU9150.hpp"
#include <cstdlib>
#include <cstdint>
#include <cinttypes>
using namespace std;

// defines

// Function Implementations
/**
 * Initialization function.  Initializes MPU9150 to a working state.
 * Sets register at 0x6B to 0x00.  Sets pass-through mode by setting 0x37 to
 * 0x02.
 *
 * @return	0	Normal return, no problems
 *			1	Error: Failed to open I2C bus
 *			2	Error: Failed to access I2C device
 *			3	Error: Failed to write to I2C device
 *			4	Error: Failed to read from I2C device
 *			5	Error: Failed to verify device identity
 */
int MPU9150::initialize(){
	// Open I2C bus
	char namebuf[64];
	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus);
	if((mpuFile = open(namebuf, O_RDWR)) < 0){
		cout << "Failed to open I2C bus " << namebuf << endl;
		return 1;
	}
	// Set I2C bus control
	if(ioctl(mpuFile, I2C_SLAVE, I2CAddress) < 0){
			cout << "I2C_SLAVE address " << hex << I2CAddress << dec 
					<< " failed..." << endl;
		return 2;
	}
	int result;
	// Wake up MPU9150: write 0x00 to MPU9150_PWR_MGMT_1
	// configure x-axis gyroscope as clock source
	if((result = writeByte(mpuFile, MPU9150_PWR_MGMT_1, 0x00 | (1 << 0)))){
		return result;
	}
	
	// Check device identity
	uint8_t deviceID;
	if((result = readByte(mpuFile, MPU9150_WHO_AM_I, &deviceID))){
		return result;
	}
	
	if(deviceID != 0x68){
		// Failed to confirm device
		cout << "Failed to confirm device identity!" << endl;
		printf("Device identity: 0x%" PRIu8 "\n\n", deviceID);
		return 5;
	}
	
	// configure device
	// set gyro to +/- 250 deg/sec
	if((result = writeBits(mpuFile, MPU9150_GYRO_CONFIG, ~(0x03 << 3), 0x18))){
		return result;
	}
	// set accel to +/- 2g
	if((result = writeBits(mpuFile, MPU9150_ACCEL_CONFIG, ~(0x03 << 3), 0x18))){
		return result;
	}
	
	// Enable magnetometer
	if((result = writeByte(mpuFile, MPU9150_I2C_INT_PIN_CFG, (1 << 1)))){
		return result;
	}
	
	// Access magnetometer at MPU9150_MAG_ADDR
	if((magFile = open(namebuf, O_RDWR)) < 0){
		cout << "Failed to open I2C bus " << namebuf << endl;
		return 1;
	}
	// Set I2C bus control
	if(ioctl(magFile, I2C_SLAVE, MPU9150_MAG_ADDR) < 0){
			cout << "I2C_SLAVE address 0x0C failed..." << endl;
		return 2;
	}
	
	// Configure magnetometer
	if((result = writeByte(magFile, MPU9150_MAG_CNTL, (1 << 0)))){
		return result;
	}
	
	// verify magnetometer
	if((result = readByte(magFile, MPU9150_MAG_WIA, &deviceID))){
		return result;
	}
	if(deviceID != 0x48){
		// Failed to confirm device
		cout << "Failed to confirm device identity!" << endl;
		printf("Device identity: 0x%" PRIu8 "\n\n", deviceID);
		return 5;
	}
	return 0;
}

/**
 * MPU9105 constructor.  Accepts for arguments the I2C bus.  Assumes default
 * address of 0x68.
 */
MPU9150::MPU9150(int bus){
	I2CBus = bus;
	I2CAddress = 0x68;
}

/**
 * MPU9150 alternate constructor.  Accepts as arguments the I2C bus and bus
 * address.
 */
MPU9150::MPU9150(int bus, int address){
	I2CBus = bus;
	I2CAddress = address;
}

/**
 * Update function.  Records full sensor state.
 */
int MPU9150::getSensorState(){
	uint8_t buffer;
	int result;
	if((result = readByte(mpuFile, MPU9150_ACCEL_XOUT_H, &buffer))){
		return result;
	}
	accel_X = buffer << 8;
	if((result = readByte(mpuFile, MPU9150_ACCEL_XOUT_L, &buffer))){
		return result;
	}
	accel_X |= buffer;
	if((result = readByte(mpuFile, MPU9150_ACCEL_YOUT_H, &buffer))){
		return result;
	}
	accel_Y = buffer << 8;
	if((result = readByte(mpuFile, MPU9150_ACCEL_YOUT_L, &buffer))){
		return result;
	}
	accel_Y |= buffer;
	if((result = readByte(mpuFile, MPU9150_ACCEL_ZOUT_H, &buffer))){
		return result;
	}
	accel_Z = buffer << 8;
	if((result = readByte(mpuFile, MPU9150_ACCEL_ZOUT_L, &buffer))){
		return result;
	}
	accel_Z |= buffer;
	
	if((result = readByte(mpuFile, MPU9150_TEMP_OUT_H, &buffer))){
		return result;
	}
	temp = buffer << 8;
	if((result = readByte(mpuFile, MPU9150_TEMP_OUT_L, &buffer))){
		return result;
	}
	temp |= buffer;
	
	if((result = readByte(mpuFile, MPU9150_GYRO_XOUT_H, &buffer))){
		return result;
	}
	gyro_X = buffer << 8;
	if((result = readByte(mpuFile, MPU9150_GYRO_XOUT_L, &buffer))){
		return result;
	}
	gyro_X |= buffer;
	if((result = readByte(mpuFile, MPU9150_GYRO_YOUT_H, &buffer))){
		return result;
	}
	gyro_Y = buffer << 8;
	if((result = readByte(mpuFile, MPU9150_GYRO_YOUT_L, &buffer))){
		return result;
	}
	gyro_Y |= buffer;
	if((result = readByte(mpuFile, MPU9150_GYRO_ZOUT_H, &buffer))){
		return result;
	}
	gyro_Z = buffer << 8;
	if((result = readByte(mpuFile, MPU9150_GYRO_ZOUT_L, &buffer))){
		return result;
	}
	gyro_Z |= buffer;
	
	if((result = readByte(magFile, MPU9150_MAG_HXH, &buffer))){
		return result;
	}
	mag_X = buffer << 8;
	if((result = readByte(magFile, MPU9150_MAG_HXL, &buffer))){
		return result;
	}
	mag_X |= buffer;
	if((result = readByte(magFile, MPU9150_MAG_HYH, &buffer))){
		return result;
	}
	mag_Y = buffer << 8;
	if((result = readByte(magFile, MPU9150_MAG_HYL, &buffer))){
		return result;
	}
	mag_Y |= buffer;
	if((result = readByte(magFile, MPU9150_MAG_HZH, &buffer))){
		return result;
	}
	mag_Z = buffer << 8;
	if((result = readByte(magFile, MPU9150_MAG_HZL, &buffer))){
		return result;
	}
	mag_Z |= buffer;
	return 0;
}

/**
 * @return	0	Normal return, no problems
 *			1	Error: Failed to open I2C bus
 *			2	Error: Failed to access I2C device
 *			3	Error: Failed to write to I2C device
 *			4	Error: Failed to read from I2C device
 *			5	Error: Failed to verify device identity
 */
int MPU9150::writeByte(int device, uint8_t regAddr, uint8_t value){
	uint8_t i2cbuf[2] = {regAddr, value};	// initialize empty buffer
	if(write(device, i2cbuf, 2) != 2){
		// Failed transaction
		printf("Failed to write 0x%02X to 0x%02X at 0x%02X on /dev/i2c-%i\n", i2cbuf[1], i2cbuf[0], (device == mpuFile) ? I2CAddress : MPU9150_MAG_ADDR, I2CBus);
		cout << strerror(errno) << endl << endl;
		return 3;
	}
	return 0;
}

int MPU9150::writeBits(int device, uint8_t regAddr, uint8_t value,
		uint8_t bitmask){
	uint8_t i2cbuf[2] = {regAddr};
	if(write(device, i2cbuf, 1) != 1){
		printf("Failed to write 0x%02X to 0x%02X on /dev/i2c-%i\n", i2cbuf[0], (device == mpuFile) ? I2CAddress : MPU9150_MAG_ADDR, I2CBus);
		cout << strerror(errno) << endl << endl;
		return 3;
	}
	if(read(device, i2cbuf, 1) != 1){
		printf("Failed to read from 0x%02X at 0x%02X on /dev/i2c-%i\n", regAddr, (device == mpuFile) ? I2CAddress : MPU9150_MAG_ADDR, I2CBus);
		cout << strerror(errno) << endl << endl;
		return 4;
	}
	i2cbuf[1] = (i2cbuf[0] & ~bitmask) | value;
	i2cbuf[0] = regAddr;
	if(write(device, i2cbuf, 2) != 2){
		// Failed transaction
		printf("Failed to write 0x%02X to 0x%02X at 0x%02X on /dev/i2c-%i\n", i2cbuf[1], i2cbuf[0], (device == mpuFile) ? I2CAddress : MPU9150_MAG_ADDR, I2CBus);
		cout << strerror(errno) << endl << endl;
		return 3;
	}
	return 0;
}

int MPU9150::readByte(int device, uint8_t regAddr, uint8_t* value){
	if(write(device, &regAddr, 1) != 1){
		printf("Failed to write 0x%02X to 0x%02X on /dev/i2c-%i\n", regAddr, (device == mpuFile) ? I2CAddress : MPU9150_MAG_ADDR, I2CBus);
		cout << strerror(errno) << endl << endl;
		return 3;
	}
	if(read(device, value, 1) != 1){
		printf("Failed to read from 0x%02X at 0x%02X on /dev/i2c-%i\n", regAddr, (device == mpuFile) ? I2CAddress : MPU9150_MAG_ADDR, I2CBus);
		cout << strerror(errno) << endl << endl;
		return 4;
	}
	return 0;
}

float MPU9150::getAccelX(){
	return accel_X / 16384.0;
}

float MPU9150::getAccelY(){
	return accel_Y / 16384.0;
}

float MPU9150::getAccelZ(){
	return accel_Z / 16384.0;
}

float MPU9150::getGyroX(){
	return gyro_X / 131.072;
}

float MPU9150::getGyroY(){
	return gyro_Y / 131.072;
}

float MPU9150::getGyroZ(){
	return gyro_Z / 131.072;
}

float MPU9150::getMagX(){
	return mag_X / 3.3319;
}

float MPU9150::getMagY(){
	return mag_Y / 3.3319;
}

float MPU9150::getMagZ(){
	return mag_Z / 3.3319;
}