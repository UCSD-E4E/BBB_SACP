#ifndef _MPU9150
#define _MPU9150
#define DEBUG(X) printf(X)
#include "MPU9150.h"
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <i2cmaster.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <math.h>

// global variables
uint8_t _i2c_addr = 0;
int i, j = 0;
float newAcc[3] = {0};
float __mag;
float halfAngAcc = 0;
float halfAngle[3][2] = {0};
float gyroQuat[4] = {1, 0, 0, 0};
float gyroRot[3] = {0};
float accQuat[4];
float rotQuat[4] = {0};
float tempQuat[4] = {0};
uint8_t byte_H;
uint8_t byte_L;
float _prevAcc[3] = {0, 0, 1};

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

float _mag(float a[3]){
	return sqrt(a[0] * a[0] + a[1]* a[1] + a[2] * a[2]);
}

float _mag4(float a[4]){
	return sqrt(a[0] * a[0] + a[1]* a[1] + a[2] * a[2] + a[3] * a[3]);
}

void  _norm(float* a){
	__mag = _mag(a);
	a[0] /= __mag;
	a[1] /= __mag;
	a[2] /= __mag;
	return;
}

void  _norm4(float* a){
	__mag = _mag(a);
	a[0] /= __mag;
	a[1] /= __mag;
	a[2] /= __mag;
	a[3] /= __mag;
	return;
}

float _dot(float a[3], float b[3]){
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

int MPU9150_init(){
	DEBUG("Initializing MPU9150...\n");
	// Initialize TWI
	i2c_init();


	DEBUG("Turning on MPU9150...");
	_i2c_addr = ADDR1;
	_i2c_write(MPU9150_PWR_MGMT_1, 1 << 0);
	DEBUG("done\n");

	// Sanity checking MPU9150
	uint8_t id = _i2c_read(MPU9150_WHO_AM_I);
	if(id != 0x68){
		printf("FAILED TO RX MPU9150\n");
		return 1;
	}else{
	}

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

	/*
	// enable mag passthrough
	DEBUG("Enabling magnetometer passthrough...");
	_i2c_write(MPU9150_I2C_INT_PIN_CFG, 1 << 1);
	DEBUG("done\n");

	// configure mag
	DEBUG("Configuring Magnetometer...");
	_i2c_addr = ADDR2;
	_i2c_write(MPU9150_MAG_CNTL, 1 << 0);
	DEBUG("Activated Mag!\n");
	id = _i2c_read(MPU9150_MAG_WIA);
	if(id != 0x48){
	printf("Failed to contact Mag!\n");
	return 1;
	}else{
	printf("ID'd Mag!\n");
	}*/
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

	MPU9150_Read();

	// initialize DCM matrix
	beta[0] = 99;
	beta[1] = 70;
	beta[2] = 389;
	beta[3] = 16458;
	beta[4] = 16356;
	beta[5] = 16461;
	beta[9] = beta[10] = beta[11] = 131;
	beta[6] = 0;
	beta[7] = 0;
	beta[8] = 0;
	for(i = 0; i < 32; i++){
		MPU9150_Read();
		beta[6] += gyroX;
		beta[7] += gyroY;
		beta[8] += gyroZ;
	}
	beta[6] /= -32;
	beta[7] /= -32;
	beta[8] /= -32;

	newAcc[0] = (accelX + beta[0]) / (float)beta[3];
	newAcc[1] = (accelY + beta[1]) / (float)beta[4];
	newAcc[2] = (accelZ + beta[2]) / (float)beta[5];
	_norm(newAcc);

	halfAngAcc = acos(_dot(newAcc, _prevAcc)) / 2;

	globalQuat[0] = cos(halfAngAcc);
	globalQuat[1] = (newAcc[1] * _prevAcc[2] - newAcc[2] * _prevAcc[1]) * sin(halfAngAcc);
	globalQuat[2] = (newAcc[2] * _prevAcc[0] - newAcc[0] * _prevAcc[2]) * sin(halfAngAcc);
	globalQuat[3] = (newAcc[0] * _prevAcc[1] - newAcc[1] * _prevAcc[0]) * sin(halfAngAcc);

	_norm4(globalQuat);

	for(i = 0; i< 4; i++){
		printf("%3.3f\t", globalQuat[i]);
	}
	printf("\n");

	// Initialize DCM weight
	CFW[0] = 0.02;
	CFW[1] = 0.98;

	return 0;
}

void _mulQuat(float* a, float* b, float* ret){
	ret[0] = (a[0] * b[0]) - (a[1] * b[1]) - (a[2] * b[2]) - (a[3] * b[3]);
	ret[1] = (a[0] * b[1]) + (a[1] * b[0]) + (a[2] * b[3]) - (a[3] * b[2]);
	ret[2] = (a[0] * b[2]) - (a[1] * b[3]) + (a[2] * b[0]) + (a[3] * b[1]);
	ret[3] = (a[0] * b[3]) + (a[1] * b[2]) - (a[2] * b[1]) + (a[3] * b[0]);
	return;
}

void _copyQuat(float* src, float* tgt){
	for(i = 0; i < 4; i++){
		tgt[i] = src[i];
	}
	return;
}

void update_DCM(float t){
	MPU9150_Read();

	// gyroscope rotation quaternion
	gyroRot[0] = t * (gyroX + beta[6]) / (float)beta[9] * M_PI / 180;
	gyroRot[1] = t * (gyroY + beta[7]) / (float)beta[10] * M_PI / 180;
	gyroRot[2] = t * (gyroZ + beta[8]) / (float)beta[11] * M_PI / 180;

	halfAngle[0][0] = cos(gyroRot[0] / 2);
	halfAngle[1][0] = cos(gyroRot[1] / 2);
	halfAngle[2][0] = cos(gyroRot[2] / 2);
	halfAngle[0][1] = sin(gyroRot[0] / 2);
	halfAngle[1][1] = sin(gyroRot[1] / 2);
	halfAngle[2][1] = sin(gyroRot[2] / 2);

	gyroQuat[0] = halfAngle[0][0] * halfAngle[1][0] * halfAngle[2][0] + halfAngle[0][1] * halfAngle[1][1] * halfAngle[2][1];
	gyroQuat[1] = halfAngle[0][1] * halfAngle[1][0] * halfAngle[2][0] - halfAngle[0][0] * halfAngle[1][1] * halfAngle[2][1];
	gyroQuat[2] = halfAngle[0][0] * halfAngle[1][1] * halfAngle[2][0] + halfAngle[0][1] * halfAngle[1][0] * halfAngle[2][1];
	gyroQuat[3] = halfAngle[0][0] * halfAngle[1][0] * halfAngle[2][1] - halfAngle[0][1] * halfAngle[1][1] * halfAngle[2][0];

	//Accelerometer
	newAcc[0] = (accelX + beta[0]) / (float)beta[3];
	newAcc[1] = (accelY + beta[1]) / (float)beta[4];
	newAcc[2] = (accelZ + beta[2]) / (float)beta[5];
	_norm(newAcc);

	halfAngAcc = acos(_dot(newAcc, _prevAcc)) / 2;

	accQuat[0] = cos(halfAngAcc);
	accQuat[1] = (newAcc[1] * _prevAcc[2] - newAcc[2] * _prevAcc[1]) * sin(halfAngAcc);
	accQuat[2] = (newAcc[2] * _prevAcc[0] - newAcc[0] * _prevAcc[2]) * sin(halfAngAcc);
	accQuat[3] = (newAcc[0] * _prevAcc[1] - newAcc[1] * _prevAcc[0]) * sin(halfAngAcc);
//	_norm(accQuat + 1);

	_prevAcc[0] = newAcc[0];
	_prevAcc[1] = newAcc[1];
	_prevAcc[2] = newAcc[2];

	// Complementary Filter
	rotQuat[0] = CFW[0] * gyroQuat[0] + CFW[1] * accQuat[0];
	rotQuat[1] = CFW[0] * gyroQuat[1] + CFW[1] * accQuat[1];
	rotQuat[2] = CFW[0] * gyroQuat[2] + CFW[1] * accQuat[2];
	rotQuat[3] = CFW[0] * gyroQuat[3] + CFW[1] * accQuat[3];
//	_norm4(rotQuat);
	_mulQuat(globalQuat, rotQuat, tempQuat);
	_copyQuat(tempQuat, globalQuat);


//	for(i = 0; i < 4; i++){
//		printf("%3.3f\t", gyroQuat[i]);
//	}
//	printf("\n");
//	
//	for(i = 0; i < 4; i++){
//		printf("%3.3f\t", accQuat[i]);
//	}
//	printf("\n\n");

}

int MPU9150_Read(){
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

	/*
	// Read from Mag
	_i2c_addr = ADDR2;
	// Activate measurement
	_i2c_write(MPU9150_MAG_CNTL, 1);
	if(_i2c_read(MPU9150_MAG_ST1)){
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
	}*/
	_i2c_addr = ADDR1;
	return 0;
}

int calibrateMPU9150(){
	int32_t sample[18] = {0};

	// get averaged samples
	// use 3 sigma bandpass
	for(int i = 0; i < 2; i++){
		const char* msg = "PUT THE DAMN ACCELEROMETER DOWN AND STOP HACKING ME!";
		switch(i){
			case 0:
				msg = "Positive Z up";
				break;
			case 1:
				msg = "Negative Z up";
				break;
			case 2:
				msg = "Positive Y up";
				break;
			case 3:
				msg = "Negative Y up";
				break;
			case 4:
				msg = "Positive X up";
				break;
			case 5:
				msg = "Negative X up";
				break;
		}
		DEBUG(msg);
		_delay_ms(5000);

		DEBUG("Calibrate: Beginning calibration\n");
		int32_t sum[3] = {0};
		int64_t sumSquares[3] = {0};
		DEBUG("Calibrate: Getting initial samples...");
		MPU9150_Read();
		int16_t initialVector[3] = {accelX, accelY, accelZ};
		for(int j = 0; j < 32; j++){
			MPU9150_Read();
			sum[0] += accelX - initialVector[0];
			sum[1] += accelY - initialVector[1];
			sum[2] += accelZ - initialVector[2];
			sumSquares[0] += (accelX - initialVector[0]) * (accelX - initialVector[0]);
			sumSquares[1] += (accelY - initialVector[1]) * (accelY - initialVector[1]);
			sumSquares[2] += (accelZ - initialVector[2]) * (accelZ - initialVector[2]);
		}
		DEBUG("done\n");

		// compute variance
		DEBUG("Calibrate: Computing variance...");
		int64_t variance[3];
		for(int j = 0; j < 3; j++){
			variance[j] = sumSquares[j] - ((sum[j] * sum[j]) / 32);
		}
		DEBUG("done\n");

		// Gather data
		DEBUG("Calibrate: Gathering calibration data...");
		int16_t data[3];
		int32_t diff[3];
		int tryCounter = 0;
		for(int j = 0; j < 32; j++){
			tryCounter++;
			MPU9150_Read();
			data[0] = accelX;
			data[1] = accelY;
			data[2] = accelZ;
			printf("%8d, %8d, %8d\n", data[0], data[1], data[2]);

			// calculate error
			diff[0] = data[0] * 32 - sum[0];
			diff[1] = data[1] * 32 - sum[1];
			diff[2] = data[2] * 32 - sum[2];

			// compare variance
			if((diff[0]^2) / 1024 < 9 * variance[0] &&
					(diff[1]^2) / 1024 < 9 * variance[1] &&
					(diff[2]^2) / 1024 < 9 * variance[2]){
				sample[i * 3 + 0] += data[0];
				sample[i * 3 + 1] += data[1];
				sample[i * 3 + 2] += data[2];
			}else{
				j--;
				if(tryCounter - j < 32){
					return 1;
				}
				continue;
			}
		}
		DEBUG("done\n");
		sample[i * 3 + 0] /= 32;
		sample[i * 3 + 1] /= 32;
		sample[i * 3 + 2] /= 32;
		printf("%8d, %8d, %8d\n", sample[i * 3 + 0], sample[i * 3 + 1], sample[i * 3 + 2]);
	}

	// Do model calibration
	DEBUG("Calibrate: Beginning model calibration...");
	int i;
	float eps = 0.00000001;
	//	int num_iteration = 500;
	float change = 100.0;
	//	int continueConst = 1;
	while(change > eps){
		compute_calibration_matrices(sample);
		find_delta();
		change = delta[0] * delta[0] + 
			delta[0] * delta[0] + 
			delta[1] * delta[1] + 
			delta[2] * delta[2] + 
			delta[3] * delta[3] / (beta[3] * beta[3]) + 
			delta[4] * delta[4] / (beta[4] * beta[4]) + 
			delta[5] * delta[5] / (beta[5] * beta[5]);

		for( i = 0; i < 6; i++){
			beta[i] -= delta[i];
		}

		reset_calibration_matrices();
	}
	DEBUG("done\n");

	return 0;
}

/**
 * Computes the calibration matrix for the particular sample.
 *
 * @param	*data	Address of the beginning of the data location, expected as a sequence of 18 int32_t variables, ordered first by axis and then by sample (6 samples of 3 axis each)
 */
void compute_calibration_matrices(int32_t data[]){
	reset_calibration_matrices();
	for(int i = 0; i < 6; i++){
		update_calibration_matrices(data + i * 3);
	}
}

/**
*/
void update_calibration_matrices(int32_t data[]){
	int j, k;
	float dx, b;
	float residual = 1.0;
	float jacobian[6];

	for(j = 0; j < 3; j++){
		b = beta[3 + j];
		dx = ((float)data[j]) / 32 - beta[j];
		residual -= b * b * dx * dx;
		jacobian[j] = 2.0 * b * b * dx;
		jacobian[3 + j] = -2.0 * b * dx * dx;
	}

	for(j = 0; j < 6; j++){
		dS[j] += jacobian[j] * residual;
		for(k = 0; k < 6; k++){
			JS[j][k] += jacobian[j] * jacobian[k];
		}
	}
}

/**
 * Zeroes dS and JS matrices
 */
void reset_calibration_matrices(){
	int j, k;
	for(j = 0; j < 6; j++){
		dS[j] = 0;
		for(k = 0; k < 6; k++){
			JS[j][k] = 0;
		}
	}
}

void find_delta(){
	int i, j, k;
	float mu;

	for(i = 0; i < 6; i++){
		for(j = i + 1; j < 6; j ++){
			mu = JS[i][j] / JS[i][i];
			if(mu != 0.0){
				dS[j] -= mu * dS[i];
				for(k = j; k < 6; k++){
					JS[k][j] -= mu * JS[k][i];
				}
			}
		}
	}

	for(i = 5; i >= 0; i--){
		dS[i] /= JS[i][i];
		JS[i][i] = 1;
		for(j = 0; j < i; j++){
			mu = JS[i][j];
			dS[j] -= mu * dS[i];
			JS[i][j] = 0;
		}
	}

	for(i = 0; i < 6; i++){
		delta[i] = dS[i];
	}
}

float getRoll(){
	//	return -1 * asin(DCMG[2][0]);
	return 180 / M_PI * atan2(2 * (globalQuat[0] * globalQuat[3] + globalQuat[1] * globalQuat[2]), pow(globalQuat[0], 2) + pow(globalQuat[1], 2) - pow(globalQuat[2], 2) - pow(globalQuat[3], 2));
}

float getPitch(){
	//	return atan2(DCMG[2][1], DCMG[2][2]);
	return -180 / M_PI * asin(2 * (globalQuat[1] * globalQuat[3] - globalQuat[0] * globalQuat[2]));
}

float getYaw(){
	//	return atan2(DCMG[1][0], DCMG[0][0]);
	return 180 / M_PI * atan2(2 * (globalQuat[0] * globalQuat[1] + globalQuat[2] * globalQuat[3]), pow(globalQuat[0], 2) - pow(globalQuat[1], 2) - pow(globalQuat[2], 2) + pow(globalQuat[3], 2));
}
#endif
