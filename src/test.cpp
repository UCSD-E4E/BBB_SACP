#define __PRINT_XYZ_VALS

#include "MPU9150.hpp"
#include <iostream>
#include <cstdio>
#include <cmath>
using namespace std;

int main(){
	MPU9150 sensor = MPU9150(1);
	int result;
	if((result = sensor.initialize())){
		cout << "Initialization Failed!" << endl;
		return result;
	}
	sensor.calibrate();

#ifdef __PRINT_XYZ_VALS
	while(1){
		sensor.getSensorState();
		printf("%.6f\t%.6f\t%.6f\t", sensor.getAccelX(), sensor.getAccelY(), sensor.getAccelZ());
		float mag = sensor.getAccelX() * sensor.getAccelX() + sensor.getAccelY() * sensor.getAccelY() + sensor.getAccelZ() * sensor.getAccelZ();
		printf("%.6f\n", sqrt(mag));
	}
#endif
	
#ifdef __SELF_TEST	
	// Z Self Test
	// Register 28, bit 5
	// Register 15[5-7], 16[0-1]
	// Result is equal to enabled output - disabled output
	// enabled result stored in 59-64
	// Set to 8g
	
	sensor.writeByte(sensor.mpuFile, MPU9150_ACCEL_CONFIG, (2 << 3) | (1 << 5));	// set 8g and z self test
	// Get self test result
	uint8_t result1 = 0;
	sensor.readByte(sensor.mpuFile, MPU9150_SELF_TEST_Z, &result1);
	uint8_t self_Test_Z1 = (result1 | 0xE0) >> 3;
	sensor.readByte(sensor.mpuFile, MPU9150_SELF_TEST_A, &result1);
	self_Test_Z1 |= (result1 | 0x03);
	printf("%d\n", self_Test_Z1);
	
	// disable self test
	sensor.writeByte(sensor.mpuFile, MPU9150_ACCEL_CONFIG, (2 << 3));
	sensor.readByte(sensor.mpuFile, MPU9150_ACCEL_ZOUT_H, &result1);
	int16_t zOutput = result1 << 8;
	sensor.readByte(sensor.mpuFile, MPU9150_ACCEL_ZOUT_L, &result1);
	zOutput |= result1;
	printf("%d\n", zOutput);
	printf("%d\n", self_Test_Z1 - zOutput);
#endif
	return 0;
}
