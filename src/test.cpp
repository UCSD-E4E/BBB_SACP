//#define __PRINT_XYZ_VALS
#define __MEASURE_ACCEL_PERF
#define _USE_MATH_DEFINES

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
//	sensor.calibrate();

#ifdef __PRINT_XYZ_VALS
	int printCounter = 0;
	while(printCounter < 32){
		sensor.getSensorState();
		printf("%.6f\t%.6f\t%.6f\t", sensor.getAccelX(), sensor.getAccelY(), sensor.getAccelZ());
		float mag = sensor.getAccelX() * sensor.getAccelX() + sensor.getAccelY() * sensor.getAccelY() + sensor.getAccelZ() * sensor.getAccelZ();
		printf("%.6f\n", sqrt(mag));
		printCounter++;
	}
#endif

#ifdef __MEASURE_ACCEL_PERF
	while(1){
		sensor.getSensorState();
		float m[3] = {sensor.getAccelX(), sensor.getAccelY(), sensor.getAccelZ()};
		float m_OLD[3] = {sensor.getAccelX(), sensor.getAccelY(), sensor.getAccelZ()};
		float s[3] = {0};
		for(int i = 0; i < 256; i++){
			sensor.getSensorState();
			m[0] = m[0] + (sensor.getAccelX() - m[0]) / (i + 1);
			m[1] = m[1] + (sensor.getAccelY() - m[1]) / (i + 1);
			m[2] = m[2] + (sensor.getAccelZ() - m[2]) / (i + 1);

			s[0] = s[0] + (sensor.getAccelX() - m_OLD[0]) * (sensor.getAccelX() - m[0]);
			s[1] = s[1] + (sensor.getAccelY() - m_OLD[1]) * (sensor.getAccelY() - m[1]);
			s[2] = s[2] + (sensor.getAccelZ() - m_OLD[2]) * (sensor.getAccelZ() - m[2]);

			m_OLD[0] = m[0];
			m_OLD[1] = m[1];
			m_OLD[2] = m[2];
		}

		cout << "Average: " << m[0] << ", " << m[1] << ", " << m[2] << endl;
		cout << "Variance: " << s[0] / 256 << ", " << s[1] / 256 << ", " << s[2] / 256 << endl;
		cout << "Std Dev: " << sqrt(s[0] / 63) << ", " << sqrt(s[1] / 63) << ", " << sqrt(s[2] / 63) << endl;
		cout << "Magnitude: " << sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]) << endl;
		cout << "Direction: " << (atan2(-m[1], m[2]) * 180) / M_PI << ", " << (atan2(m[0], sqrt(m[1] * m[1] + m[2] * m[2])) * 180.0 / M_PI) << endl;
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
