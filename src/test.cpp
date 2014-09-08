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
	if((result = sensor.getSensorState())){
		cout << "Sensor Data Retrieval Failed!" << endl;
		return result;
	}
	cout << sensor.getAccelX() << endl;
	cout << sensor.getAccelY() << endl;
	cout << sensor.getAccelZ() << endl;
	float magnitude = sensor.getAccelX() * sensor.getAccelX() + sensor.getAccelY() * sensor.getAccelY() + sensor.getAccelZ() * sensor.getAccelZ();
	magnitude = sqrt(magnitude);
	cout << magnitude << endl;
	
	// if(!sensor.calibrate()){
		// return 1;
	// }
	// sensor.getSensorState();
	// cout << sensor.getAccelX() << endl;
	// cout << sensor.getAccelY() << endl;
	// cout << sensor.getAccelZ() << endl;
	// magnitude = sensor.getAccelX() * sensor.getAccelX() + sensor.getAccelY() * sensor.getAccelY() + sensor.getAccelZ() * sensor.getAccelZ();
	// magnitude = sqrt(magnitude);
	// cout << magnitude << endl;
	
	// Print out sensor data
	while(1){
		if(sensor.getSensorState()){
			return 1;
		}
		magnitude = sensor.getAccelX() * sensor.getAccelX() + sensor.getAccelY() * sensor.getAccelY() + sensor.getAccelZ() * sensor.getAccelZ();
		magnitude = sqrt(magnitude);
		printf("%.3f\t$.3f\t%.3f\t%.3f\n", sensor.getAccelX(), sensor.getAccelY(), sensor.getAccelZ(), magnitude);
	}
	return 0;
}
