#include "MPU9150.hpp"
#include <iostream>
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
	
	if(!sensor.calibrate()){
		return 1;
	}
	sensor.getSensorState();
	cout << sensor.getAccelX() << endl;
	cout << sensor.getAccelY() << endl;
	cout << sensor.getAccelZ() << endl;
	magnitude = sensor.getAccelX() * sensor.getAccelX() + sensor.getAccelY() * sensor.getAccelY() + sensor.getAccelZ() * sensor.getAccelZ();
	magnitude = sqrt(magnitude);
	cout << magnitude << endl;
	return 0;
}
