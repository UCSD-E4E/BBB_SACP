#include "MPU9150.h"
#include <iostream>
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
	
	if(!calibrate()){
		return 1;
	}
	return 0;
}
