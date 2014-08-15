#include "MPU9150.h"

int main(){
	MPU9150 sensor(1);
	int result;
	result = sensor.initialize();
}