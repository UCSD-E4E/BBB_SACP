#include "MPU9150.h"
using namespace std;

int main(){
	MPU9150 sensor = MPU9150(1);
	int result;
	result = sensor.initialize();
	return result;
}
