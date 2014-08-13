/**
 * MPU9150 Class Implementation
 * 
 */

// Includes
#include <cstdio>
#include <iostream>
#include <sys/ioctl.h>
#include "MPU9150.h"
using namespace std;

// defines

// Function Implementations
int MPU9150::initialize(){
	char namebuf[64];
	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus);
	int file;
	if((file = open(namebuf, O_RDWR)) < ){
		cout << "Failed to open MPU9150 on" << namebuf << endl;
		return 1;
	}
	if(ioctl(file, I2C_SLAVE, I2CAddress) < 0){
			cout << "I2C_SLAVE address " << I2CAddress << " failed..." << endl;
		return 2;
	}