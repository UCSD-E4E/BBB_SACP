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
#include "MPU9150.h"
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
	int file;
	if((file = open(namebuf, O_RDWR)) < 0){
		cout << "Failed to open I2C bus " << namebuf << endl;
		return 1;
	}
	// Set I2C bus control
	if(ioctl(file, I2C_SLAVE, I2CAddress) < 0){
			cout << "I2C_SLAVE address " << hex << I2CAddress << dec 
					<< " failed..." << endl;
		return 2;
	}
	
	// Wake up MPU9150: write 0x00 to MPU9150_PWR_MGMT_1
	char i2cbuf[64] = {0};	// initialize empty buffer
	i2cbuf[0] = MPU9150_PWR_MGMT_1;
	i2cbuf[1] = 0x00;
	if(write(file, i2cbuf, 2) != 1){	// attempt to write 0x00 to 
			// MPU9150_PWR_MGMT_1
		// Failed transaction
		printf("Failed to write 0x%02hhx to 0x%02hhx at 0x%02x on %s\n", i2cbuf[1], 
				i2cbuf[0], I2CAddress, I2CBus);
		cout << strerror(errno) << endl << endl;
		return 3;
	}
	
	// Check device identity
	// Write register address
	i2cbuf[0] = MPU9150_WHO_AM_I;	// read from MPU9150_WHO_AM_I
	if(write(file, i2cbuf, 1) != 1){	// attempt to push read register address
		// Failed transaction
		cout << "Failed to write MPU9150_WHO_AM_I to I2C device!" << endl;
		cout << strerror(errno) << endl << endl;
		return (3);
	}
	
	// Read MPU9150_WHO_AM_I register
	if(read(file, i2cbuf, 1) != 1){	// attempt to pull read register
		// Failed transaction
		cout << "Failed to read register contents from I2C device!" <<  endl;
		cout << strerror(errno) << endl << endl;
		return (4);
	}
	
	if(i2cbuf[0] != 0x68){
		// Failed to confirm device
		cout << "Failed to confirm device identity!" << endl;
		cout << "Device identity: 0x" << hex << i2cbuf[0] << dec << endl 
				<< endl;
		return 5;
	}
	
	// Enable magnetometer
	i2cbuf[0] = MPU9150_I2C_INT_PIN_CFG;
	i2cbuf[1] = (1 << 1);	// enable I2C bypass
	if(write(file, i2cbuf, 2) != 1){	// attempt to write data
		// Failed transaction
		printf("Failed to write 0x%02hhx to 0x%02hhx at 0x%02x on %s\n", i2cbuf[1], 
				i2cbuf[0], I2CAddress, I2CBus);
		cout << strerror(errno) << endl << endl;
		return 3;
	}
	
	// Access magnetometer at MPU9150_MAG_ADDR
	int magFile;
	if((magFile = open(namebuf, O_RDWR)) < 0){
		cout << "Failed to open I2C bus " << namebuf << endl;
		return 1;
	}
	// Set I2C bus control
	if(ioctl(magFile, I2C_SLAVE, MPU9150_MAG_ADDR) < 0){
			cout << "I2C_SLAVE address 0x0C failed..." << endl;
		return 2;
	}
	
	i2cbuf[0] = MPU9150_MAG_CNTL;
	i2cbuf[1] = (1 << 0);	// enable Magnetometer to single measurement
	if(write(magFile, i2cbuf, 2) != 1){	// attempt to write data
		// Failed transaction
		printf("Failed to write 0x%02hhx to 0x%02hhx at 0x%02x on %s\n", i2cbuf[1], 
				i2cbuf[0], I2CAddress, I2CBus);
		cout << strerror(errno) << endl << endl;
		return 3;
	}
	
	// verify magnetometer
	i2cbuf[0] = MPU9150_MAG_WIA;
	if(write(magFile, i2cbuf, 1) !=1){	// attempt to write read register addr
		// Failed transaction
		cout << "Failed to write read register address to I2C magnetometer!" << endl;
		cout << strerror(errno) << endl << endl;
		return (3);
	}
	
	// Read MPU0150_MAG_WIA register
	if(read(magFile, i2cbuf, 1) != 1){	// attempt to pull read register
		// Failed transaction
		cout << "Failed to read register contents from I2C device!" <<  endl;
		cout << strerror(errno) << endl << endl;
		return (4);
	}
	
	if(i2cbuf[0] != 0x48){
		// Failed to confirm device
		cout << "Failed to confirm device identity!" << endl;
		cout << "Device identity: 0x" << hex << i2cbuf[0] << dec << endl 
				<< endl;
		return 5;
	}
}

/**
 * MPU9105 constructor.  Accepts for arguments the I2C bus.  Assumes default
 * address of 0x68.
 */
MPU9150::MPU9150(int bus){
	I2CBus = bus;
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
	return 0;
}
