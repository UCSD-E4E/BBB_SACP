/**
 * Calibration suite for the MPU9150's magnetometer.  The purpose of this
 * program is to gather the data necessary for the included Matlab scripts to
 * accurately calculate the calibration matrix for the magnetometer.
 *
 * @author Nathan Hui
 * @date 1/15/2015
 */

///////////////////
// Include Files //
///////////////////
#include <MPU9150.h>
#include <uart.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <i2cwrap.h>
#include <i2cmaster.h>
#include <MPU9150_reg.h>
#define F_CPU 16000000
#include <util/delay.h>

/////////////
// Defines //
/////////////
#define ADDR1	0x68
#define ADDR2	0x0C
#define NUM_READS	2000

//////////////////////
// Global Variables //
//////////////////////
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

/////////////////////////
// Function Prototypes //
/////////////////////////
void die();

//////////////////////////
// Function Definitions //
//////////////////////////
/**
 * Die!
 */
void die(){
	while(1){
	}
}

/**
 * Main function.  High level control script.
 *
 * @param  argc System argument list length
 * @param  argv System argument pointers
 * @return      System return status
 */
int main(int argc, char** argv){
	////////////
	// Setup //
	////////////
	uart_init();
	stderr = stdout = stdin = &uart_str;

	// Setup MPU9150
	if(MPU9150_init()){
		printf("MPU9150 initialization failed!\n");
		die();
	}
	if(MPU9150_Read()){
		printf("MPU9150 test read failed!\n");
		die();
	}

	///////////////////
	// Main sequence //
	///////////////////
	_delay_ms(1000);
	for(int i = 0; i < NUM_READS; i++){
		_delay_ms(10);
		if(twi_write(ADDR2, MPU9150_MAG_CNTL, 1)){
			printf("Start read failed!\n");
			die();
		}
		uint8_t status;
		do{
			if(twi_read_byte(ADDR2, MPU9150_MAG_ST1, &status)){
				printf("Check status bit failed!\n");
				die();
			}	
		}while(!(status & 1));
		// raw_mag[X]
		int raw_mag[3];
		i2c_start_wait(ADDR2 << 1);	// SLA+W
		i2c_write(MPU9150_MAG_HXL);	// write start reg
		i2c_rep_start((ADDR2 << 1) + 1);
		uint8_t byte_L = i2c_readAck();
		uint8_t byte_H = i2c_readAck();
		raw_mag[0] = byte_H << 8 | byte_L;

		// raw_mag[Y]
		byte_L = i2c_readAck();
		byte_H = i2c_readAck();
		raw_mag[1] = byte_H << 8 | byte_L;

		// raw_mag[Z]
		byte_L = i2c_readAck();
		byte_H = i2c_readNak();
		i2c_stop();
		raw_mag[2] = byte_H << 8 | byte_L;
		printf("%d,%d,%d\n", raw_mag[0], raw_mag[1], raw_mag[2]);	
	}
}
