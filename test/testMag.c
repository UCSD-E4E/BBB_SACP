/**
 * Test suite for magnetometer
 * Author: Nathan Hui
 * Date: 12/28/14
 */

// Include Files
#include <MPU9150.h>
#include <uart.h>
#include <i2cwrap.h>
#include <i2cmaster.h>
#include <stdio.h>
#include <MPU9150_reg.h>
#include <math.h>


#define ADDR1	0x68
#define ADDR2	0x0C


// Global Variables
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
const double MAG_MAT[3][3] = {{0.0071, 	0.0002, 	0.0003},
							  {0, 		0.0070, 	-0.003},
							  {0, 		0, 			0.0072}};
const double MAG_VEC[3] = {-20.9245, 36.9164, 81.9928};

// Function Prototypes
void die(void);


int main(int argc, char** argv){
	uart_init();
	stderr = stdout = stdin = &uart_str;
	i2c_init();

	printf("Activating MPU9150...\n");
	printf("Sending start and address...");
	i2c_start_wait(ADDR1 << 1);
	// if(i2c_start(ADDR1 << 1)){
	// 	printf("Failed!\n");
	// 	die();
	// }
	printf("done\n");
	printf("Writing destination...");
	if(i2c_write(MPU9150_PWR_MGMT_1)){
		printf("Failed!\n");
		die();
	}
	printf("done\n");
	printf("Writing value...");
	if(i2c_write(1 << 0)){
		printf("Failed!\n");
		die();
	}
	printf("done\n");
	printf("Sending stop...");
	i2c_stop();
	printf("done\n");
	// if(twi_write(ADDR1, MPU9150_PWR_MGMT_1, 1 << 0)){
	// 	printf("Failed!\n");
	// 	die();
	// }
	printf("done\n");

	printf("Pinging MPU9150...");
	uint8_t id;
	if(twi_read_byte(ADDR1, MPU9150_WHO_AM_I, &id)){
		printf("Failed!\n");
		die();
	}
	printf("done\n");
	printf("ID: 0x%2X, should be 0x68\n", id);

	printf("Enabling mag passthrough...");
	if(twi_write(ADDR1, MPU9150_I2C_INT_PIN_CFG, 1 << 1)){
		printf("Failed!\n");
		die();
	}
	printf("done\n");

	printf("Activiating mag...");
	if(twi_write(ADDR2, MPU9150_MAG_CNTL, 1 << 0)){
		printf("Failed!\n");
		die();
	}
	printf("done\n");

	printf("Pinging mag...");
	if(twi_read_byte(ADDR2, MPU9150_MAG_WIA, &id)){
		printf("Failed!\n");
		die();
	}
	printf("done\n");
	printf("Magnetometer ID: 0x%2X, should be 0x48\n", id);

	printf("Testing read...");
	if(twi_write(ADDR2, MPU9150_MAG_CNTL, 1)){
		printf("Failed!\n");
		die();
	}
	printf("done\n");

	printf("Waiting for read completion...");
	uint8_t status;
	if(twi_read_byte(ADDR2, MPU9150_MAG_ST1, &status)){
		printf("Failed!\n");
		die();
	}
	while(!(status & 1)){
		if(twi_read_byte(ADDR2, MPU9150_MAG_ST1, &status)){
			printf("Failed!\n");
			die();
		}	
	}
	printf("done\n");
	printf("Status: 0x%d\n", status);

	printf("Getting sample data...");
	// Activate measurement
	twi_write(ADDR2, MPU9150_MAG_CNTL, 1);
	int raw_mag[3];
	uint8_t status1;
	if(twi_read_byte(ADDR2, MPU9150_MAG_ST1, &status1)){
		printf("Failed!\n");
		die();
	}
	while(!(status1 & 1)){
		if(twi_read_byte(ADDR2, MPU9150_MAG_ST1, &status1)){
			printf("Failed!\n");
			die();
		}	
	}
	// raw_mag[X]
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
	printf("done\n");

	// Apply calibration
	double calibData1[3] = {0};
	double calibData2[3] = {0};
	for(int i = 0; i < 3; i++){
		calibData1[i] = raw_mag[i] - MAG_VEC[i];
	}
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			calibData2[i] += calibData1[j] * MAG_MAT[i][j];
		}
	}

	printf("Test result: %f\t%f\t%f\n", calibData2[0], calibData2[1], calibData2[2]);

	printf("Continuing read tests:");
	int counter = 0;
	double mag;
	while(1){
		if(twi_write(ADDR2, MPU9150_MAG_CNTL, 1)){
			printf("Failed!\n");
			break;
		}

		uint8_t status;
		if(twi_read_byte(ADDR2, MPU9150_MAG_ST1, &status)){
			printf("Failed!\n");
			break;
		}
		while(!(status & 1)){
			if(twi_read_byte(ADDR2, MPU9150_MAG_ST1, &status)){
				printf("Failed!\n");
				goto broken;
			}	
		}

		// raw_mag[X]
		i2c_start_wait(ADDR2 << 1);	// SLA+W
		i2c_write(MPU9150_MAG_HXL);	// write start reg
		i2c_rep_start((ADDR2 << 1) + 1);
		byte_L = i2c_readAck();
		byte_H = i2c_readAck();
		raw_mag[0] = (int)(byte_H << 8 | byte_L);

		// raw_mag[Y]
		byte_L = i2c_readAck();
		byte_H = i2c_readAck();
		raw_mag[1] = (int)(byte_H << 8 | byte_L);

		// raw_mag[Z]
		byte_L = i2c_readAck();
		byte_H = i2c_readNak();
		i2c_stop();
		raw_mag[2] = (int)(byte_H << 8 | byte_L);

		for(int i = 0; i < 3; i++){
			calibData1[i] = raw_mag[i] - MAG_VEC[i];
		}
		mag = 0;
		for(int i = 0; i < 3; i++){
			calibData2[i] = 0;
			for(int j = 0; j < 3; j++){
				calibData2[i] += calibData1[j] * MAG_MAT[i][j];
			}
			mag += calibData2[i] * calibData2[i];
		}

		printf("Test result: %f\t%f\t%f\t%f\n", calibData2[1], calibData2[0], calibData2[2], sqrt(mag));
		counter++;
	}
	broken:
	printf("Failed at %d iterations!", counter);
	return 0;
}

void die(void){
	while(1){
	}
}