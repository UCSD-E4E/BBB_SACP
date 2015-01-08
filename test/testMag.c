/**
 * Test suite for magnetometer
 * Author: Nathan Hui
 * Date: 12/28/14
 */

// Include Files
#include <MPU9150.h>
#include <uart.h>
#include <i2cwrap.h>
#include <stdio.h>

// Global Variables
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

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

	printf("Continuing read tests:");
	int counter = 0;
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