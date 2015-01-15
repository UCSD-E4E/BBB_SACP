/**
 * Test suite for MPU9150
 * Author: Nathan Hui
 * Date: 1/12/15
 */

///////////////////
// Include Files //
///////////////////
#include <MPU9150.h>
#include <uart.h>
#include <avr/io.h>
#include <i2cwrap.h>
#include <i2cmaster.h>
#include <stdio.h>
#include <stdint.h>
#include <MPU9150_reg.h>

/////////////
// Defines //
/////////////

//////////////////////
// Global Variables //
//////////////////////
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

/////////////////////////
// Function Prototypes //
/////////////////////////
void die(void);

//////////////////////////
// Function Definitions //
//////////////////////////
void die(void){
	while(1){

	}
}
int main(int argc, char** arvg){
	uart_init();
	stderr = stdout = stdin = &uart_str;
	
	printf("Running MPU9150 setup...");
	if(MPU9150_init()){
		printf("failed!\n");
		die();
	}
	printf("done\n");

	printf("Testing MPU9150 read...");
	if(MPU9150_Read()){
		printf("failed!\n");
		die();
	}
	printf("done\n");

	printf("Results of test read:\n");
	double DCM[3][3] = {{0}};
	get_DCM(DCM);

	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			printf("%3.3f\t", DCM[i][j]);
		}
		printf("\n");
	}
	printf("Please sanity check above DCM!\n");

	printf("Testing continuous reads...");
	while(1){
		if(MPU9150_Read()){
			printf("failed!\n");
			die();
		}
	}


}


