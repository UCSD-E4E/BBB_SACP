#include <MPU9150.h>
#define F_CPU 16000000UL
#include <avr/io.h>
#include <uart.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

int main(int argc, char** argv){
	uart_init();
	stdout = stdin = &uart_str;
	float quat1[4] = {.707107, .707107, 0, 0};
	float quat2[4] = {.707107, 0, .707107, 0};
	float quat3[4] = {0};
	_mulQuat(quat1, quat2, quat3);

	for(int i = 0; i < 4; i++){
		printf("%3.3f\t", quat3[i]);
	}
	printf("\n");
	while(1){
		continue;
	}
}
