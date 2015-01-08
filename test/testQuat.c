#include <MPU9150.h>
#define F_CPU 16000000UL
#include <avr/io.h>
#include <uart.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

int main(int argc, char** argv){
	uart_init();
	stdout = stdin = &uart_str;

	float newAcc[3] = {0, 0, 1.001};
	float _prevAcc[3] = {0, 0, 1};
	
	float halfAngAcc = acos(fmin(fmax(_dot(newAcc, _prevAcc), -1.0), 1.0)) / 2;

	printf("%3.3f\t", cos(halfAngAcc));
	printf("%3.3f\t", (newAcc[1] * _prevAcc[2] - newAcc[2] * _prevAcc[1]) * sin(halfAngAcc));
	printf("%3.3f\t", (newAcc[2] * _prevAcc[0] - newAcc[0] * _prevAcc[2]) * sin(halfAngAcc));
	printf("%3.3f\n", (newAcc[0] * _prevAcc[1] - newAcc[1] * _prevAcc[0]) * sin(halfAngAcc));

	while(1){
		continue;
	}
}
