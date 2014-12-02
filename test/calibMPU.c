#include <stdio.h>
#include <uart.h>
#include <MPU9150.h>


FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

int main(int argc, char** argv){
	uart_init();
	stdout = stdin = &uart_str;
	MPU9150_init();
	calibrateMPU9150();
	while(1){
	}
}
