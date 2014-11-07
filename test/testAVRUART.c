#define F_CPU 16000000UL

#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <uart.h>

int main(int argc, char** argv){
	uart_init();
	FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	stdout = stdin = &uart_str;
	while(1){
		printf("Hello World!\n");
		_delay_ms(500);
	}
}
