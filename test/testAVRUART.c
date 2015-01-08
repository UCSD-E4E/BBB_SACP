#define F_CPU 16000000UL

#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <uart.h>

int main(int argc, char** argv){
	uart_init();
	FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	stdout = stdin = &uart_str;
	printf("Arduino Gimbal Controller v%d.%d, compiled %s at %s\n", 0, 9, __DATE__, __TIME__);
	double test = 3.1415;
	printf("Float: %f\n", test);
	//Initialize
	DDRB |= (1 << 5);	//configure PB5 as output

	//loop
	while(1){
		PORTB ^= (1 << 5);	// toggle PB5
		_delay_ms(500);	// delay 500 ms
	}
}
