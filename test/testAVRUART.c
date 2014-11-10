#define F_CPU 16000000UL

#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <uart.h>

int main(int argc, char** argv){
	uart_init();
	FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	stdout = stdin = &uart_str;
	int roll, pitch, yaw;
	roll = 0;
	pitch = 0;
	yaw = 0;
	while(1){
		_delay_ms(500);
		if(UCSR0A & (1 << RXC0)){
			scanf("%d %d %d", &roll, &pitch, &yaw);
			printf("%d\t%d\t%d\n", roll, pitch, yaw);
		}
	}
}
