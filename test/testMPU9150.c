#define F_CPU 16000000UL	// 16 MHz

#include <MPU9150.h>
#include <avr/io.h>
#include <uart.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <util/delay.h>

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void loop(){
//	PORTB ^= (1 << PORTB5);
	PORTB |= (1 << PORTB5);
	update_DCM(0.020);
//	MPU9150_Read();
	PORTB &= ~(1 << PORTB5);
//	printf("%6i\t%6i\t%6i\t%6i\t%6i\t%6i\t%6i\t%6i\t%6i\n", accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magZ, magY, magZ); 
//	printf("%3.6f\t%3.6f\t%3.6f\n", getRoll(), getPitch(), getYaw());
	_delay_ms(15);

}

int main(int argc, char** arvg){
	uart_init();
	stdout = stdin = &uart_str;
	printf("Beginning...\n");
	MPU9150_init();
	DDRB |= 1 << DDB5;
	int counter = 0;
	while(counter < 32){
		loop();
//		counter++;
	}
	while(1){
		continue;
	}
}


