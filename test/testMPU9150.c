#define F_CPU 16000000UL	// 16 MHz

#include <MPU9150.h>
#include <uart.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <util/delay.h>

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void loop(){
	MPU9150_Read();
	printf("%6i\t%6i\t%6i\t%6i\t%6i\t%6i\t%6i\t%6i\t%6i\n", accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magZ, magY, magZ); 
	_delay_ms(5);

}

int main(int argc, char** arvg){
	uart_init();
	stdout = stdin = &uart_str;
	printf("Beginning...\n");
	MPU9150_init();
	while(1){
		loop();
	}
}


