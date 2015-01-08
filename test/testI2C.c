/**
 * Test suite for I2C interface on the Atmega 328p. This suite simply sends a
 * set of characters on the I2C line.
 */
#include <stdio.h>
#include <uart.h>
#include <i2cmaster.h>

int main(int argc, char** argv){
	uart_init();
	FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	stdout = stdin = &uart_str;

	i2c_init();
	i2c_start_wait(0x68 << 1 + I2C_WRITE);
	printf("Beginning...");
	i2c_write((uint8_t)'k');
	i2c_write((uint8_t)'n');
	i2c_stop();
	while(1){
		printf("done\n");
	}
}
