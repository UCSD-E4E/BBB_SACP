#define F_CPU 16000000UL	// 16 MHz

#include <MPU9150.h>
#include <avr/io.h>
#include <uart.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <util/delay.h>

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

float min[3];
float max[3];
float mean[3];

void loop(){
//	PORTB ^= (1 << PORTB5);
	PORTB |= (1 << PORTB5);
	update_DCM(0.03660);
//	MPU9150_Read();
	PORTB &= ~(1 << PORTB5);
//	printf("%3.1f\t%3.1f\t%3.1f\t%3.1f\t%3.1f\t%3.1f\t%3.1f\t%3.1f\t%3.1f\n", 
//			(accelX + beta[0]) / (float)beta[3],
//			(accelY + beta[1]) / (float)beta[4],
//			(accelZ + beta[2]) / (float)beta[5],
//			(gyroX + beta[6]) / (float)beta[9],
//			(gyroY + beta[7]) / (float)beta[10],
//			(gyroZ + beta[8]) / (float)beta[11],
//			(magZ),
//			(magY),
//			(magZ)); 
	printf("%3.3f\t%3.3f\t%3.3f\n", getRoll(), getPitch(), getYaw());
//	printf("%3.6f\t%3.6f\t%3.6f\n%3.6f\t%3.6f\t%3.6f\n%3.6f\t%3.6f\t%3.6f\n\n", DCMG[0][0], DCMG[0][1], DCMG[0][2], DCMG[1][0], DCMG[1][1], DCMG[1][2], DCMG[2][0], DCMG[2][1], DCMG[2][2]);
//	if(min[0] > (gyroX + beta[6])){
//		min[0] = gyroX + beta[6];
//	}
//	if(min[1] > gyroY + beta[7]){
//		min[1] = gyroY + beta[7];
//	}
//	if(min[2] > gyroZ + beta[8])
//		min[2] = gyroZ + beta[8];
//	if(max[0] < gyroX+beta[6])
//		max[0] = gyroX+beta[6];
//	if(max[1] < gyroY+beta[7])
//		max[1] = gyroY+beta[7];
//	if(max[2] < gyroZ+beta[8])
//		max[2] = gyroZ+beta[8];
//	printf("%3d\t%3d\t\t%3d\t%3d\t\t%3d\t%3d\n", min[0], max[0], min[1], max[1], min[2], max[2]);
//	mean[0] += (gyroX + beta[6]) / (float)(beta[9]);
//	mean[0] /= 2;
//	mean[1] += (gyroY + beta[7]) / (float)(beta[9]);
//	mean[1] /= 2;
//	mean[2] += (gyroZ + beta[8]) / (float)(beta[9]);
//	mean[2] /= 2;
//	if(min[0] > mean[0])
//		min[0] = mean[0];
//	if(min[1] > mean[1])
//		min[1] = mean[1];
//	if(min[2] > mean[2])
//		min[2] = mean[2];
//	if(max[0] < mean[0])
//		max[0] = mean[0];
//	if(max[1] < mean[1])
//		max[1] = mean[1];
//	if(max[2] < mean[2])
//		max[2] = mean[2];
//	printf("%3.6f\t%3.6f\t%3.6f\n", mean[0], mean[1], mean[2]);
//	printf("%3.2f\t%3.2f\t\t%3.2f\t%3.2f\t\t%3.2f\t%3.2f\n", min[0], max[0], min[1], max[1], min[2], max[2]);
	_delay_ms(30);

}

int main(int argc, char** arvg){
	uart_init();
	stderr = stdout = stdin = &uart_str;
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


