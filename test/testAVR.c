#include <avr/io.h>

#define F_CPU 16000000UL

#include <util/delay.h>

int main(int argc, char** argv){
	//Initialize
	DDRB |= (1 << 5);	//configure PB5 as output

	//loop
	while(1){
		PORTB ^= (1 << 5);	// toggle PB5
		_delay_ms(500);	// delay 500 ms
	}
}
