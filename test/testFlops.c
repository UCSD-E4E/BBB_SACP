#define F_CPU 16000000UL
#include <avr/io.h>

int main(int argc, char** argv){
	// initialize
	DDRB |= (1 << 5);	// configure PB5 as output
	long i = 4;
	//loop
	while(1){
		PORTB ^= (1 << 5);	// one edge (rising or falling), is 1 fp op
		i += 103;
		i -= 53;
		i *= 113;
		i /= 29;
	}
}
