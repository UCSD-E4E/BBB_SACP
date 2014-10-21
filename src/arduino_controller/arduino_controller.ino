// Includes
//#include <Servo.h>

// Definitions
#define PITCH_SERVO	D5
#define PITCH_MIN	164
#define PITCH_RANGE
#define PITCH_MAX	212
#define YAW_SERVO	D3
#define YAW_MIN		141
#define YAW_ZERO	191
#define YAW_MAX		241
#define ROLL_SERVO	B1
#define ROLL_MIN	245
#define ROLL_RANGE
#define ROLL_MAX	505

// Global Variables

void setup(){
	// Setup pins as outputs
	DDRD |= (1 << 5) | (1 << 3);
	DDRB |= (1 << 1);

	cli();	// disable interrupts
	
	// Configure Timer0
	TCCR2A = 0;
	TCCR2B = 5;	// Select 256 prescaler @ 31 kHz overflow
	TIMSK2 = 1 << 0 | 1 << 1 | 1 << 2;	// enable overflow interrupt and output compare

	// Configure Timer1
	TCCR1A = 0;
	TCCR1B = 3;	// Select no prescaler
	TIMSK1 |= 1 << 0 | 1 << 1 | 1 << 2;	// enable overflow interrupt
	OCR1B = 600;	// Set frequency

	sei(); // reenable interrupts
}

void loop(){
	OCR2B = 188;	// set mid pitch
	OCR2A = YAW_ZERO;
	OCR1A = 375;	// set mid roll
}

ISR(TIMER2_COMPA_vect){
	// Yaw falling edge
	PORTD &= ~(1 << 3);
}
ISR(TIMER2_COMPB_vect){
	// Pitch falling edge
	PORTD &= ~(1 << 5);
}
ISR(TIMER1_COMPA_vect){
	// Roll falling edge
	PORTB &= ~(1 << 1);
}

ISR(TIMER2_OVF_vect){
	// Rising edge for pitch and yaw
	PORTD |= 1 << 5 | 1 << 3;
}

ISR(TIMER1_COMPB_vect){
	// Rising edge for roll
	PORTB |= 1 << 1;
	TCNT1 = 0;
}
