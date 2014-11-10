// Includes
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <uart.h>

// Definitions
#define PITCH_SERVO	D5
#define PITCH_MIN	121
#define PITCH_RANGE	175
#define PITCH_MAX	255
#define YAW_SERVO	D3
#define YAW_MIN		141
#define YAW_ZERO	191
#define YAW_MAX		241
#define ROLL_SERVO	B1
#define ROLL_MIN	333
#define ROLL_RANGE	60
#define ROLL_MAX	425
#define YAW_KP		1
#define YAW_KI		0
#define YAW_KD		0

// Global Variables

int main(int argc, char** argv){
	// setup
	// Configure UART
	uart_init();
	FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	stdout = stdin = &uart_str;
	
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



	// Configure servo initial locations
	OCR2B = (PITCH_MIN + PITCH_MAX) / 2;	// set mid pitch
	OCR2A = YAW_ZERO;
	OCR1A = (ROLL_MIN + ROLL_MAX) / 2;	// set mid roll

	// configure sensor
	int curPitch = 0;
	int curYaw = 0;
	int curRoll = 0;

	// configure controller
	int rollGoal = 0;
	int pitchGoal = 0;
	int yawGoal = 0;
	int error = 0;
	int curSpeed = 0;
	int accError = 0;
	int lastPos = 0;
	int setPoint = 0;
	int yawOutput = 0;
	int pitchOutput = 0;
	int rollOutput = 0;

	while(1){
		// Get pos
		// Set setpoint
		if(UCSR0A & (1 << RXC0)){
			scanf("%d %d %d", &rollGoal, &pitchGoal, &yawGoal);
			printf("%c\n", (char) 0x06);
		}
		// Calculate output
		error = curYaw - setPoint;
		accError += error;
		curSpeed = curYaw - lastPos;
		lastPos = curYaw;
		yawOutput = YAW_KP * error + YAW_KI * accError + YAW_KD * curSpeed;
		// Implement output
	}
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
