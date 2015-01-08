/**
 * Arduino Controller
 *
 * @author NATHAN HUI
 */

//////////////
// Includes //
//////////////
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <uart.h>
#include <MPU9150.h>

/////////////////
// Definitions //
/////////////////
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
#define MAJVER		0
#define MINVER		2
#define DEBUG 		1
#define DEBUG_PRINT(fmt, ...) do{ if(DEBUG) fprintf(stderr, "%s:%d: " fmt, __FILE__, __LINE__, ##__VA_ARGS__);}while(0)

//////////////////////
// Global Variables //
//////////////////////
int enableStabilization = 0;
int rollGoal = 0;
int pitchGoal = 0;
int yawGoal = 0;
volatile int error = 0;
volatile int curSpeed = 0;
volatile int accError = 0;
volatile int lastPos = 0;
volatile int yawOutput = 0;
volatile int stupidYawCount = 0;
int mode = 1;
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

/////////////////////////
// Function Prototypes //
/////////////////////////
void setRoll(int angle);
void setPitch(int angle);
void setYaw(int angle);
void bumpRoll(int angle);
void bumpPitch(int angle);
void bumpYaw(int angle);
void setup();

//////////////////////////
// Function Definitions //
//////////////////////////
/**
 * Setup function.  This function configures the Atmega328p hardware
 */
void setup(){
	// setup
	// Configure UART
	uart_init();
	stderr = stdout = stdin = &uart_str;

	// // Setup pins as outputs
	// DDRD |= (1 << 5) | (1 << 3);
	// DDRB |= (1 << 1) | (1 << 5);

	// cli();	// disable interrupts

	// // Configure Timer2
	// TCCR2A = 0;
	// TCCR2B = 5;	// Select 256 prescaler @ 31 kHz overflow
	// TIMSK2 = 1 << 0 | 1 << 1 | 1 << 2;	// enable overflow interrupt and output compare

	// // Configure Timer1
	// TCCR1A = 0;
	// TCCR1B = 3;	// Select no prescaler
	// TIMSK1 |= 1 << 0 | 1 << 1 | 1 << 2;	// enable overflow interrupt
	// OCR1B = 600;	// Set frequency

	// // Configure Timer0
	// TCCR0A |= (1 << WGM01);	// set CTC mode on Timer0
	// TCCR0B |= (1 << CS00) | (1 << CS01);	// enable 0 prescaler
	// OCR0A = 249;	// set controller frequency here!
	// TIMSK0 |= (1 << OCIE0A);	// enable overflow interrupt on A

	// sei(); // reenable interrupts

	// // Configure servo initial locations
	// OCR2B = (PITCH_MIN + PITCH_MAX) / 2;	// set mid pitch
	// OCR2A = YAW_ZERO;
	// OCR1A = (ROLL_MIN + ROLL_MAX) / 2;	// set mid roll
	// MPU9150_init();
}

int main(int argc, char** argv){
	setup();
	printf("Arduino Gimbal Controller v%d.%d, compiled %s at %s\n", MAJVER, MINVER, __DATE__, __TIME__);

	// configure sensor

	// Initialize DCM matrix

	// configure controller

	while(1){
		// Get pos

		// Set setpoint and process commands

		// Calculate output

		// Implement output

	}
}

void setRoll(int angle){
	rollGoal = angle;
	OCR1A = (int)((float)angle / ROLL_RANGE * (ROLL_MAX - ROLL_MIN) / 2 + (ROLL_MIN + ROLL_MAX) / 2);
}

void setPitch(int angle){
	pitchGoal = angle;
	OCR2B = (int)((float)angle / PITCH_RANGE * (PITCH_MAX - PITCH_MIN) / 2 + (PITCH_MIN + PITCH_MAX) / 2);
}

void bumpRoll(int angle){
	setRoll(rollGoal + angle);
}

void bumpPitch(int angle){
	setPitch(pitchGoal + angle);
}

void bumpYaw(int angle){
	yawGoal = angle;
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

ISR(TIMER0_COMPA_vect){
	PORTB ^= (1 << PORTB5);
	// Set roll
	// nothing

	// set pitch
	// nothing

	// set yaw
	switch(mode){
		case 1:
			// stupid mode
			if(stupidYawCount > 0){
				switch(yawGoal){
					case 1:
						OCR2A = 193;
						break;
					default:
					case 0:
						OCR2A = YAW_ZERO;
						break;
					case -1:
						OCR2A = 189;
						break;
				}
				stupidYawCount--;
			}else{
				OCR2A = YAW_ZERO;
			}
//		case 0:
//			error = curYaw - yawGoal;
//			accError += error;
//			curSpeed = curYaw - lastPos;
//			lastPos = curYaw;
//			yawOutput = YAW_KP * error + YAW_KI * accError + YAW_KD * curSpeed;

	}
}
