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
#include <util/delay.h>
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
#define MAJVER		0
#define MINVER		3
#define DEBUG 		1
#define DEBUG_PRINT(fmt, ...) do{ if(DEBUG) fprintf(stderr, "%s:%d: " fmt, \
		__FILE__, __LINE__, ##__VA_ARGS__);}while(0)

//////////////////////
// Global Variables //
//////////////////////
volatile uint64_t time_ms = 0;
uint64_t ctrl_update = 0;
uint64_t uart_update = 0;
int enableStabilization = 0;
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
void setRoll(double angle);
void setPitch(double angle);
void bumpRoll(int angle);
void bumpPitch(int angle);
void bumpYaw(int angle);
void setup();
int get_DCM_UART(double DCM[3][3]);
void invert_matrix(double ret[][3], double mat[][3]);
void _compose_rotations(double ret[][3], double rot1[][3], double rot2[][3]);
double DCM_to_Roll(double DCM[3][3]);
double DCM_to_Pitch(double DCM[3][3]);
double DCM_to_Yaw(double DCM[3][3]);

//////////////////////////
// Function Definitions //
//////////////////////////
/**
 * Calculates and returns the yaw component of the given rotation matrix,
 * using a y-x'-z" rotation scheme.
 *
 * @return	calculated yaw component of the given rotation matrix
 */
double DCM_to_Yaw(double DCM[3][3]){
	return acos(DCM[1][1] / cos(-asin(DCM[1][2])));
}

/**
 * Calculates and returns the pitch component of the given rotation matrix,
 * using a y-x'-z" rotation scheme.
 *
 * @return	calculated pitch component of the given rotation matrix
 */
double DCM_to_Pitch(double DCM[3][3]){
	double x = -asin(DCM[1][2]);
	return asin(DCM[0][2] / cos(x));
}

/**
 * Calculates and returns the roll component of the given rotation matrix,
 * using a y-x'-z" rotation scheme.
 *
 * @return	calculated pitch component of the given rotation matrix
 */
double DCM_to_Roll(double DCM[3][3]){
	return -asin(DCM[1][2]);
}

/**
 * Composes the rotation rot2 onto rot1, i.e. apply rot1, then apply rot2.
 * ret must be initialized to zero prior to calling this function.
 *
 * @param ret	Pointer to the array to populate with the new rotation matrix
 * @param ret1	Pointer to the array representing matrix 1 in row col notation
 * @param ret2	Pointer to the array representing matrix 2 in row col notation
 */
void _compose_rotations(double ret[][3], double rot1[][3], double rot2[][3]){
	for(int row = 0; row < 3; row++){
		for(int col = 0; col < 3; col++){
			for(int i = 0; i < 3; i++){
				ret[row][col] += rot1[row][i] * rot2[i][col];
			}
		}
	}
}

/**
 * Inverts the given matrix and stores it in ret[][3].
 *
 * @param ret	Pointer to the array to populate with the new rotation matrix
 * @param mat	Pointer to the array containing the given matrix
 */
void invert_matrix(double ret[][3], double mat[][3]){
	double det = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) -
				 mat[1][0] * (mat[0][1] * mat[2][2] - mat[2][1] * mat[0][2]) +
				 mat[2][0] * (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]);
	ret[0][0] = (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) / det;
	ret[0][1] = (mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2]) / det;
	ret[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) / det;
	ret[1][0] = (mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2]) / det;
	ret[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) / det;
	ret[1][2] = (mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2]) / det;
	ret[2][0] = (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]) / det;
	ret[2][1] = (mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1]) / det;
	ret[2][2] = (mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1]) / det;
}

/**
 * Gets the DCM uart.  Blocks if there is a message waiting, else immediately
 * returns.
 *
 * @param DCM	DCM to fill
 * @return	Returns 0 if received successful DCM.  Returns 1 if no message in
 *                 queue.  Returns 2 if bad message.
 */
int get_DCM_UART(double DCM[3][3]){
	// First, check if there is something.  If not, return 0.
	if(!(UCSR0A & (1 << RXC0))){
		return 1;
	}

	// Must be a message.  Get the message!
	char buffer[32] = {0};
	int bufferSpot = 0;

	// for each row of the DCM
	for(int line = 0; line < 3; line++){
		bufferSpot = 0;
		while(UCSR0A & (1 << RXC0)){	// Get the line of the message
			buffer[bufferSpot] = UDR0;
			if(buffer[bufferSpot] == '\n' || buffer[bufferSpot] == '\r'){
				break;
			}
			bufferSpot++;	// next cell in buffer
		}
		if(sscanf(buffer, "%lf%lf%lf", &DCM[line][0], &DCM[line][1], 
				&DCM[line][2]) != 3){
			return 2;
		}
	}
	return 0;
}

/**
 * Setup function.  This function configures the Atmega328p hardware.
 *
 * Initializes the following:
 * 	UART at 57600 baud with 8N1 frame
 * 	Arduino Pin 3, 5, 6, and 13 as high Z (output)
 * 	Timer 0, 1, 2 for PWM generation and 1 ms timer
 * 	MPU9150 to a ready state
 * 	Servos to mid-range
 * 	
 */
void setup(){
	// setup
	// Configure UART
	uart_init();
	stderr = stdout = stdin = &uart_str;

	// Setup pins as outputs
	DDRD |= (1 << 5) | (1 << 3) | (1 << 6);	// Pitch (Pin 5) and Yaw (Pin 3)
			// and Roll (Pin 6)
	DDRB |= (1 << 5);	// LED (Pin 13)

	cli();	// disable interrupts

	// Configure Timer2
	TCCR2A = 0;
	TCCR2B = 5;	// Select 256 prescaler @ 31 kHz overflow
	TIMSK2 = 1 << 0 | 1 << 1 | 1 << 2;	// overflow interrupt & output compare

	// Configure Timer1
	TCCR1A = 0;
	TCCR1B = 3;	// Select no prescaler
	TIMSK1 |= 1 << 0 | 1 << 1 | 1 << 2;	// enable overflow interrupt
	OCR1B = 600;	// Set frequency

	// Configure Timer0
	TCCR0A |= (1 << WGM01);	// set CTC mode on Timer0
	TCCR0B |= (1 << CS00) | (1 << CS01);	// enable 0 prescaler
	OCR0A = 249;	// set 1 ms frequency here!  This is for the ms counter
	TIMSK0 |= (1 << OCIE0A);	// enable overflow interrupt on A

	sei(); // reenable interrupts

	// Configure servo initial locations
	OCR2B = (PITCH_MIN + PITCH_MAX) / 2;	// set mid pitch
	OCR2A = YAW_ZERO;
	OCR1A = (ROLL_MIN + ROLL_MAX) / 2;	// set mid roll

	// Initialize MPU9150
	MPU9150_init();

}

int main(int argc, char** argv){
	setup();
	printf("Arduino Gimbal Controller v%d.%d, compiled %s at %s\n", MAJVER, 
			MINVER, __DATE__, __TIME__);

	// Initialize DCM matrices
	double DCM_goal[3][3];
	double DCM_base[3][3];
	double DCM_binv[3][3];
	double DCM_gimb[3][3];
	double rollGoal = 0;
	double pitchGoal = 0;
	double yawGoal = 0;

	double y_p = 1;
	double y_d = 0;
	double y_i = 0;
	double y_int = 0;
	double y_spd = 0;
	double y_prev = 0;

	// configure controller
	while(1){
		if(time_ms >= uart_update){
			uart_update = time_ms + 100;
			// get UART DCM
			int result = get_DCM_UART(DCM_goal);
			if(result == 0){

				// Invert MPU9150 DCM
				invert_matrix(DCM_binv, DCM_base);

				// Calculate the gimbal rotation
				_compose_rotations(DCM_gimb, DCM_binv, DCM_goal);

				// Extract RP, store in goal
				rollGoal = DCM_to_Roll(DCM_gimb);
				pitchGoal = DCM_to_Pitch(DCM_gimb);

				// Extract Y from UART DCM, store in goal
				yawGoal = DCM_to_Yaw(DCM_base);
			}
		}

		if(time_ms >= ctrl_update){
			// update time
			ctrl_update = time_ms + 10;

			// Update DCM
			PORTB ^= (1 << 5);
			update_DCM(0.01);
			// get MPU9150 DCM
			get_DCM(DCM_base);

			// setRoll
			OCR1A = (int)(rollGoal *180 / M_PI / ROLL_RANGE * (ROLL_MAX - 
					ROLL_MIN) / 2 + (ROLL_MIN + ROLL_MAX) / 2);

			// setPitch
			OCR2B = (int)(pitchGoal * 180 / M_PI / PITCH_RANGE * (PITCH_MAX - 
					PITCH_MIN) / 2 + (PITCH_MIN + PITCH_MAX) / 2);

			// YAW PID calc
			double curY = DCM_to_Yaw(DCM_base);
			double err = curY - yawGoal;
			y_int += err;
			y_spd = curY - y_prev;
			y_prev = curY;
			double y_out = y_p * err + y_i * y_int + y_d * y_spd + YAW_ZERO;
			if(y_out > YAW_MAX){
				y_out = YAW_MAX;
			}
			if(y_out < YAW_MIN){
				y_out = YAW_MIN;
			}
			OCR2A = (int) y_out;
		}
		
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

ISR(TIMER0_COMPA_vect){
	// 1 ms counter
	time_ms++;
}
