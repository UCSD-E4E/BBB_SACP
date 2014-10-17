/**
 * BBB_SACP Control Process
 */

// Includes
#include <zmq.hpp>
#include <quaternion.hpp>
#include <MPU9150.hpp>
#include "dataStructure.hpp"
#include <PWM.hpp>
#include <Servo.hpp>
#include <CRServo.hpp>
#include <sstream>
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <cstdio>
#include <string.h>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
using namespace std;

// Defines
#define YAW_P 0
#define YAW_I 0
#define YAW_D 0

// Global Variables
Quaternion <float> setPoint;
Quaternion <float> imuPoint;
bool _stabilization = true;
float yawSetPoint = 0;
struct termios options;

// Function definitions
float swap_bytes(unsigned char* buffer){
	unsigned char reversed[4];
	reversed[0] = buffer[3];
	reversed[1] = buffer[2];
	reversed[2] = buffer[1];
	reversed[3] = buffer[0];

	float tmp;
	memcpy(&tmp, &reversed[0], sizeof(float));
	return tmp;
}

int main(int argc, char** argv){
	// Set up communications
	// Initialize single ZMQ context with single I/O thread
	cout << "Initializing..." << flush;
	zmq::context_t control_Context (1);
	zmq::socket_t control_Socket (control_Context, ZMQ_REP);
	control_Socket.bind("tcp://127.0.0.1:55000");

	zmq::context_t data_Context (1);
	zmq::socket_t data_Socket (data_Context, ZMQ_PUB);
	data_Socket.bind("tcp://127.0.0.1:55002");

	int sensorFile = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	if(sensorFile == -1){
		cerr << "could not access uStrain!" << endl;
		abort();
	}else{
		tcgetattr(sensorFile, &options);
		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);
		options.c_cflag |= (CLOCAL | CREAD);
		tcsetattr(sensorFile, TCSANOW, & options);
	}
	uint8_t uStrainBuffer[128];
	int uStrain_byte_cnt;
	uint8_t uStrainOut[128];
	cout << "done" << endl;

	// Set up stabilization
	cout << "Initializing Stabilization..." << flush;
	setPoint = Quaternion <float> (1, 0, 0, 0);
	imuPoint = Quaternion <float> (1, 0, 0, 0);
	float sensorUpdate[4];

	cerr << "Fail here" << endl;
	Servo pitchServo (PWM_P8_13, 50, 740000, 2300000, 5888888);
	Servo rollServo(PWM_P8_34, 50, 740000, 2300000, 5888888);
	CRServo yawServo(PWM_P8_45, 50, 670000, 1534000, 2400000);
	cout << "done." << endl;

	float yawSetPt = 0.0;
	float prevYaw = 0.0;
	float curYawVel = 0.0;
	float curYawErr = 0.0;

	
	// Begin doing stuff
	cout << "Beginning control loops:" << endl;
	bool runState = true;
	struct timeval tp;
	uint64_t prevRun = 0;
	while(runState){
		gettimeofday(&tp, NULL);
		if(tp.tv_usec / 1000 + tp.tv_sec > prevRun + 50000){
			prevRun += 50000;
			zmq::message_t command;
			if(control_Socket.recv(&command, ZMQ_NOBLOCK)){
				// process command
				cout << "Have command!\nProcessing command..." << flush;
				string cmdValue((char*)command.data(), command.size());
				istringstream iss(cmdValue);
				string cmd;
				iss >> cmd;
				if(!cmd.compare("SETPOINT")){
					float setQuat[4];
					iss >> setQuat[0] >> setQuat[1] >> setQuat[2] >> setQuat[3];
					setPoint = Quaternion<float>(setQuat);
					cout << "Have setpoint!" << endl;
				}else if(!cmd.compare("STABILIZATION")){
					iss >> _stabilization;
					cout << "Have stabilize command!" << endl;
				}else if(!cmd.compare("exit")){
					runState = false;
					cout << "Exiting now!" << endl;
				}else{
					cout << "Bad Command" << endl;
					runState = false;
				}
				// Received command, reply
				cmd = "done";
				command.rebuild(4);
				memcpy((void*)command.data(), cmd.c_str(), cmd.size());
				if(!control_Socket.send(command)){
					abort();
				}
			}else{
			}
			// No command, continue

			// Check for sensor update
			uStrain_byte_cnt = read(sensorFile, &uStrainBuffer, 1);
			if(uStrain_byte_cnt > 0 && uStrainBuffer[0] == 0x65){
				uStrain_byte_cnt = read(sensorFile, &uStrainBuffer, 4);
				uint8_t sensorFileriptor_set = uStrainBuffer[0];
				uint8_t payload_length = uStrainBuffer[1];
				uint8_t field_length = uStrainBuffer[2];
				uint8_t field_sensorFileriptor = uStrainBuffer[3];

				if(payload_length == field_length){
					uStrain_byte_cnt = read(sensorFile, &uStrainBuffer, payload_length);
					switch(sensorFileriptor_set){
						case 0x01:
							if(field_sensorFileriptor == 0xF1){
								if(uStrainBuffer[1] == 0x00){
									cout << uStrainBuffer[0] << ": ACK" << endl;
								}else{
									cout << uStrainBuffer[0] << ": NACK" << endl;
								}
							}
							break;
						case 0x80:
							switch(field_sensorFileriptor){
								case 0x0A:	// Quaternion
									imuPoint = Quaternion <float> (swap_bytes(&uStrainBuffer[0]), swap_bytes(&uStrainBuffer[4]), swap_bytes(&uStrainBuffer[8]), swap_bytes(&uStrainBuffer[12]));
									break;
								case 0x0C:	// Euler Angles
									// do nothing;
								default:
									break;
							}
							break;
						case 0x81:
							switch(field_sensorFileriptor){
								case 0x03:
									uint8_t valid = uStrainBuffer[48];
									uint8_t lat_lng_valid = valid & 0x01;
									uint8_t elipsoid_height_valid = valid & 0x02;
									uint8_t msl_height_valid = valid & 0x04;
									uint8_t horizontal_accuracy_valid = valid & 0x08;
									uint8_t vertical_accuracy_valid  = valid & 0x10;

									break;
							}
					}
				}
			}
			// TODO Check that sensor update is correct

			// Update setPoint and execute
			Quaternion <float> moveQuat = setPoint / imuPoint;
			float movePoints[3];
			moveQuat.toEuler(movePoints);

			rollServo.setAngle(movePoints[0]);
			pitchServo.setAngle(movePoints[1]);
			curYawVel = movePoints[2] - prevYaw;
			curYawErr += movePoints[2] - yawSetPt;
			prevYaw = movePoints[2];
			float yawSpeed = YAW_P * (movePoints[2] - yawSetPt) + YAW_I * (curYawErr) + YAW_D * curYawVel;
			// TODO send velocity to yawServo

			// Publish data
		}
	}
}
