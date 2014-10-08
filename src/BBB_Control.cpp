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
using namespace std;

// Defines

// Global Variables
Quaternion <float> setPoint;
Quaternion <float> imuPoint;
bool _stabilization = true;
float yawSetPoint = 0;

int main(int argc, char** argv){
	// Set up communications
	// Initialize single ZMQ context with single I/O thread
	cout << "Initializing..." << flush;
	zmq::context_t control_Context (1);
	zmq::socket_t control_Socket (control_Context, ZMQ_REP);
	control_Socket.bind("tcp://127.0.0.1:53679");

	zmq::context_t data_Context (1);
	zmq::socket_t data_Socket (data_Context, ZMQ_PUB);
	data_Socket.bind("tcp://127.0.0.1:53680");

	zmq::context_t sensor_Context (1);
	zmq::socket_t sensor_Socket(sensor_Context, ZMQ_SUB);
	sensor_Socket.connect("tcp://127.0.0.1:53681");
	cout << "done." << endl;

	// Set up sensors
//	MPU9150 sensor (1);
//	int result = sensor.initialize();
//	if(result){
//		return 1;
//	}
//
//	if((result = getSensorState())){
//		return 1;
//	}
//	// TODO: enable and configure FIFO
//
	// Set up stabilization
	cout << "Initializing Stabilization..." << flush;
	setPoint = Quaternion <float> (1, 0, 0, 0);
	imuPoint = Quaternion <float> (1, 0, 0, 0);
	float sensorUpdate[4];

	Servo pitchServo (PWM_P8_13, 50, 740000, 2300000, 5888888);
	Servo rollServo(PWM_P8_34, 50, 740000, 2300000, 5888888);
	//CRServo yawServo(PWM_P8_45, 50, 670000, 1534000, 2400000);
	cout << "done." << endl;

	// TODO: enable and configure interrupt driven sensor update

	// Begin doing stuff
	cout << "Beginning control loops:" << endl;
	while(1){
		zmq::message_t command;
//		cout << "Checking for command..." << flush;
		if(control_Socket.recv(&command, ZMQ_NOBLOCK)){
			// process command
//			cout << "Have command!\nProcessing command..." << flush;
			std::istringstream iss(static_cast <char*> (command.data()));
			string cmd;
			iss >> cmd;
			if(!cmd.compare("SETPOINT")){
				float setQuat[4];
				iss >> setQuat[0] >> setQuat[1] >> setQuat[2] >> setQuat[3];
				setPoint = Quaternion<float>(setQuat);
				cout << "Have setpoing!" << endl;
			}else if(!cmd.compare("STABILIZATION")){
				iss >> _stabilization;
//				cout << "Have stabilize command!" << endl;
			}
		}else{
//			cout << "no command." << endl;
		}
		// No command, continue

		// Check for sensor update
//		cout << "Checking for sensor update..." << flush;
		zmq::message_t sensor_update;
		if(sensor_Socket.recv(&sensor_update, ZMQ_NOBLOCK)){
//			cout << "Have update!\nProcessing update..." << flush;
			// got update!
			std::istringstream iss(static_cast <char*> (sensor_update.data()));
			iss >> sensorUpdate[0] >> sensorUpdate[1] >> sensorUpdate[2] >> sensorUpdate[3];
			imuPoint = Quaternion <float> (sensorUpdate);
//			cout << "done." << endl;
		}else{
//			cout << "no update." << endl;
		}
		// Update setPoint and execute
//		cout << "Updating control loop..." << flush;
		Quaternion <float> moveQuat = setPoint / imuPoint;
		float movePoints[3];
		moveQuat.toEuler(movePoints);

		rollServo.setAngle(movePoints[0]);
		pitchServo.setAngle(movePoints[1]);
//		cout << "done." << endl;

		// Publish data
	}
}
