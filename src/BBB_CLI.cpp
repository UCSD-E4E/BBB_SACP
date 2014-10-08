/**
 * UI Controller for Project Spectre BeagleBoneBlack branch
 * 
 * The concept for this controller is to have a control process run in the
 * background, interpreting the UI wants, while the UI provides both a GUI 
 * interface (selectable) to the user.  The control process will be CLI
 * controlled, and provide a debugging interface (again CLI).
 *
 * This program will provide the basic control interface.  It will directly
 * control the gimbal and associated hardware via CLI.  The program will
 * react in the following manner:
 * * At startup, the gimbal will automatically tune to a level orientation in
 * * a stabilized manner.  Yaw will not be stabilized.
 *
 * Author: Nathan Hui
 * Date: 21 Jul 2014
 */

/*
 * CHANGELOG
 * 7/21/14
 * * Lay out structure
 * * NOTE: Eventually will have to find a library to provide the proper
 * * * non-blocking behavior for the CLI.
 *
 * 7/29/14
 * * Added main CLI interface
 * * Added quaternion math support
 * * Added stabilize state feature
 * * Note: will now use 0MQ to implement separate process control for hardware
 * * Added display setpoint feature
 * 
 * 9/8/14
 * * Fixed .h to .hpp reference for quaternion library
 */
 

// Defines
#define VERSION 	0.0
#define MAX_BUF 	256
#define R_FIVE_DEG	0.9990482216 // real part of five degree rotation quaternion
#define I_FIVE_DEG	0.0436193874 // imaginary part of five degree rotation quaternion
#define R_ONE_DEG	0.9999619231 // real part of one degree rotation quaternion
#define I_ONE_DEG	0.0087265352 // imaginary part of one degree rotation quaternion

// Includes
#include <iostream>
#include <cstdio>
#include <cstdint>
#include <string>
#include <quaternion.hpp>
#include <zmq.hpp>

using namespace std;

// Variables
uint8_t stabilize = true;
uint8_t displayAngle = false;
Quaternion <float> setPoint;
Quaternion <float> ROLL_BUMP = Quaternion <float> (R_FIVE_DEG, I_FIVE_DEG, 0, 0);
Quaternion <float> PITCH_BUMP = Quaternion <float> (R_FIVE_DEG, 0, I_FIVE_DEG, 0);
Quaternion <float> YAW_BUMP = Quaternion <float> (R_FIVE_DEG, 0, 0, I_FIVE_DEG);

// Function definitions
void printHelp();
void printQuaternion(Quaternion <float> q);

int main(int argc, char** argv){
	int rc = 0;
	cout << "Project Spectre BeagleBone Black\n";
	cout << "Version: " << VERSION << endl;
	
	cout << endl << "Initializing..." << endl;

	setPoint = Quaternion <float> (1, 0, 0, 0);
	// set up communications
	zmq::context_t control_Context(1);
	zmq::socket_t control_Socket(control_Context, ZMQ_REQ);
	control_Socket.connect("tcp://127.0.0.1:55000");

	zmq::context_t data_Context(1);
	zmq::socket_t data_Socket(data_Context, ZMQ_SUB);
	data_Socket.connect("tcp://127.0.0.1:55002");

	cout << "Initialized" << endl;
	
	std::string cmd("");
	uint8_t runState = true;
	uint8_t goodCmd = false;
	while(runState){
		goodCmd = false;
		cout << ">";
		cin >> cmd;
		// Process commands
		if(!cmd.compare("exit")){// Compare to "exit"
			goodCmd = true;
			cout << "Exiting..." << endl;
			runState = false;
		}
		if(!cmd.compare("w")){	// compare to "w"
			goodCmd = true;
			// bump absolute pitch setpoint 5 degrees up
			setPoint = PITCH_BUMP * setPoint;
		}
		if(!cmd.compare("s")){	// compare to "s"
			goodCmd = true;
			// bump absolute pitch setpoint 5 degrees down
			setPoint = PITCH_BUMP.inverse() * setPoint;
		}
		if(!cmd.compare("d")){	// compare to "a"
			goodCmd = true;
			// bump absolute yaw setpoint 5 degrees left
			setPoint = YAW_BUMP * setPoint;
		}
		if(!cmd.compare("a")){	// compare to "d"
			goodCmd = true;
			// bump absolute yaw setpoint 5 degrees right
			setPoint = YAW_BUMP.inverse() * setPoint;
		}
		if(!cmd.compare("e")){	// compare to "q"
			goodCmd = true;
			// bump absolute roll setpoint 5 degrees left
			setPoint = ROLL_BUMP * setPoint;
		}
		if(!cmd.compare("q")){	// compare to "e"
			goodCmd = true;
			// bump absolute roll setpoint 5 degrees right
			setPoint = ROLL_BUMP.inverse() * setPoint;
		}
		if(!cmd.compare("stabilize")){	// compare to "stabilize"
			goodCmd = true;
			cin >> cmd;
			if(!cmd.compare("on")){	// compare next token to "on"
				// activate stabilize
				stabilize = true;
				cout << "Stabilization Active...\n";
			}else if(!cmd.compare("off")){	// compare next token to "off"
				// deactivate stabilize
				stabilize = false;
				cout << "Stabilization Inactive\n";
			}else{
				cout << "Improper usage!" << endl;
				printHelp();
			}
		}
		if(!cmd.compare("control")){	// compare to "control"
			goodCmd = true;
			cin >> cmd;
			if(!cmd.compare("coarse")){
				// switch to coarse
				YAW_BUMP = Quaternion <float> (R_FIVE_DEG, 0, 0, I_FIVE_DEG);
				ROLL_BUMP = Quaternion <float> (R_FIVE_DEG, I_FIVE_DEG, 0, 0);
				PITCH_BUMP = Quaternion <float> (R_FIVE_DEG, 0, I_FIVE_DEG, 0);
			}else if(!cmd.compare("fine")){
				// switch to fine
				YAW_BUMP = Quaternion <float> (R_ONE_DEG, 0, 0, I_ONE_DEG);
				ROLL_BUMP = Quaternion <float> (R_ONE_DEG, I_ONE_DEG, 0, 0);
				PITCH_BUMP = Quaternion <float> (R_ONE_DEG, 0, I_ONE_DEG, 0);
			}else{
				cout << "Improper usage!" << endl;
				printHelp();
			}
		}
		if(!cmd.compare("reset")){	// compare to "reset"
			goodCmd = true;
			setPoint = Quaternion <float> (1, 0, 0, 0);
			YAW_BUMP = Quaternion <float> (R_FIVE_DEG, 0, 0, I_FIVE_DEG);
			ROLL_BUMP = Quaternion <float> (R_FIVE_DEG, I_FIVE_DEG, 0, 0);
			PITCH_BUMP = Quaternion <float> (R_FIVE_DEG, 0, I_FIVE_DEG, 0);
		}
		if(!cmd.compare("help") || !cmd.compare("?")){	// compare to "help"
			goodCmd = true;
			printHelp();
		}
		if(!cmd.compare("getAngle")){
			goodCmd = true;
			printQuaternion(setPoint);
		}
		if(!cmd.compare("displayAngle")){	// compare to "displayAngle"
			goodCmd = true;
			cin >> cmd;
			if(!cmd.compare("on")){
				displayAngle = true;
			}else if(!cmd.compare("off")){
				displayAngle = false;
			}else{
				cout << "Improper usage!" << endl;
				printHelp();
			}
		}
		if(!goodCmd){
			cout << "Improper usage!" << endl;
			printHelp();
		}
		// Send command;
		zmq::message_t control_Msg;
		memcpy((void*)control_Msg.data(), cmd.c_str(), cmd.size());
		control_Socket.send(control_Msg);
		control_Socket.recv(&control_Msg);
		if(displayAngle){
			printQuaternion(setPoint);
		}
		// Get IMU State
		// Calculate error
		// Implement correction
	}
	return 0;
}

/**
 * This function prints the normal help text for the program.
 *
 */
void printHelp(){
	cout << "Valid Commands:\n";
	cout << "    exit                    Cleanly exits the program\n";
	cout << "    stabilize [on|off]      Turns on/off active stabilization.  Active\n";
	cout << "                            stabilization attempts to maintain the orientation\n";
	cout << "                            of the camera relative to the ground.  Inactive \n";
	cout << "                            stabilization maintains the orientation of the\n";
	cout << "                            camera relative to the platform body.\n";
	cout << "    w                       Bumps the pitch setpoint up 5 degrees\n";
	cout << "    s                       Bumps the pitch setpoint down 5 degrees\n";
	cout << "    a                       Bumps the yaw setpoint left 5 degrees\n";
	cout << "    d                       Bumps the yaw setpoint right 5 degrees\n";
	cout << "    q                       Bumps the roll setpoint counterclockwise 5 degrees\n";
	cout << "    e                       Bumps the roll setpoint clockwise 5 degrees\n";
	cout << "    control [fine|coarse]   Changes bump angle.  Coarse bump is 5 degrees.\n";
	cout << "                            Fine bump is 1 degree.\n";
	cout << "    getAngle                Prints out the current setpoint in both quaternion\n";
	cout << "                            and Euler angle form\n";
	cout << "    displayAngle [on|off]   Turns on/off angle display for each command\n";
	return;
}

/**
 * This function prints out the values of a quaternion in a structure format.
 *
 */
void printQuaternion(Quaternion <float> q){
	float quatVals[4];
	q.getValues(quatVals);
	cout << "Q: (";
	printf("%.3f", quatVals[0]);
	cout << " (";
	printf("%.3f", quatVals[1]);
	cout << ", ";
	printf("%.3f", quatVals[2]);
	cout << ", ";
	printf("%.3f", quatVals[3]);
	cout << "))\t";
	
	float eulerAngles[3];
	q.toEuler(eulerAngles);
	cout << "A: (";
	printf("%.3f", eulerAngles[0]);
	cout << ", ";
	printf("%.3f", eulerAngles[1]);
	cout << ", ";
	printf("%.3f", eulerAngles[2]);
	cout << ")" << endl;
}
