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
 */

// Defines
#define VERSION 0.0

// Includes
#include <stdio>
#include <stdint>

int main(int argc, char** argv){
	cout << "Project Spectre BeagleBone Black\r\n";
	cout << "Version: " << VERSION << "\r\n";
	
	string cmd;
	uint8_t runState = true;
	while(runState){
		// Get IMU State
		// Calculate error
		// Implement correction
	}
	return 0;
}
