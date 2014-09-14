/**
 * BBB_SACP Control Process
 */

// Includes
#include <zmq.hpp>
#include "quaternion.hpp"
#include "MPU9150.hpp"

// Defines

// Global Variables

int main(int argc, char** argv){
	// Set up communications
	// Initialize single ZMQ context with single I/O thread
	zmq::context_t control_Context (1);
	zmq::socket_t control_Socket (control_Context, ZMQ_REP);
	control_Socket.bind("tcp://*:53679");

	// Set up sensors

	// Set up stabilization

	// Begin doing stuff
	while(1){

	}
}
