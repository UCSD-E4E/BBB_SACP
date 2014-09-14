/**
 * BBB_SACP Control Process
 */

// Includes
#include <zmq.hpp>
#include "quaternion.hpp"
#include "MPU9150.hpp"
#include "dataStructure.hpp"
using namespace std;

// Defines

// Global Variables


int main(int argc, char** argv){
	// Set up communications
	// Initialize single ZMQ context with single I/O thread
	zmq::context_t control_Context (1);
	zmq::socket_t control_Socket (control_Context, ZMQ_REP);
	control_Socket.bind("tcp://*:53679");

	zmq::context_t data_Context (1);
	zmq::socket_t data_Socket (data_Context, ZMQ_PUB);
	data_Socket.bind("tcp://eth0:53680");

	// Set up sensors
	MPU9150 sensor (1);
	int result = sensor.initialize();
	if(result){
		return 1;
	}

	if((result = getSensorState())){
		return 1;
	}


	// Set up stabilization

	// Begin doing stuff
	while(1){

	}
}
