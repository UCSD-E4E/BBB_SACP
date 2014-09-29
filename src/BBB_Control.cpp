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
Quaternion <float> setPoint;

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
	// TODO: enable and configure FIFO

	// Set up stabilization
	setPoint = Quaternion <float> (1, 0, 0, 0);
	// TODO: enable and configure interrupt driven sensor update

	// Begin doing stuff
	while(1){
		zmq::message_t command ();
		result = zmq_msg_init(&command);
		assert(result == 0);
		result = zmq_recv(&data_Socket, &command, ZMQ_NOBLOCK);
		if(!(result == -1 && errno == EAGAIN) || (result >= 0)){
			assert(result == 0);
		}else if(result >= 0){
			// handle command
		}
		// No command, continue

		// Check for sensor update
		
		// Update setPoint

		// Publish data
	}
}
