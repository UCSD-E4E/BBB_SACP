/**
 * dataStructure class holds the data for the controller process and provides
 * convenience functions to help expose the data.
 */
dataStructure::dataStructure(){
	_roll = 0;
	_pitch = 0;
	_yaw = 0;
}

dataStructure::dataStructure(const dataStructure& reference){
	_roll = reference->_roll;
	_pitch = reference->_pitch;
	_yaw = reference->_yaw;
}

void* dataStructure::getZMQMessage(){


