#include "Servo.hpp"
#include <iostream>
using namespace std;


Servo::Servo(const pwm_pins_t pin, const uint32_t frequency, 
		const uint32_t lowPulse, const uint32_t highPulse, 
		const int rotateRange):_pwmPin(pin, frequency), _lowPulse(lowPulse), 
		_highPulse(highPulse), _range(rotateRange), _curPulse(15000000){
	_pwmPin.setPolarity(PWM_PULSE_HIGH);
}

void Servo::setAngle(const float degrees){
	uint32_t curPulse = degrees / _range * (_highPulse - _lowPulse) + _lowPulse;
	assert(curPulse >= _lowPulse && curPulse <= _highPulse);
	_curPulse = curPulse;
	_curLocation = degrees;
	_pwmPin.setOnTime(_curPulse);
}

int Servo::getAngle(){
	return _curLocation;
}
