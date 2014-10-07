#include "Servo.hpp"

Servo::Servo(const uint8_t pin, const uint32_t frequency, 
		const uint32_t lowPulse, const uint32_t highPulse, 
		const float msDegRatio):_pwmPin(pin, frequency), _lowPulse(lowPulse), 
		_highPulse(highPulse), _pulseScaling(msDegRatio), _curPulse(15000000){
	_pwmPin.setPolarity(PWM_PULSE_HIGH);
}

void Servo::setAngle(const float degrees){
	uint32_t curPulse = degrees * _pulseScaling + (_lowPulse + _highPulse) / 2;
	assert(curPulse >= _lowPulse && curPulse <= _highPulse);
	_curPulse = curPulse;
	_curLocation = degrees;
	_pwmPin.setOnTime(_curPulse);
}

int Servo::getAngle(){
	return _curLocation;
}
