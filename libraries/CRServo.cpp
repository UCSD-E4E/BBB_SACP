#include "CRServo.hpp"

CRServo::CRServo(const pwm_pins_t pin, const uint32_t frequency,
		const uint32_t zeroSpeed, const uint32_t maxForwardSpeed,
		const uint32_t maxBackwardSpeed):_pwmPin(pin, frequency),
		_zeroSpeed(zeroSpeed), _maxPulse(maxForwardSpeed),
		_minPulse(maxBackwardSpeed){
	_pwmPin.setPolarity(PWM_PULSE_HIGH);
	_pwmPin.setOnTime(_zeroSpeed);
}

void CRServo::setSpeed(const float speed){
	assert(speed >= -1 && speed <= 1);
	uint32_t curSpeed;
	if(speed < 0){
		curSpeed = speed * (_zeroSpeed - _minPulse) + _zeroSpeed;
	}else{
		curSpeed = speed * (_maxPulse - _zeroSpeed) + _zeroSpeed;
	}
	_curSpeed = speed;
	_pwmPin.setOnTime(curSpeed);
}

float CRServo::getSpeed(){
	return _curSpeed;
}
