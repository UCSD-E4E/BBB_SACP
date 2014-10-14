#ifndef _CRSERVO
#include "PWM.hpp"

class CRServo{
	public:
		CRServo(const pwm_pins_t pin, const uint32_t frequency, const uint32_t zeroSpeed, const uint32_t maxForwardSpeed, const uint32_t maxBackwardSpeed);
		void setSpeed(const float speed);
		float getSpeed();
	private:
		PWM _pwmPin;
		uint32_t _zeroSpeed;
		uint32_t _maxPulse;
		uint32_t _minPulse;
		float _curSpeed;
};
#endif
