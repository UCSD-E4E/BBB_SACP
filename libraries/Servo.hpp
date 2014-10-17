#include "PWM.hpp"

class Servo{
	public:
		Servo(const pwm_pins_t pin, const uint32_t frequency, const uint32_t lowPulse, const uint32_t highPulse, const int rotateRange);
		void setAngle(const float degrees);
		int getAngle();
	private:
		PWM _pwmPin;
		int32_t _curLocation;
		uint32_t _lowPulse;
		int _range;
		uint32_t _highPulse;
		uint32_t _curPulse;
};
