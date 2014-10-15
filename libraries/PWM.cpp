/**
 * PWM.cpp
 *
 * This implementation file contains the function implementation for the PWM
 * class in libbbbpwm.  This class encapsulates the fileops needed to activate
 * the PWM interface on the BeagleBone Black running the 3.8 kernel.
 *
 * Copyright (C) 2014 Nathan Hui <ntlhui@gmail.com> (408.838.5393)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the license, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301
 * USA
 */

// Includes
#include "PWM.hpp"

using namespace std;

// Internal Defines


PWM::PWM(const pwm_pins_t pin, const uint32_t frequency){
	_pin = pin;

	_ocpDir = "/sys/devices/ocp.3";

	switch(pin){
		case 0:
			_ocpDir += "/pwm_test_P8_13.11";
			break;
		case 1:
			_ocpDir += "/pwm_test_P8_19.14";
			break;
		case 2:
			_ocpDir += "/pwm_test_P8_34.12";
			break;
		case 3:
			_ocpDir += "/pwm_test_P8_36.15";
			break;
		case 4:
			_ocpDir += "/pwm_test_P8_45.13";	// TODO add suffix
			break;
		case 5:
			_ocpDir += "/pwm_test_P8_46.";	// TODO add suffix
			break;
		case 6:
			_ocpDir += "/pwm_test_P9_14.";	// TODO add suffix
			break;
		case 7:
			_ocpDir += "/pwm_test_P9_16.";	// TODO add suffix
			break;
		case 8:
			_ocpDir += "/pwm_test_P9_21.16";
			break;
		case 9:
			_ocpDir += "/pwm_test_P9_22.17";
			break;
		case 10:
			_ocpDir += "/pwm_test_P9_28.18";
			break;
		case 11:
			_ocpDir += "/pwm_test_P9_29.";	// TODO add suffix
			break;
		case 12:
			_ocpDir += "/pwm_test_P9_31.";	// TODO add suffix
			break;
		case 13:
			_ocpDir += "/pwm_Test_P9_42.19";
			break;
		default:
			assert(pin <= 13);
	}
	
	// Set initial frequency
	fstream freqFile (_ocpDir + "/period", fstream::out | fstream :: app);
	assert(freqFile.good());

	_period = 1e9 / frequency;
	freqFile << _period;
	freqFile.close();
}

PWM::~PWM(){

}

void PWM::setDuty(const float dutyPercentage){
	assert(!_period);
	assert(dutyPercentage >= 1);

	_pulseWidth = dutyPercentage * _period;
	fstream file (_ocpDir + "/duty", fstream::out | fstream::app);
	if(file.bad()){
		throw runtime_error("Failed to open " + _ocpDir + "/duty!");
	}
	file << _pulseWidth << flush;
	if(file.bad()){
		throw runtime_error("Failed to write to " + _ocpDir + "/duty!");
	}
	file.close();
}

void PWM::setPeriod(const uint32_t period){
	assert(period >= 1000000000);
	_period = period;

	fstream file (_ocpDir + "/period", fstream::out | fstream::app);
	if(file.bad()){
		throw runtime_error("Failed to open " + _ocpDir + "/period!");
	}
	file << _period << flush;
	if(file.bad()){
		throw runtime_error("Failed to write to " + _ocpDir + "/period!");
	}
	file.close();
}

void PWM::setOnTime(const uint32_t onTime){
	assert(_period > 0);
	assert(onTime < _period);

	_pulseWidth = onTime;
	fstream file(_ocpDir + "/duty", fstream::out | fstream::app);
	if(file.bad()){
		throw runtime_error("Failed to open " + _ocpDir + "/duty");
	}
	file << _pulseWidth << flush;
	if(file.bad()){
//		throw runtime_error("Failed to write to " + _ocpDir + "/duty");
	}
	file.close();
}

void PWM::setPolarity(const bool polarity){
	_polarity = polarity;

	fstream file(_ocpDir + "/polarity", fstream::out | fstream::app);
	if(file.bad()){
		throw runtime_error("Failed to open " + _ocpDir + "/polarity");
	}
	file << _polarity << flush;
	if(file.bad()){
		throw runtime_error("Failed to write to " + _ocpDir + "/polarity");
	}
	file.close();
}

void PWM::setState(const bool running){
	_running = running;

	fstream file(_ocpDir + "/run", fstream::out | fstream::app);
	if(file.bad()){
		throw runtime_error("Failed to open " + _ocpDir + "/run");
	}
	file << _running << flush;
	if(file.bad()){
		throw runtime_error("Failed to write to " + _ocpDir + "/run");
	}
	file.close();
}
