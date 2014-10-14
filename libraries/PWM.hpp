/**
 * PWM.hpp
 *
 * This header file contains the function definitions for the PWM class in
 * libbbbpwm.  This class encapsulates the fileops needed to activate the PWM
 * interface on the BeagleBone Black running the 3.8 kernel.
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

#ifndef _BBB_PWM
#define _BBB_PWM

// Includes
#include <cstdlib>
#include <assert.h>
#include <cstdio>
#include <sys/types.h>
#include <unistd.h>
#include <ftw.h>
#include <string>
#include <stdexcept>
#include <fstream>
#include <iostream>

// Constants Definitions
enum pwm_pins_t{
	PWM_P8_13 = 0,
	PWM_P8_19 = 1,
	PWM_P8_34 = 2,
	PWM_P8_36 = 3,
	PWM_P8_45 = 4,
	PWM_P8_46 = 5,
	PWM_P9_14 = 6,
	PWM_P9_16 = 7,
	PWM_P9_21 = 8,
	PWM_P9_22 = 9,
	PWM_P9_28 = 10,
	PWM_P9_29 = 11,
	PWM_P9_31 = 12,
	PWM_P9_42 = 13,
}

#define PWM_PULSE_HIGH	0
#define PWM_PULSE_LOW	1
#define PWM_ON	1
#define PWM_OFF	0

class PWM{
	public:
		PWM(const pwm_pins_t pin, const uint32_t frequency);
		~PWM();
		void setDuty(const float dutyPercentage);
		void setPeriod(const uint32_t period);
		void setPolarity(const bool polarity);
		void setOnTime(const uint32_t onTime);
		void setState(const bool running);
	private:
		uint8_t _pin;
		uint32_t _period;
		uint32_t _pulseWidth;
		bool _polarity;
		bool _running;
		uint8_t _capeMgrNo;
		std::string _ocpDir;
		/**
		 * Checks the file tree walk (ftw) entry for bone_capemgr number.
		 */
		static int _checkCapeMgr(const char* fpath, const struct stat* sb, 
				int tflag, struct FTW *ftwbuf);	
};

#endif
