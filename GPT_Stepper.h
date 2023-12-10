/*

GPT_Stepper.h  --  Control stepper motor pulses using a GPT timer
     Copyright (C) 2023  David C.

     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.

     */

#ifndef GPT3_STEPPER_H
#define GPT3_STEPPER_H

#include "Arduino.h"

typedef enum {
	CHANNEL_A,
	CHANNEL_B
} gpt_channel_t;

class GPT_Stepper {
	
	R_GPT0_Type* timer;
	gpt_channel_t channel;
	uint8_t divBits;
	uint8_t directionPin;
	uint8_t stepPin;
	void setupStepPin();
	
public:
	GPT_Stepper(uint8_t spin, uint8_t dpin) : directionPin(dpin), stepPin(spin) {}
	bool init();
	void setupTimer();
	void setPeriod(uint32_t us);
	void setSpeed(int stepsPerSecond);
	uint16_t getDivider();
	uint32_t getTimerResolution();
	uint16_t getMinSpeed();
	
};


#endif /* GPT3_STEPPER_H */