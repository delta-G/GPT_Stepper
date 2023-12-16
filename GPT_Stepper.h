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

#if !defined(ARDUINO_UNOR4_WIFI)
#error This library is only supported on the UNO-R4-WiFi at this time.  It does not support older UNO boards.
#endif

#include "Arduino.h"

typedef enum {
	CHANNEL_A, CHANNEL_B
} gpt_channel_t;

class GPT_Stepper {

protected:
	enum Direction_t {
		D_CCW, D_CW
	};
	friend void GPT0_ISR();
	static GPT_Stepper* isrRegistry[8];
	void internalISR();

private:

	R_GPT0_Type *timer;
	gpt_channel_t channel;
	static bool timerClaimed[8];
	

	uint8_t directionPin;
	uint8_t stepPin;

	void setupStepPin(uint8_t port, uint8_t pin);
	void setupTimer();
	void setDirection(Direction_t direction);

	bool timerRunning();
	void startTimer();
	void stopTimer();

	uint16_t getDivider();
	uint32_t getTimerResolution();
		

public:
	GPT_Stepper(uint8_t spin, uint8_t dpin) :
			directionPin(dpin), stepPin(spin) {
	}
	bool init();
	void setSpeed(float stepsPerSecond);
	void stop();
	void setPeriod(uint32_t us);

	GPT_Stepper() = delete;
	GPT_Stepper(const GPT_Stepper&) = delete;
	GPT_Stepper& operator =(const GPT_Stepper&) = delete;

};

#endif /* GPT3_STEPPER_H */
