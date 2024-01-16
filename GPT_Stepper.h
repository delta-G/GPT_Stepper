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
#include "IRQManager.h"
#include "FspTimer.h"

class GPT_Stepper {

protected:
	enum Direction_t {
		D_CCW, D_CW
	};
	enum gpt_channel_t {
		CHANNEL_A, CHANNEL_B
	};
	friend void GPT0_ISR();
	friend void GPT1_ISR();
	friend void GPT2_ISR();
	friend void GPT3_ISR();
	friend void GPT4_ISR();
	friend void GPT5_ISR();
	friend void GPT6_ISR();
	friend void GPT7_ISR();
	static GPT_Stepper *isrRegistry[8];
	void internalISR();
	uint8_t eventLinkIndex;

private:

	R_GPT0_Type *timer;
	gpt_channel_t channel;
	static bool timerClaimed[8];

	uint8_t directionPin;
	uint8_t stepPin;
	bool invert;
	volatile long position;
	volatile float speed;
	float requestedSpeed;
	float acceleration;

	void setupStepPin(uint8_t port, uint8_t pin);
	void setupTimer();
	void setupInterrupt(uint8_t ch, void (*isr)());
	void setCurrentSpeed(float stepsPerSecond);
	void setPeriod(uint32_t us);
	float getNewSpeed();
	void setDirection(Direction_t direction);

	void stopTimer();
	void startTimer();
	bool timerRunning();

	uint16_t getDivider();
	uint32_t getTimerResolution();

public:
	GPT_Stepper(uint8_t spin, uint8_t dpin) :
			GPT_Stepper(spin, dpin, 100.0) {
	}
	GPT_Stepper(uint8_t spin, uint8_t dpin, float acc) :
			GPT_Stepper(spin, dpin, acc, false) {
	}
	GPT_Stepper(uint8_t spin, uint8_t dpin, float acc, bool inv) :
			directionPin(dpin), stepPin(spin), acceleration(acc), invert(inv) {
	}
	bool init();
	void setAcceleration(float stepsPerSecondPerSecond);
	void setSpeed(float stepsPerSecond);
	void stop();
	long getPosition();
	void setHome();
	float getCurrentSpeed();
	float getAcceleration();
	bool atSpeed();

	GPT_Stepper() = delete;
	GPT_Stepper(const GPT_Stepper&) = delete;
	GPT_Stepper& operator =(const GPT_Stepper&) = delete;

};

void GPT0_ISR();
void GPT1_ISR();
void GPT2_ISR();
void GPT3_ISR();
void GPT4_ISR();
void GPT5_ISR();
void GPT6_ISR();
void GPT7_ISR();

#endif /* GPT3_STEPPER_H */
