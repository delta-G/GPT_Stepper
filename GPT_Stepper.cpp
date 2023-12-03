// #include "includes/ra/fsp/src/bsp/cmsis/Device/RENESAS/Include/R7FA4M1AB.h"
// /*

// GPT_Stepper.cpp  --  Control stepper motor pulses using a GPT timer
//      Copyright (C) 2023  David C.

//      This program is free software: you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation, either version 3 of the License, or
//      (at your option) any later version.

//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.

//      You should have received a copy of the GNU General Public License
//      along with this program.  If not, see <http://www.gnu.org/licenses/>.

//      */

#include "GPT_Stepper.h"

const uint32_t clock_uHz = (F_CPU / 1000000);

void setSpeed(int stepsPerSecond) {
	uint32_t us = 1000000UL / stepsPerSecond;
	setPeriod(us);
}

uint16_t getMinSpeed() {
	// get the minimum speed without changing divider
	return (F_CPU) / (getTimerResolution() * getDivider());
}

uint16_t getDivider() {
	return ((R_GPT3->GTCR >> (R_GPT0_GTCR_TPCS_Pos + 1)) & 0x07) * 4;
}

uint32_t getTimerResolution() {
	return 0xFFFF;  // Not supporting 32 bit yet
}

void setPeriod(uint32_t us) {
	uint8_t divbits = 0;
	// PCLKD is running at full system speed, 48MHz.  That's 48 ticks per microsecond.
	uint32_t ticks = (clock_uHz * us);
	uint32_t timerResolution = getTimerResolution(); // We'll need this when we support 32 bit timers
	uint32_t resetCount = 0xFFFF;
	// Find smallest divider that works.  1, 4, 16, 64, 256, 1024
	uint8_t div = 0;
	for (div = 0; div < 5; div++) {
		if (ticks < timerResolution) {
			resetCount = ticks;
			break;
		}
		ticks /= 4;
	}
	uint8_t currentDiv = ((R_GPT3->GTCR >> (R_GPT0_GTCR_TPCS_Pos + 1)) & 0x07);
	if (currentDiv == div) {
		// if the prescaler still works then we can just set the reset value
		R_GPT3->GTPBR = resetCount;
	} else {
		Serial.print("Current Div :");
		Serial.print(currentDiv);
		Serial.print("  div : ");
		Serial.println(div);
//     // stops counter and sets prescaler
		R_GPT3->GTSTP = (1 << 3);
		R_GPT3->GTCR = (div << (R_GPT0_GTCR_TPCS_Pos + 1)); // R_GPT0_GTCR_TPCS_Pos is wrong.
		R_GPT3->GTPR = resetCount;

		// reset the compare match for the new prescaler
		uint32_t hiTicks = 48ul * 3;    // 3 microsecond pulse
		if (div)
			hiTicks /= (4 * div);  // Scale to the prescaler.
		if (hiTicks < 2)
			hiTicks = 2;   // minimum pulse
		R_GPT3->GTCCR[0] = hiTicks;

		// figure out our current place in the count with the new divider
		uint32_t currentCount = R_GPT3->GTCNT;
		if (div > currentDiv) {
			// If the new divider is larger then the top value will lekely be smaller
			// So we need to check the count is not already past 
			uint32_t newCount = (currentCount * currentDiv) / div;
			if (newCount >= resetCount) {
				newCount = resetCount;
			}
			R_GPT3->GTCNT = newCount;
		} else {
			// since the new divider is smaller, the top value will 
			// likely be larger so let's just check that it's in range.
			if (currentCount >= resetCount) {
				R_GPT3->GTCNT = resetCount;
			}
		}

//     // restart the timer
		R_GPT3->GTCR |= 1;
	}
}
