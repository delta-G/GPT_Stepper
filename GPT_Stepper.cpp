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
	return 1 << (((R_GPT3->GTCR >> (R_GPT0_GTCR_TPCS_Pos + 1)) & 0x07) * 2);
}

uint32_t getTimerResolution() {
	return 0xFFFF;  // Not supporting 32 bit yet
}

void setPeriod(uint32_t us) {
	// PCLKD is running at full system speed, 48MHz.  That's 48 ticks per microsecond.
	uint32_t ticks = (clock_uHz * us);
	uint32_t timerResolution = getTimerResolution(); // We'll need this when we support 32 bit timers
	uint32_t resetCount = 0xFFFF;
	// Find smallest divider that works.  1, 4, 16, 64, 256, 1024
	uint8_t div = 0;
	for (div = 0; div < 5; div++) {
		if (ticks < (timerResolution - 2000)) {
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

//     // stops counter and sets prescaler
//		R_GPT3->GTSTP = (1 << 3);
		R_GPT3->GTCR = 0;
		R_GPT3->GTCR = (div << (R_GPT0_GTCR_TPCS_Pos + 1)); // R_GPT0_GTCR_TPCS_Pos is wrong.
		R_GPT3->GTPR = resetCount;
		R_GPT3->GTPBR = resetCount;
		// reset the compare match for the new prescaler
		uint32_t hiTicks = 48ul * 3;    // 3 microsecond pulse
		if (div) {
			hiTicks /= (1 << (div * 2));  // Scale to the prescaler.
		}
		if (hiTicks < 2) {
			hiTicks = 2;   // minimum pulse
		}
		R_GPT3->GTCCR[0] = hiTicks;

		// figure out our current place in the count with the new divider
		uint32_t currentCount = R_GPT3->GTCNT;
		uint32_t newCount = 0;
		if (div > currentDiv) {
			// If the new divider is larger then the top value will lekely be smaller
			// So we need to check the count is not already past 
			newCount = (currentCount * (1 << (currentDiv * 2)))
					/ (1 << (div * 2));
			if (newCount >= resetCount) {
				newCount = resetCount - 1;
			} else if (newCount < hiTicks) {
				newCount = hiTicks + 1;
			}
		} else {
			// since the new divider is smaller, the top value will 
			// likely be larger so let's just check that it's in range.

			if (currentCount >= resetCount) {
				newCount = resetCount - 1;
			} else if (currentCount < hiTicks) {
				newCount = hiTicks + 1;
			} else {
				newCount = currentCount;
			}
		}
		R_GPT3->GTCNT = newCount;

//     // restart the timer
		R_GPT3->GTCR |= 1;
	}
}

void setupPin() {
	R_PFS->PORT[1].PIN[11].PmnPFS = (1 << R_PFS_PORT_PIN_PmnPFS_PDR_Pos)
			| (1 << R_PFS_PORT_PIN_PmnPFS_PMR_Pos)
			| (3 << R_PFS_PORT_PIN_PmnPFS_PSEL_Pos);
}

void setupGPT3() {

	// enable in Master stop register
	R_MSTP->MSTPCRD &= ~(1 << R_MSTP_MSTPCRD_MSTPD6_Pos);

	// enable Write GTWP
	R_GPT3->GTWP = 0xA500;

	// set count direction GTUDDTYC
	R_GPT3->GTUDDTYC = 0x00000001;

	//Select count clock GTCR  1/64 prescaler
	R_GPT3->GTCR = 0x03000000;

	//Set Cycle GTPR
	R_GPT3->GTPR = 0xFFFF;
	R_GPT3->GTPBR = 0x3FFF;

	//Set initial value GTCNT
	R_GPT3->GTCNT = 0;

	setupPin();

	//set GTIOC pin function GTIOR
	R_GPT3->GTIOR = 0x00000009;

	//Enable GTIOC pin GTIOR
	R_GPT3->GTIOR |= 0x100;

	//Set buffer ops GTBER
	R_GPT3->GTBER = 0x100001;

	//Set compare match GTCCRA / GTCCRB
	R_GPT3->GTCCR[0] = 5;

	//Set Buffer Values GTCCRC / GTCCRE and GTCCRD / GTCCRF
	// Not applicable to our situation

	//Start count operation GTCR.CST = 1
	R_GPT3->GTCR |= 1;
}
