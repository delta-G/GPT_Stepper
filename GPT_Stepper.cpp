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

bool GPT_Stepper::timerClaimed[8];
GPT_Stepper *GPT_Stepper::isrRegistry[8];



/**********************************************************************
 * 
 *   Public Member Functions
 * 
 *********************************************************************/

bool GPT_Stepper::init() {

	bool rv = false;

	switch (stepPin) {
	case 0:
		if (!timerClaimed[4]) {
			timerClaimed[4] = true;
			timer = R_GPT4;
			channel = CHANNEL_B;
			setupStepPin(3, 1);
			setupInterrupt(4, GPT4_ISR);
			isrRegistry[4] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 4);
		}
		break;
	case 1:
		if (!timerClaimed[4]) {
			timerClaimed[4] = true;
			timer = R_GPT4;
			channel = CHANNEL_A;
			setupStepPin(3, 2);
			setupInterrupt(4, GPT4_ISR);
			isrRegistry[4] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 4);
		}
		break;
	case 2:
		if (!timerClaimed[1]) {
			timerClaimed[1] = true;
			timer = R_GPT1;
			channel = CHANNEL_B;
			setupStepPin(1, 4);
			setupInterrupt(1, GPT1_ISR);
			isrRegistry[1] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 1);
		}
		break;
	case 3:
		if (!timerClaimed[1]) {
			timerClaimed[1] = true;
			timer = R_GPT1;
			channel = CHANNEL_A;
			setupStepPin(1, 5);
			setupInterrupt(1, GPT1_ISR);
			isrRegistry[1] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 1);
		}
		break;
	case 4:
		if (!timerClaimed[0]) {
			timerClaimed[0] = true;
			timer = R_GPT0;
			channel = CHANNEL_B;
			setupStepPin(1, 6);
			setupInterrupt(0, GPT0_ISR);
			isrRegistry[0] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 0);
		}
		break;
	case 5:
		if (!timerClaimed[0]) {
			timerClaimed[0] = true;
			timer = R_GPT0;
			channel = CHANNEL_A;
			setupStepPin(1, 7);
			setupInterrupt(0, GPT0_ISR);
			isrRegistry[0] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 0);
		}
		break;
	case 6:
		if (!timerClaimed[3]) {
			timerClaimed[3] = true;
			timer = R_GPT3;
			channel = CHANNEL_A;
			setupStepPin(1, 11);
			setupInterrupt(3, GPT3_ISR);
			isrRegistry[3] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 3);
		}
		break;
	case 7:
		if (!timerClaimed[3]) {
			timerClaimed[3] = true;
			timer = R_GPT3;
			channel = CHANNEL_B;
			setupStepPin(1, 12);
			setupInterrupt(3, GPT3_ISR);
			isrRegistry[3] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 3);
		}
		break;
	case 8:
		if (!timerClaimed[7]) {
			timerClaimed[7] = true;
			timer = R_GPT7;
			channel = CHANNEL_A;
			setupStepPin(3, 4);
			setupInterrupt(7, GPT7_ISR);
			isrRegistry[7] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 7);
		}
		break;
	case 9:
		if (!timerClaimed[7]) {
			timerClaimed[7] = true;
			timer = R_GPT7;
			channel = CHANNEL_B;
			setupStepPin(3, 3);
			setupInterrupt(7, GPT7_ISR);
			isrRegistry[7] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 7);
		}
		break;
	case 10:
		if (!timerClaimed[2]) {
			timerClaimed[2] = true;
			timer = R_GPT2;
			channel = CHANNEL_A;
			setupStepPin(1, 3);
			setupInterrupt(2, GPT2_ISR);
			isrRegistry[2] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 2);
		}
		break;
	case 11:
		if (!timerClaimed[6]) {
			timerClaimed[6] = true;
			timer = R_GPT6;
			channel = CHANNEL_A;
			setupStepPin(4, 11);
			setupInterrupt(6, GPT6_ISR);
			isrRegistry[6] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 6);
		}
		break;
	case 12:
		if (!timerClaimed[6]) {
			timerClaimed[6] = true;
			timer = R_GPT6;
			channel = CHANNEL_B;
			setupStepPin(4, 10);
			setupInterrupt(6, GPT6_ISR);
			isrRegistry[6] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 6);
		}
		break;
	case 13:
		if (!timerClaimed[2]) {
			timerClaimed[2] = true;
			timer = R_GPT2;
			channel = CHANNEL_B;
			setupStepPin(1, 2);
			setupInterrupt(2, GPT2_ISR);
			isrRegistry[2] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 2);
		}
		break;
	case A4:
		if (!timerClaimed[5]) {
			timerClaimed[5] = true;
			timer = R_GPT5;
			channel = CHANNEL_A;
			setupStepPin(1, 1);
			setupInterrupt(5, GPT5_ISR);
			isrRegistry[5] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 5);
		}
		break;
	case A5:
		if (!timerClaimed[5]) {
			timerClaimed[5] = true;
			timer = R_GPT5;
			channel = CHANNEL_B;
			setupStepPin(1, 0);
			setupInterrupt(5, GPT5_ISR);
			isrRegistry[5] = this;
			FspTimer::set_timer_is_used(GPT_TIMER, 5);
		}
		break;

	}
	if (timer) {
		setupTimer();
		rv = true;
	}
	pinMode(directionPin, OUTPUT);
	digitalWrite(directionPin, LOW);
	return rv;
}

void GPT_Stepper::setAcceleration(float stepsPerSecondPerSecond) {
	if (stepsPerSecondPerSecond >= 0) {
		noInterrupts();
		acceleration = stepsPerSecondPerSecond;
		interrupts();
	}
}

/// TODO:  This should check against the 32bit resolution of the timer
////        and actually set the closest achievable value and return it
void GPT_Stepper::setSpeed(float stepsPerSecond) {
	noInterrupts();
	requestedSpeed = invert? stepsPerSecond : -stepsPerSecond;
	if(speed < 50){
		stopTimer(); // So we don't have to wait forever to update the speed
	}
	interrupts();
	if (!timerRunning()) {
		setCurrentSpeed(getNewSpeed());
	}
}	

void GPT_Stepper::stop() {
	// stop the timer:
	requestedSpeed = 0;
	setCurrentSpeed(0);
}



long GPT_Stepper::getPosition() {
	noInterrupts();
	long rv = position;
	interrupts();
	return rv;
}

void GPT_Stepper::setHome() {
	noInterrupts();
	position = 0;
	interrupts();
}

float GPT_Stepper::getCurrentSpeed() {
	noInterrupts();
	float rv = speed;
	interrupts();
	return rv;
}

float GPT_Stepper::getAcceleration() {
	return acceleration;
}

bool GPT_Stepper::atSpeed() {
	noInterrupts();
	bool rv = (speed == requestedSpeed);
	interrupts();
	return rv;
}

/**********************************************************************
 * 
 *   Private Member Functions
 * 
 *********************************************************************/

void GPT_Stepper::setupStepPin(uint8_t port, uint8_t pin) {
	R_PFS->PORT[port].PIN[pin].PmnPFS = (1 << R_PFS_PORT_PIN_PmnPFS_PDR_Pos)
			| (1 << R_PFS_PORT_PIN_PmnPFS_PMR_Pos)
			| (3 << R_PFS_PORT_PIN_PmnPFS_PSEL_Pos);
}

void GPT_Stepper::setupTimer() {

	// enable in Master stop register in case they're not already
	R_MSTP->MSTPCRD &= ~(1 << R_MSTP_MSTPCRD_MSTPD6_Pos);  // 16bit GPT
	R_MSTP->MSTPCRD &= ~(1 << R_MSTP_MSTPCRD_MSTPD5_Pos);  // 32bit GPT

	// enable Write GTWP
	timer->GTWP = 0xA500;

	// set count direction GTUDDTYC
	timer->GTUDDTYC = 0x00000001;

	//Select count clock GTCR  1/64 prescaler
	timer->GTCR = 0x03000000;

	//Set Cycle GTPR
	timer->GTPR = 0xFFFF;
	timer->GTPBR = 0x3FFF;

	//Set initial value GTCNT
	timer->GTCNT = 0;

	//set GTIOC pin function GTIOR
	//0x16 is Initial state = HIGH, LOW at cycle end, HIGH at compare match.
	if (channel == CHANNEL_A) {
		timer->GTIOR = 0x00000016UL;
		timer->GTIOR |= 0x100;  // enable the pin
	} else if (channel == CHANNEL_B) {
		timer->GTIOR = 0x00160000UL;
		timer->GTIOR |= 0x1000000;  // enable the pin
	}

	//Set buffer ops GTBER
	timer->GTBER = 0x100001;

	//Set compare match GTCCRA / GTCCRB
	timer->GTCCR[channel] = 5;

	//Set Buffer Values GTCCRC / GTCCRE and GTCCRD / GTCCRF
	// Not applicable to our situation

	//Start count operation GTCR.CST = 1
	// Handled by setPeriod when the stepper is started
//	timer->GTCR |= 1;
}

void GPT_Stepper::setupInterrupt(uint8_t ch, void (*isr)()) {
	timer_cfg_t base;
	base.channel = ch;
	base.cycle_end_irq = FSP_INVALID_VECTOR;
	TimerIrqCfg_t iCfg;
	iCfg.base_cfg = &base;
	gpt_extended_cfg_t fake;
	iCfg.gpt_ext_cfg = &fake;
	iCfg.agt_ext_cfg = nullptr;

	__disable_irq();
	if (IRQManager::getInstance().addTimerOverflow(iCfg, isr)) {
		// Serial.println("Attached Interrupt.");
		eventLinkIndex = iCfg.base_cfg->cycle_end_irq;
		R_BSP_IrqDisable((IRQn_Type) eventLinkIndex);
		R_BSP_IrqStatusClear((IRQn_Type) eventLinkIndex);
		NVIC_SetPriority((IRQn_Type) eventLinkIndex, 14);
		R_BSP_IrqEnable((IRQn_Type) eventLinkIndex);
	}
	timer->GTST &= ~(1 << R_GPT0_GTST_TCFPO_Pos);
	__enable_irq();
}

void GPT_Stepper::setCurrentSpeed(float stepsPerSecond) {
	if (!timer) {
		return;
	}
	speed = stepsPerSecond;
	if(abs(speed) < 0.1){
		speed = 0;
	}
	if (speed == 0) {
		stopTimer();
	} else {
		if (speed < 0) {
			setDirection (D_CCW);
		} else {
			setDirection (D_CW);
		}
		uint32_t us = 1000000UL / (speed > 0 ? speed : -speed);
		setPeriod(us);
	}
}

void GPT_Stepper::setPeriod(uint32_t us) {
	if (!timer) {
		return;
	}
	// PCLKD is running at full system speed, 48MHz.  That's 48 ticks per microsecond.
	uint32_t ticks = (clock_uHz * us);
	uint32_t timerResolution = getTimerResolution(); // We'll need this when we support 32 bit timers
	uint32_t resetCount = 0xFFFF;
	// Find smallest divider that works.  1, 4, 16, 64, 256, 1024
	uint8_t div = 0;
	for (div = 0; div <= 5; div++) {
		if (ticks < (timerResolution - 2000)) {
			resetCount = ticks;
			break;
		}
		ticks /= 4;
	}
	uint8_t currentDiv = ((timer->GTCR >> (R_GPT0_GTCR_TPCS_Pos + 1)) & 0x07);
	if (currentDiv == div && timerRunning()) {
		// if the prescaler still works and the timer is running 
		// then we can just set the reset value for the next count
		timer->GTPBR = resetCount;
	} else {
//     // stops counter and sets prescaler
		stopTimer();
		timer->GTCR = (div << (R_GPT0_GTCR_TPCS_Pos + 1)); // R_GPT0_GTCR_TPCS_Pos is wrong.
		timer->GTPR = resetCount;
		timer->GTPBR = resetCount;
		// reset the compare match for the new prescaler
		uint32_t hiTicks = 48ul * 3;    // 3 microsecond pulse
		if (div) {
			hiTicks /= (1 << (div * 2));  // Scale to the prescaler.
		}
		if (hiTicks < 2) {
			hiTicks = 2;   // minimum pulse
		}
		timer->GTCCR[channel] = hiTicks;

		// figure out our current place in the count with the new divider
		uint32_t currentCount = timer->GTCNT;
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
		timer->GTCNT = newCount;
		// restart the timer 
		startTimer();
	}
}

float GPT_Stepper::getNewSpeed() {
	float rv = speed;
	if (speed < requestedSpeed) {
		if (speed == 0) {
			rv = sqrt(2.0 * acceleration);
		} else {
			rv = speed + (abs(acceleration / speed));
		}
		if (rv > requestedSpeed) {
			rv = requestedSpeed;
		}
	} else if (speed > requestedSpeed) {
		if (speed == 0) {
			rv = -(sqrt(2.0 * acceleration));
		} else {
			rv = speed - (abs(acceleration / speed));
		}
		if (rv < requestedSpeed) {
			rv = requestedSpeed;
		}
	}
	return rv;
}

void GPT_Stepper::setDirection(Direction_t direction) {
	if (direction == D_CCW) {
		digitalWrite(directionPin, LOW);
	} else {
		digitalWrite(directionPin, HIGH);
	}
}

void GPT_Stepper::stopTimer() {
	if (timer)
		timer->GTCR &= ~1;
}

void GPT_Stepper::startTimer() {
	if (timer)
		timer->GTCR |= 1;
}

bool GPT_Stepper::timerRunning() {
	if (timer) {
		return (timer->GTCR & 1);
	}
	return false;
}

uint16_t GPT_Stepper::getDivider() {
	return 1 << (((timer->GTCR >> (R_GPT0_GTCR_TPCS_Pos + 1)) & 0x07) * 2);
}

uint32_t GPT_Stepper::getTimerResolution() {
	return 0xFFFF;  // Not supporting 32 bit yet
}

void GPT_Stepper::internalISR() {
	if (speed < 0) {
		position -= 1;
	} else {
		position += 1;
	}
	setCurrentSpeed(getNewSpeed());
}

/**********************************************************************
 * 
 *   ISR Functions
 * 
 *********************************************************************/

void GPT0_ISR() {
	R_GPT0->GTST &= ~(1 << R_GPT0_GTST_TCFPO_Pos);
	R_ICU->IELSR[GPT_Stepper::isrRegistry[0]->eventLinkIndex] &=
			~(R_ICU_IELSR_IR_Msk);
	GPT_Stepper::isrRegistry[0]->internalISR();
}
void GPT1_ISR() {
	R_GPT1->GTST &= ~(1 << R_GPT0_GTST_TCFPO_Pos);
	R_ICU->IELSR[GPT_Stepper::isrRegistry[1]->eventLinkIndex] &=
			~(R_ICU_IELSR_IR_Msk);
	GPT_Stepper::isrRegistry[1]->internalISR();
}
void GPT2_ISR() {
	R_GPT2->GTST &= ~(1 << R_GPT0_GTST_TCFPO_Pos);
	R_ICU->IELSR[GPT_Stepper::isrRegistry[2]->eventLinkIndex] &=
			~(R_ICU_IELSR_IR_Msk);
	GPT_Stepper::isrRegistry[2]->internalISR();
}
void GPT3_ISR() {
	R_GPT3->GTST &= ~(1 << R_GPT0_GTST_TCFPO_Pos);
	R_ICU->IELSR[GPT_Stepper::isrRegistry[3]->eventLinkIndex] &=
			~(R_ICU_IELSR_IR_Msk);
	GPT_Stepper::isrRegistry[3]->internalISR();
}
void GPT4_ISR() {
	R_GPT4->GTST &= ~(1 << R_GPT0_GTST_TCFPO_Pos);
	R_ICU->IELSR[GPT_Stepper::isrRegistry[4]->eventLinkIndex] &=
			~(R_ICU_IELSR_IR_Msk);
	GPT_Stepper::isrRegistry[4]->internalISR();
}
void GPT5_ISR() {
	R_GPT5->GTST &= ~(1 << R_GPT0_GTST_TCFPO_Pos);
	R_ICU->IELSR[GPT_Stepper::isrRegistry[5]->eventLinkIndex] &=
			~(R_ICU_IELSR_IR_Msk);
	GPT_Stepper::isrRegistry[5]->internalISR();
}
void GPT6_ISR() {
	R_GPT6->GTST &= ~(1 << R_GPT0_GTST_TCFPO_Pos);
	R_ICU->IELSR[GPT_Stepper::isrRegistry[6]->eventLinkIndex] &=
			~(R_ICU_IELSR_IR_Msk);
	GPT_Stepper::isrRegistry[6]->internalISR();
}
void GPT7_ISR() {
	R_GPT7->GTST &= ~(1 << R_GPT0_GTST_TCFPO_Pos);
	R_ICU->IELSR[GPT_Stepper::isrRegistry[7]->eventLinkIndex] &=
			~(R_ICU_IELSR_IR_Msk);
	GPT_Stepper::isrRegistry[7]->internalISR();
}
