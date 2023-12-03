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

void setPeriod(uint32_t us);
void setSpeed(int stepsPerSecond);

#endif /* GPT3_STEPPER_H */