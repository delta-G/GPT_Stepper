# GPT_Timer

This library leverages the GPT timers on the digital pins of the Arduino UNO-R4 to drive a simple step pulse.  The library is for use with a stepper driver that takes a direction and step input.  

Each stepper consumes a GPT timer.  Since each timer is shared by two pins, this will disable PWM on one other pin besides the one being used for the stepper.  Additionally, two steppers cannot have step pins that share a timer.  Below is a list of the timers on the UNO-R4 Wifi and the associated pins.  You can only use one pin out of each row for a stepper for a total of 7 possible steppers.  
<br>
<br>

| **Timer** | **A**  | **B**  |
|-----------|--------|--------|
|  GPT-0    |      5 |      4 |
|  GPT-1    |      3 |      2 |
|  GPT-2    |      10|      13|
|  GPT-3    |      6 |      7 |
|  GPT-4    |      1 |      0 |
|  GPT-5    |      A4|      A5|
|  GPT-6    |      11|      12|
|  GPT-7    |      8 |      9 |



# Constructor:
There are three versions.  The first needs just the step pin and direction pin.  The second adds an acceleration value, and the third also adds a boolean to invert the directional control.  
```
    GPT_Stepper(uint8_t spin, uint8_t dpin) :
			GPT_Stepper(spin, dpin, 100.0) {
	}
	GPT_Stepper(uint8_t spin, uint8_t dpin, float acc) :
			GPT_Stepper(spin, dpin, acc, false) {
	}
	GPT_Stepper(uint8_t spin, uint8_t dpin, float acc, bool inv) :
			directionPin(dpin), stepPin(spin), acceleration(acc), invert(inv) {
	}
```

The following functions are also available:

`bool init();`  Call from setup to start the timer and initialize the pin

`void setAcceleration(float stepsPerSecondPerSecond);`  Sets acceleration value

`void setSpeed(float stepsPerSecond);`  Sets the requested speed.  The motor will accelerate to this speed according to the set acceleration value. 

`void stop();`  Immediately stops the pulses to the motor. 

`long getPosition();`  Get current motor position

`void setHome();`  Zeros the current motor position. 

`float getCurrentSpeed();`  Get the current speed of the motor in steps/second

`float getAcceleration();`  Get the current acceleration value in steps/second/second

`bool atSpeed();`  Returns true if the motor is at the requested speed and is no longer accelerating, otherwise returns false. 





# Changelog

v0.2.0 - changing speeds at less than 50 steps per second no longer uses acceleration
v0.3.0 - Informs FspTimer.h that it is claiming the timer so FspTimer doesn't try to reallocate it
v0.4.0 - Fixes Bug Inverted motor reporting inverted speed. 


