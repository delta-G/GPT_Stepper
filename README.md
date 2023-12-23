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






