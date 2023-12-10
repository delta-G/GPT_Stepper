#include "GPT_Stepper.h"

const uint8_t buttonPin = 8;
uint32_t interval = 250;
int speed = 1;
int inc = 5;
int maxSpeed = 850;
int minSpeed = 1;

GPT_Stepper stepper(2,7);

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("\n\nStarting SpeedUp_SlowDown.ino\n\n");

  stepper.init();
  stepper.setSpeed(speed);
  delay(5000);
}

void loop() {

  static uint32_t pm = millis();
  uint32_t cm = millis();
  if (cm - pm >= interval) {
    pm = cm;
    if (digitalRead(buttonPin) == LOW) {
      speed = speed + inc;
      if ((speed >= maxSpeed) || (speed <= minSpeed)) {
        inc = -inc;
        speed += inc;
      }
      stepper.setSpeed(speed);
    }
  }
}
