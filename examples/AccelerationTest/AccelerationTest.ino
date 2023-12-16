#include "GPT_Stepper.h"

const uint8_t buttonPin = 8;
long speed = 10;

GPT_Stepper stepper(6,7);

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("\n\nStarting AccelerationTest.ino\n\n");

  stepper.init();
  stepper.setAcceleration(100);
  stepper.setRequestedSpeed(speed);
  delay(5000);
  speed = -500;
}

void loop() {

  static uint8_t lastButtonState = HIGH;
  uint8_t buttonState = digitalRead(buttonPin);
  if(buttonState != lastButtonState){
    delay(50);
    buttonState = digitalRead(buttonPin);
    if(buttonState == LOW){
      speed = -speed;
      stepper.setRequestedSpeed(speed);
    }
    lastButtonState = buttonState;
  }  
}
