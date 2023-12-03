#include "GPT_Stepper.h"

const uint8_t buttonPin = 7;
uint32_t interval = 250;
int speed = 1;
int inc = 5;
int maxSpeed = 850;
int minSpeed = 1;



void setup() {
  pinMode(7, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("\n\nStarting SpeedUp_SlowDown.ino\n\n");

  setupGPT3();
  setSpeed(speed);
  delay(5000);
}

void loop() {

  static uint32_t pm = millis();
  uint32_t cm = millis();
  if(cm - pm >= interval){
    pm = cm;
    speed = speed + inc;
    if((speed >= maxSpeed)||(speed <= minSpeed)){
      inc = -inc;
      speed += inc;
    }
    setSpeed(speed);
  }


}

