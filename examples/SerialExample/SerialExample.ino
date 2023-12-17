#include "GPT_Stepper.h"

GPT_Stepper steppers[2] = {
  GPT_Stepper(6, 7, 100.0),
  GPT_Stepper(12, 11, 100.0)
};

uint8_t numSteppers = sizeof(steppers) / sizeof(steppers[0]);
char buf[32];
uint8_t idx;
bool receiving;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\nStarting GPT_Stepper SerialExample.ino\n\n");

  for (int i = 0; i < numSteppers; i++) {
    steppers[i].init();
  }
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();

    if (c == '<') {
      idx = 0;
      receiving = true;
    }
    if (receiving) {
      buf[idx++] = c;
      buf[idx] = 0;
      if (c == '>') {
        receiving = false;
        parseCommand();
      }
    }
  }
}

void parseCommand() {
  uint8_t which = buf[2] - '0';
  switch (buf[1]) {
    case 'S':
      steppers[which].setSpeed(atof(buf + 4));
      break;
    case 'X':
      steppers[which].stop();
      break;
    case 'A':
      steppers[which].setAcceleration(atof(buf+4));
      break;
    case 'P':
      Serial.print("Stepper ");
      Serial.print(which);
      Serial.print(" Position : ");
      Serial.println(steppers[which].getPosition());
      break;
    case 'C':
      Serial.print("Stepper ");
      Serial.print(which);
      Serial.print(" Current Speed : ");
      Serial.println(steppers[which].getCurrentSpeed());
      break;
  }
}
