#include "GPT_Stepper.h"

#define PRINT_REG(BUCKET, REGISTER) \
  do { \
    uint32_t t = BUCKET->REGISTER; \
    Serial.print(#REGISTER " : 0x"); \
    Serial.println(t, HEX); \
  } while (false)


void setup() {

  pinMode(7, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("\n\nStarting R4_GPT_Test.ino\n\n");
  printRegisters();

  setupGPT3();
  printRegisters();
  delay(250);
  printRegisters();
  setSpeed(350);
  printRegisters();
}

void loop() {
  static uint8_t oldState = HIGH;
  uint8_t state = digitalRead(7);
  if (state == LOW && oldState == HIGH) {
    delay(50);
    printRegisters();
  }
  oldState = state;

  static int oldVal = 0;
  int val = analogRead(0);
  int dif = val - oldVal;
  if (abs(dif) >= 3) {
    Serial.println(val);
    oldVal = val;
    val = map(val, 0, 1023, 1, 1000);
    setSpeed(val);
  }
}



void printRegisters() {
  Serial.println("\n***Registers***\n");
  PRINT_REG(R_GPT3, GTWP);
  PRINT_REG(R_GPT3, GTPR);
  PRINT_REG(R_GPT3, GTPBR);
  PRINT_REG(R_GPT3, GTCR);
  PRINT_REG(R_GPT3, GTUDDTYC);
  PRINT_REG(R_GPT3, GTIOR);
  PRINT_REG(R_GPT3, GTCNT);

  PRINT_REG(R_GPT3, GTCCR[0]);
}
