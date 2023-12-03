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

void setupPin() {
  R_PFS->PORT[1].PIN[11].PmnPFS = (1 << R_PFS_PORT_PIN_PmnPFS_PDR_Pos) | (1 << R_PFS_PORT_PIN_PmnPFS_PMR_Pos) | (3 << R_PFS_PORT_PIN_PmnPFS_PSEL_Pos);
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
