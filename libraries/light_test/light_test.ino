#include "Arduino.h"
#define LIGHT_LED_PIN_NUM    PA1



void setup() {
 

  pinMode(LIGHT_LED_PIN_NUM, OUTPUT);
  digitalWrite(LIGHT_LED_PIN_NUM, HIGH);
}

void loop() {
//   put your main code here, to run repeatedly:

digitalWrite(LIGHT_LED_PIN_NUM, LOW);
// delay(1000);

// digitalWrite(LIGHT_LED_PIN_NUM, HIGH);
// delay(1000);



}
