#include <Arduino.h>
#include "ECUHardware.h"

void setup() {

  SerialUSB.begin(115200);
  ECU_TSEN_SERIAL.begin(ECU_TSEN_BAUD);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(5,OUTPUT);
  delay(500);
  digitalWrite(5,HIGH);
  delay(3000);
  SerialUSB.println("Hello World");
  delay(1000);
}

void loop() {
  static uint last_blink=0;
  if ((millis()-last_blink) > 200) {
    last_blink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  if (SerialUSB.available()) {
    char c = SerialUSB.read();
    ECU_TSEN_SERIAL.write(c);  
  }

  if (ECU_TSEN_SERIAL.available()) {
    SerialUSB.write(ECU_TSEN_SERIAL.read());
  }
}