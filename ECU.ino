#include <Arduino.h>
#include <SPI.h>
#include "ECUHardware.h"
#include "ECULoRa.h"

const int ledPin = 13; // Pin number for the LED

void setup() {
    // Initialize the digital pin as an output.
    pinMode(ledPin, OUTPUT);

    Serial.begin(115200);

    delay(3000);

    Serial.println("Starting ECU...");
    if (!ECULoRaInit(LORA_LEADER, 1000, LORA_CS, RESET_PIN, LORA_INT, &SPI, LORA_SCK, LORA_MISO, LORA_MOSI)) {
        Serial.println("LoRa Initialization Failed!");
    } else {
        Serial.println("LoRa Initialized Successfully!");
    }

}

void loop() {
    static int counter = 0;
    digitalWrite(ledPin, HIGH); // Turn the LED on (HIGH is the voltage level)
    delay(1000);                // Wait for a second
    digitalWrite(ledPin, LOW);  // Turn the LED off by making the voltage LOW
    delay(1000);                // Wait for a second
    Serial.println(String("Counter: ") + counter);
    counter++;
    ECULoRaMsg_t msg;
    if (ecu_lora_rx(&msg)) {
        String received_msg;
        for (int i = 0; i < msg.data_len; i++) {
            received_msg += (char)msg.data[i];
        }
        Serial.println(String("Received: ") + received_msg);
    }
    String data_str = "Hello from ECU!";
    if (ecu_lora_tx((uint8_t*)data_str.begin(), data_str.length())) {
        Serial.println(data_str + " transmitted successfully!");
    } else {
        Serial.println("Failed to transmit LoRa.");
    }
}