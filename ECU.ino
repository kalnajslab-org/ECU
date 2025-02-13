#include <Arduino.h>
#include <SPI.h>
#include "ECUHardware.h"
#include "ECULoRa.h"

void setup() {
    Serial.begin(115200);

    delay(3000);

    Serial.println("Starting ECU...");
    if (!ECULoRaInit(
            LORA_LEADER, 
            1000, 
            ECU_LORA_CS, 
            ECU_LORA_RST, 
            ECU_LORA_INT, 
            &SPI, 
            ECU_LORA_SCK, 
            ECU_LORA_MISO, 
            ECU_LORA_MOSI,
            LORA_FREQ, 
            LORA_BW, 
            LORA_SF, 
            LORA_POWER)) {
        Serial.println("LoRa Initialization Failed!");
    } else {
        Serial.println("LoRa Initialized Successfully!");
    }

}

void loop() {
    static int counter = 0;
    delay(1000);                                          // Wait for a second

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