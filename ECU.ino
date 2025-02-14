#include <Arduino.h>
#include "ECU_Lib.h"

void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println("Starting ECU...");
    initializeECU(1000);

}

void loop() {
    static int counter = 0;
    delay(1000);

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

    // Toggle the 12V
    enable12V((counter % 10) < 5);

    // Get the board health
    ECUBoardHealth_t boardVals;
    getBoardHealth(boardVals);

    String s;
    s = "V5:";
    s += boardVals.V5;
    s += ", ";
    //s += "12V_I:";
    //s += analogRead(pin12V_IMON);
    //s += ", ";
    s += "V12:";
    s += boardVals.V12;
    s += ", ";
    s += "Temp(C):";
    s += boardVals.TempC;
    s += ", ";
    s += "V56:";
    s += boardVals.V56;
    s += ", ";
    s += "I_SW:";
    s += boardVals.ISW;
    Serial.println(s);

}