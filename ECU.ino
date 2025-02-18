#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "ECUHardware.h"
#include "ECU_Lib.h"

TinyGPSPlus ecu_gps;

void setup() {
    Serial.begin(115200);
    delay(3000);

    ECU_GPS_SERIAL.begin(9600);
    
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
    String data_str = "ECU data at a RATS near you!";
    if (ecu_lora_tx((uint8_t*)data_str.begin(), data_str.length())) {
        Serial.println(data_str + " transmitted successfully");
    } else {
        Serial.println("Failed to transmit LoRa.");
    }

    while (ECU_GPS_SERIAL.available() > 0)
    {
        if(ecu_gps.encode(ECU_GPS_SERIAL.read())) {
            Serial.print("Valid: ");
            Serial.print(ecu_gps.location.isValid());
            Serial.print(" Time:");
            Serial.print(ecu_gps.time.hour());
            Serial.print(":");
            Serial.print(ecu_gps.time.minute());
            Serial.print(":");
            Serial.print(ecu_gps.time.second());
            Serial.print(".");
            Serial.print(ecu_gps.time.centisecond());
            Serial.print(" Latitude:");
            Serial.print(ecu_gps.location.lat(), 6);
            Serial.print(" Longitude:");
            Serial.print(ecu_gps.location.lng(), 6);
            Serial.print(" Altitude:");
            Serial.print(ecu_gps.altitude.meters());
            Serial.print(" Speed:");
            Serial.print(ecu_gps.speed.kmph());
            Serial.print(" Course:");
            Serial.print(ecu_gps.course.deg());
            Serial.println();
        }
    }

    // Get the board health
    ECUBoardHealth_t boardVals;
    getBoardHealth(boardVals);

    if (boardVals.TempC > 30.0) {
        digitalWrite(HEATER_DISABLE, HIGH);
    } else {
        digitalWrite(HEATER_DISABLE, LOW);
    }
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
    s += ", HeaterOn:";
    s += !digitalRead(HEATER_DISABLE);
    Serial.println(s);

}