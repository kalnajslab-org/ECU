#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "ECUHardware.h"
#include "ECUReport.h"
#include "ECU_Lib.h"

TinyGPSPlus ecu_gps;
ECUReport_t ecu_report;

void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println("Starting ECU...");
    initializeECU(1000);

    ecu_report_init(ecu_report);
}

void loop() {
    static int counter = 0;
    delay(1000);

    Serial.println("--------------------");
    Serial.println(String("Counter: ") + counter);
    counter++;
    ECULoRaMsg_t msg;
    if (ecu_lora_rx(&msg)) {
        String received_msg;
        for (int i = 0; i < msg.data_len; i++) {
            received_msg += (char)msg.data[i];
        }
        Serial.println(String("**** Received: ") + received_msg);
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

//    TSEN_DATA_VECTOR tsen_data = tsen_read();
//        Serial.println("TSEN Data:");
//        for (int i = 0; i < tsen_data.size(); i++) {
//            Serial.print(tsen_data[i]);
//        }
//        Serial.println();

    // Get the board health
    ECUBoardHealth_t boardVals;
    getBoardHealth(boardVals);

    if (boardVals.BoardTempC > 30.0) {
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
    s += boardVals.BoardTempC;
    s += ", ";
    s += "V56:";
    s += boardVals.V56;
    s += ", ";
    s += "I_SW:";
    s += boardVals.ISW;
    s += ", HeaterOn:";
    s += !digitalRead(HEATER_DISABLE);
    Serial.println(s);
    Serial.println();

    add_status(!digitalRead(HEATER_DISABLE), ecu_report);

    add_ecu_health(boardVals.V5, boardVals.V12, boardVals.V56, boardVals.BoardTempC, ecu_report);

    add_gps(ecu_gps.location.isValid(), ecu_gps.location.lat(), ecu_gps.location.lng(), ecu_gps.altitude.meters(), ecu_report);
    
    ecu_report_print(&ecu_report);
    Serial.println();

    etl::array<uint8_t, ECU_REPORT_SIZE_BYTES> data = ecu_report_serialize(ecu_report);
    if (ecu_lora_tx(data.begin(), data.size())) {
        Serial.println("Data transmitted successfully");
    } else {
        Serial.println("Failed to transmit LoRa.");
    }

}