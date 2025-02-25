#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "ECUHardware.h"
#include "ECUReport.h"
#include "ECU_Lib.h"
#include "RS41.h"

TinyGPSPlus ecu_gps;
ECUReport_t ecu_report;
RS41 rs41(RS41_SERIAL, RS41_EN);


void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println("Starting ECU...");
    initializeECU(1000, rs41);

    ecu_report_init(ecu_report);
}

void loop() {
    static int counter = 0;
    delay(1000);

    Serial.println("--------------------");
    Serial.println(String("Counter: ") + counter);
    counter++;

    // LoRa incoming message
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

    // RS41
    RS41::RS41SensorData_t sensor_data = rs41.decoded_sensor_data(false);
    if (sensor_data.valid)
    {
        SerialUSB.print(sensor_data.frame_count);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.air_temp_degC);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.humdity_percent);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.hsensor_temp_degC);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.pres_mb);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.internal_temp_degC);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.module_status);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.module_error);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.pcb_supply_V);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.lsm303_temp_degC);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.pcb_heater_on);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.mag_hdgXY_deg);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.mag_hdgXZ_deg);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.mag_hdgYZ_deg);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.accelX_mG);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.accelY_mG);
        SerialUSB.print(",");
        SerialUSB.print(sensor_data.accelZ_mG);
        SerialUSB.println();
        add_rs41(sensor_data.air_temp_degC, sensor_data.humdity_percent, sensor_data.hsensor_temp_degC, sensor_data.pres_mb, sensor_data.pcb_heater_on, ecu_report);
    }
    else
    {
        SerialUSB.println("Unable to obtain RS41 sensor data");
    }

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

    add_gps(
        ecu_gps.location.isValid(),
        ecu_gps.location.lat(),
        ecu_gps.location.lng(),
        ecu_gps.altitude.meters(),
        ecu_gps.satellites.value(),
        ecu_gps.hdop.hdop(),
        ecu_gps.location.age() / 1000,
        ecu_report
    );
    
    ECUReportBytes_t payload = ecu_report_serialize(ecu_report);
    if (!ecu_lora_tx(payload.begin(), payload.size())) {
        Serial.println("Failed to transmit LoRa.");
    }

    ECUReport_t ecu_report_sent = ecu_report_deserialize(payload);
    ecu_report_print(ecu_report_sent, true);
    Serial.println();
}