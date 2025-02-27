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
    static int missed_tsen = 0;
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

    TSEN_DATA_VECTOR tsen_data = tsen_read();
    if (tsen_data.size() == 19 && tsen_data[0] == '#') {
        Serial.print("TSEN Data: ");
        for (unsigned int i = 1; i < tsen_data.size()-1; i++) {
            Serial.print(tsen_data[i]);
        }
        Serial.println();
        // TSEN_DATA_VECTOR data format:
        // "#001 76FC44 80D4A2\r\0"
        // Extract substrings for each sensor
        TSEN_DATA_VECTOR airt_v; airt_v.assign(tsen_data.begin()+1, tsen_data.begin()+4); airt_v.push_back('\0');
        TSEN_DATA_VECTOR prest_v; prest_v.assign(tsen_data.begin()+5, tsen_data.begin()+11); prest_v.push_back('\0');
        TSEN_DATA_VECTOR pres_v; pres_v.assign(tsen_data.begin()+12, tsen_data.begin()+18); pres_v.push_back('\0');
        // Convert substrings to unsigned integers
        uint16_t airt_val = strtoul(airt_v.data(), NULL, 16);
        uint32_t prest_val = strtoul(prest_v.data(), NULL, 16);
        uint32_t pres_val = strtoul(pres_v.data(), NULL, 16);
        // Add to the ECU report
        add_tsen(airt_val, prest_val, pres_val, ecu_report);
    } else {
        missed_tsen++;
        if (missed_tsen > 5) {
            // Haven't received a TSEN message in a while, prompt the TSEN
            tsen_prompt();
            missed_tsen = 0;
        }
    }

    // RS41
    ecu_report.rs41_valid = false;
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
        add_rs41(true, sensor_data.air_temp_degC, sensor_data.humdity_percent, sensor_data.hsensor_temp_degC, sensor_data.pres_mb, sensor_data.pcb_heater_on, ecu_report);
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