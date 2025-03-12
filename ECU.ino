#include <Arduino.h>
#include <ArduinoJson.h>
#include "ECUHardware.h"
#include "ECUReport.h"
#include "ECU_Lib.h"
#include "RS41.h"

TinyGPSPlus ecu_gps;
ECUReport_t ecu_report;
RS41 rs41(RS41_SERIAL, RS41_EN);
JsonDocument ecu_json_doc;

float temp_setpoint = 0;

void setup()
{
    Serial.begin(115200);
    delay(3000);
    Serial.println("Build date: " __DATE__ " " __TIME__);
    Serial.println("Starting ECU...");
    initializeECU(1000, rs41);
}

void loop()
{
    static int counter = 0;
    static int missed_tsen = 0;
    delay(971);

    // Initialize the ECU report
    ecu_report_init(ecu_report);

    counter++;

    // LoRa incoming message
    ECULoRaMsg_t msg;
    if (ecu_lora_rx(&msg))
    {
        String received_msg;
        for (int i = 0; i < msg.data_len; i++)
        {
            received_msg += (char)msg.data[i];
        }
        DeserializationError error = deserializeJson(ecu_json_doc, received_msg);
        if (!error)
        {
            // Message decoded successfully
            Serial.println(received_msg);
            float tempC = ecu_json_doc["tempC"] | -999.0;
            if (tempC != -999.0)
            {
                temp_setpoint = tempC;
                Serial.println("Temp setpoint: " + String(temp_setpoint));
            } else {
                Serial.println("Failed to decode tempC from incoming LoRa message");
            }
        } else {
            Serial.println("Failed to deserialize incoming LoRa message");
        }
    }

    // GPS
    while (ECU_GPS_SERIAL.available() > 0)
    {
        if (ecu_gps.encode(ECU_GPS_SERIAL.read()))
        {
            // print_gps(ecu_gps);
            add_gps(
                ecu_gps.location.isValid(),
                ecu_gps.location.lat(),
                ecu_gps.location.lng(),
                ecu_gps.altitude.meters(),
                ecu_gps.satellites.value(),
                ecu_gps.date.value(),
                ecu_gps.time.value(),
                ecu_gps.location.age() / 1000,
                ecu_report);
        }
    }

    // TSEN
    TSEN_DATA_VECTOR tsen_data = tsen_read();
    if (tsen_data.size() == 19 && tsen_data[0] == '#')
    {
        // print_tsen(tsen_data);

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
    }
    else
    {
        missed_tsen++;
        if (missed_tsen > 5)
        {
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
        // print_rs41(sensor_data);
        add_rs41(
            true,
            sensor_data.air_temp_degC,
            sensor_data.humdity_percent,
            sensor_data.hsensor_temp_degC,
            sensor_data.pres_mb,
            sensor_data.pcb_heater_on,
            ecu_report
        );
    }
    else
    {
        SerialUSB.println("Unable to obtain RS41 sensor data");
    }

    // Board health
    ECUBoardHealth_t boardVals;
    getBoardHealth(boardVals);
    // print_board_health(boardVals);
    if (boardVals.BoardTempC > temp_setpoint)
    {
        digitalWrite(HEATER_DISABLE, HIGH);
    }
    else
    {
        digitalWrite(HEATER_DISABLE, LOW);
    }
    

    add_status(!digitalRead(HEATER_DISABLE), ecu_report);
    add_ecu_health(
        boardVals.V5,
        boardVals.V12,
        boardVals.V56,
        boardVals.BoardTempC,
        boardVals.ISW,
        ecu_report
    );

    // Serialize and transmit the ECU report
    ECUReportBytes_t payload = ecu_report_serialize(ecu_report);
    if (!ecu_lora_tx(payload.begin(), payload.size()))
    {
        Serial.println("Failed to transmit LoRa.");
    }

    // Deserialize and print the ECU report
    if (!(counter % 10))
    {
        ECUReport_t ecu_report_sent = ecu_report_deserialize(payload);
        ecu_report_print(ecu_report_sent, true);
        Serial.println("--------------------");
    }
}