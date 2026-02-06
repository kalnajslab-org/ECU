#include "ECU_Lib.h"
#include <TeensyID.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>

#define DS_RESOLUTION 12

static OneWire oneWire(DS18_TEMP);
static DallasTemperature ds18(&oneWire);

// Resistance values in kOhms
#define R5 499.0
#define R6 10.0
#define R13 30.0
#define R14 10.0
#define R15 48.7
#define R16 10.0
#define V_PER_COUNT (3.3 / 1024.0)

static uint8_t gps_serial_RX_buffer[ECU_GPS_BUFFSIZE];
static uint8_t mac_address[6];

bool initializeECU(int lora_report_interval_ms, RS41 &rs41)
{

    ECU_GPS_SERIAL.begin(ECU_GPS_BAUD);
    ECU_GPS_SERIAL.addMemoryForRead(&gps_serial_RX_buffer[0], ECU_GPS_BUFFSIZE);

    ECU_TSEN_SERIAL.begin(ECU_TSEN_BAUD);

    bool success = true;

    // Initialize the digital pins
    pinMode(V12_EN, OUTPUT);
    pinMode(SW_I_HRES_EN, OUTPUT);
    pinMode(SW_IMON_EN, OUTPUT);
    pinMode(V_ZEPHR_VMON, INPUT);
    pinMode(V12_MON, INPUT);
    pinMode(V5_MON, INPUT);
    pinMode(HEATER_DISABLE, OUTPUT);

    // Enable the current sensing
    digitalWrite(SW_IMON_EN, HIGH);
    // Set standard accuracy current sensing
    digitalWrite(SW_I_HRES_EN, (SNS_I_HRES ? HIGH : LOW));

    // Enable the 12V
    digitalWrite(V12_EN, HIGH);

    // Initialize the RS41
    rs41.init();
    Serial.println("RS41 metadata: " + rs41.meta_data());

    // Configure the DS18B20 temperature sensor
    ds18.begin();
    ds18.setResolution(DS_RESOLUTION);
    ds18.setWaitForConversion(false);

    // Start the temperature conversion
    ds18.requestTemperatures();

    // Request TSEN data
    tsen_prompt();

    // Initialize the LoRa module
    if (!ECULoRaInit(
            LORA_LEADER,
            lora_report_interval_ms,
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
            LORA_POWER))
    {
        Serial.println("LoRa Initialization Failed!");
        success = false;
    }
    else
    {
        Serial.println("LoRa Initialized Successfully!");
    }

    // Get the MAC address to use as ECU ID
    teensyMAC(mac_address);
    Serial.print("ECU MAC Address: ");
    for (int i = 0; i < 6; ++i) {
        if (i > 0) Serial.print(":");
        Serial.print(mac_address[i], HEX);
    }
    Serial.println();
    return success;
}

uint8_t ecu_id()
{
    return mac_address[5];
}

void process_lora(float& tempC_setpoint, RS41& rs41, bool& rs41_metadata_requested) {
    ECULoRaMsg_t msg;
    if (ecu_lora_rx(&msg))
    {
        if (msg.data_len > 2)
        {

            // Ignore ECUReport messages, but we are interested in
            // RATS messages which begin with <0><paired_ecu>
            if (msg.data[0] == 0)
            {
                // The paired_ecu must be zero or match our ecu_id.
                // It is contained in the second byte of the RATS message.
                uint8_t paired_ecu = msg.data[1];
                if (paired_ecu == 0 || paired_ecu == ecu_id())
                {
                    String received_msg;
                    for (int i = 2; i < msg.data_len; i++)
                    {
                        received_msg += (char)msg.data[i];
                    }
                    DeserializationError error = deserializeJson(ecu_json_doc, received_msg);
                    if (!error)
                    {
                        // Message decoded successfully
                        Serial.println(received_msg);

                        if (ecu_json_doc.containsKey("tempC")) {
                            float tempC = ecu_json_doc["tempC"] | -999.0;
                            if (tempC != -999.0)
                            {
                                tempC_setpoint = tempC;
                                Serial.println("Temp setpoint: " + String(tempC_setpoint));
                            }
                            else
                            {
                                Serial.println("Failed to decode tempC from incoming LoRa message");
                            }
                        }

                        if (ecu_json_doc.containsKey("rs41Regen")) {
                            bool regen = ecu_json_doc["rs41Regen"];
                            if (regen)
                            {
                                rs41.recondition();
                                Serial.println("RS41 regeneration started");
                            }
                        }
                        if (ecu_json_doc.containsKey("rs41Enable")) {
                            bool rs41_power = ecu_json_doc["rs41Enable"];
                            if (rs41_power)
                            {
                                digitalWrite(RS41_EN, HIGH);
                                Serial.println("RS41 powered on");
                            } else {
                                digitalWrite(RS41_EN, LOW);
                                Serial.println("RS41 powered off");
                            }
                        }
                        if (ecu_json_doc.containsKey("tsenPower")) {
                            bool tsen_power = ecu_json_doc["tsenPower"];
                            if (tsen_power)
                            {
                                digitalWrite(V12_EN, HIGH);
                                Serial.println("TSEN powered on");
                            } else {
                                digitalWrite(V12_EN, LOW);
                                Serial.println("TSEN powered off");
                            }
                        }
                        if (ecu_json_doc.containsKey("rs41Metadata")) {
                            bool rs41_metadata = ecu_json_doc["rs41Metadata"];
                            if (rs41_metadata)
                            {
                                rs41_metadata_requested = true;
                            }
                        }
                    }
                    else
                    {
                        Serial.println("Failed to decode incoming LoRa message");
                    }
                }
            }
        }
    }
}

void getBoardHealth(ECUBoardHealth_t &boardVals)
{
    static float last_temp = -1000.0;

    if (ds18.isConversionComplete())
    {
        last_temp = ds18.getTempCByIndex(0);
        ds18.requestTemperatures();
    }
    boardVals.BoardTempC = last_temp;

    boardVals.V56 = analogRead(V_ZEPHR_VMON) * (3.3 / 1024.0) * (R5 + R6) / R6;
    boardVals.V5 = analogRead(V5_MON) * (3.3 / 1024.0) * (R13 + R14) / R14;
    boardVals.V12 = analogRead(V12_MON) * (3.3 / 1024.0) * (R15 + R16) / R16;
    uint sw_imon_count = analogRead(SW_IMON);
    boardVals.ISW = 1000.0 * (((sw_imon_count * 3.3) / 1024.0) / R_SNS) * (SNS_I_HRES ? 24.0 : 800.0);
    boardVals.CpuTempC = tempmonGetTemp();
}

TSEN_DATA_VECTOR tsen_read()
{
    static TSEN_DATA_VECTOR buffer;
    static TSEN_DATA_VECTOR empty_buffer;

    while (ECU_TSEN_SERIAL.available() > 0)
    {
        char c = ECU_TSEN_SERIAL.read();
        buffer.push_back(c);

        // Check if the buffer contains the start character '#'
        if (buffer.front() != '#')
        {
            buffer.clear();
            continue;
        }

        // Check if the buffer contains the end character '\r'
        if (c == '\r')
        {
            TSEN_DATA_VECTOR result = buffer;
            buffer.clear();
            tsen_prompt();
            return result;
        }
    }
    return empty_buffer;
}

void tsen_prompt()
{
    ECU_TSEN_SERIAL.print("*01A?\r");
    ECU_TSEN_SERIAL.flush();
}

void print_tsen(TSEN_DATA_VECTOR &tsen_data)
{
    SerialUSB.print("TSEN: ");
    for (char c : tsen_data)
    {
        SerialUSB.print(c);
    }
    SerialUSB.println();
}

void manage_heater(float board_temp_C, float temp_setpoint_C)
{
    static bool heater_disabled = false;

    if (!heater_disabled && board_temp_C > temp_setpoint_C + BOARD_TEMP_HYSTERESIS_C)
    {
        digitalWrite(HEATER_DISABLE, HIGH);
        heater_disabled = true;
    }
    else if (heater_disabled && board_temp_C < temp_setpoint_C - BOARD_TEMP_HYSTERESIS_C)
    {
        digitalWrite(HEATER_DISABLE, LOW);
        heater_disabled = false;
    }
}

void print_rs41(RS41::RS41SensorData_t &sensor_data)
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
}

void print_gps(TinyGPSPlus &gps)
{
    Serial.print("Valid: ");
    Serial.print(gps.location.isValid());
    Serial.print(" Time:");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.print(gps.time.second());
    Serial.print(".");
    Serial.print(gps.time.centisecond());
    Serial.print(" Latitude:");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Longitude:");
    Serial.print(gps.location.lng(), 6);
    Serial.print(" Altitude:");
    Serial.print(gps.altitude.meters());
    Serial.print(" Speed:");
    Serial.print(gps.speed.kmph());
    Serial.print(" Course:");
    Serial.print(gps.course.deg());
    Serial.println();
}

void print_board_health(ECUBoardHealth_t &boardVals)
{
    String s;
    s = "V5:";
    s += boardVals.V5;
    s += ", ";
    s += "V12:";
    s += boardVals.V12;
    s += ", ";
    s += "Temp(C):";
    s += boardVals.BoardTempC;
    s += ", ";
    s += "V56:";
    s += boardVals.V56;
    s += ", ";
    s += "ISW:";
    s += boardVals.ISW;
    s += ", CpuTempC:";
    s += boardVals.CpuTempC;
    s += ", HeaterOn:";
    s += !digitalRead(HEATER_DISABLE);
    Serial.println(s);
}

ECUReport_t rs41_report(RS41& rs41)
{
    // Declare an ECU report for the RS41 metadata
    ECUReport_t rs41_metadata_report;

    // Fetch RS41 metadata 
    String meta_data = "RSS421 " + rs41.meta_data();

    // Prepare the ECU report for type ECU_REPORT_RAW
    rs41_metadata_report.rev = ECU_REPORT_REV;
    rs41_metadata_report.msg_type = ECU_REPORT_RAW;
    rs41_metadata_report.ecu_id = ecu_id();

    // Copy the metadata into the raw data field of the report
    rs41_metadata_report.n_bytes = min(meta_data.length(), ECU_MAX_RAW_BYTES);
    memcpy(rs41_metadata_report.raw, meta_data.c_str(), rs41_metadata_report.n_bytes);

    // Serialize and deserialize the report to verify correctness
    //auto payload = ecu_report_serialize(rs41_metadata_report);
    //auto deserialized_report = ecu_report_deserialize(payload);
    //Serial.print("Deserialized RS41 metadata: ");
    //ecu_report_print_raw(deserialized_report);
    //Serial.println();

    return rs41_metadata_report;
}