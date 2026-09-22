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

// u-blox GPS dynamic platform model values (UBX-CFG-NAV5 dynModel field).
#define GPS_DYNMODEL_AIRBORNE_1G 6

// Reads one incoming UBX frame from gpsSerial, scanning past any interleaved
// NMEA/other bytes and validating the frame's checksum. On success, fills
// out_class/out_id/out_payload/out_payload_len and returns true. A payload
// longer than max_payload_len is rejected (skipped, scanning continues).
// Blocks the caller for up to timeout_ms waiting for a complete, valid frame.
static bool read_ubx_frame(Stream &gpsSerial, uint8_t &out_class, uint8_t &out_id,
                            uint8_t *out_payload, uint16_t max_payload_len,
                            uint16_t &out_payload_len, uint32_t timeout_ms)
{
    elapsedMillis elapsed;
    enum { SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B } state = SYNC1;
    uint16_t len = 0, idx = 0;
    uint8_t cls = 0, id = 0;
    uint8_t ck_a = 0, ck_b = 0;

    while (elapsed < timeout_ms) {
        if (!gpsSerial.available()) continue;
        uint8_t b = gpsSerial.read();

        switch (state) {
        case SYNC1:
            state = (b == 0xB5) ? SYNC2 : SYNC1;
            break;
        case SYNC2:
            state = (b == 0x62) ? CLASS : SYNC1;
            ck_a = ck_b = 0;
            break;
        case CLASS:
            cls = b; ck_a += b; ck_b += ck_a; state = ID;
            break;
        case ID:
            id = b; ck_a += b; ck_b += ck_a; state = LEN1;
            break;
        case LEN1:
            len = b; ck_a += b; ck_b += ck_a; state = LEN2;
            break;
        case LEN2:
            len |= (uint16_t)b << 8; ck_a += b; ck_b += ck_a;
            idx = 0;
            if (len > max_payload_len) { state = SYNC1; break; } // too big; resync
            state = (len == 0) ? CK_A : PAYLOAD;
            break;
        case PAYLOAD:
            out_payload[idx++] = b; ck_a += b; ck_b += ck_a;
            if (idx == len) state = CK_A;
            break;
        case CK_A:
            state = (b == ck_a) ? CK_B : SYNC1;
            break;
        case CK_B:
            if (b == ck_b) {
                out_class = cls;
                out_id = id;
                out_payload_len = len;
                return true;
            }
            state = SYNC1;
            break;
        }
    }
    return false;
}

// Waits up to timeout_ms for a UBX-ACK-ACK/ACK-NAK (class 0x05, id 0x01/0x00)
// responding to a previously-sent UBX message of the given class/id, and
// prints the result to the console.
static void wait_for_gps_config_ack(Stream &gpsSerial, uint8_t msg_class, uint8_t msg_id, uint32_t timeout_ms)
{
    elapsedMillis elapsed;
    while (elapsed < timeout_ms) {
        uint8_t cls, id, payload[2];
        uint16_t payload_len;
        if (!read_ubx_frame(gpsSerial, cls, id, payload, sizeof(payload), payload_len,
                             timeout_ms - elapsed)) {
            break;
        }
        if (cls != 0x05 || payload_len != 2 || payload[0] != msg_class || payload[1] != msg_id) {
            continue; // not the ack we're waiting for; keep scanning
        }
        bool is_ack = (id == 0x01);
        Serial.println(is_ack
            ? "GPS CFG-NAV5 ACKed by receiver"
            : "GPS CFG-NAV5 NAKed by receiver -- airborne mode NOT applied");
        return;
    }
    Serial.println("GPS CFG-NAV5 ack timed out -- no response from receiver");
}

// Sends a UBX-CFG-NAV5 poll (empty-body request) and waits up to timeout_ms
// for the receiver's reply, which carries its current NAV5 configuration.
// Prints whether the live dynModel matches expected_dyn_model -- this is the
// actual applied setting, not just an acknowledgement that a prior write was
// accepted.
static void verify_gps_dynmodel(Stream &gpsSerial, uint8_t expected_dyn_model, uint32_t timeout_ms)
{
    uint8_t poll[8] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0, 0};
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; i++) {
        ck_a += poll[i];
        ck_b += ck_a;
    }
    poll[6] = ck_a;
    poll[7] = ck_b;
    gpsSerial.write(poll, sizeof(poll));

    elapsedMillis elapsed;
    while (elapsed < timeout_ms) {
        uint8_t cls, id, payload[36];
        uint16_t payload_len;
        if (!read_ubx_frame(gpsSerial, cls, id, payload, sizeof(payload), payload_len,
                             timeout_ms - elapsed)) {
            break;
        }
        if (cls != 0x06 || id != 0x24 || payload_len != 36) {
            continue; // not the CFG-NAV5 reply; keep scanning
        }
        uint8_t actual_dyn_model = payload[2]; // dynModel is payload offset 2
        if (actual_dyn_model == expected_dyn_model) {
            Serial.printf("GPS dynamic platform model verified: %u\n", actual_dyn_model);
        } else {
            Serial.printf("GPS dynamic platform model MISMATCH: expected %u, receiver reports %u\n",
                          expected_dyn_model, actual_dyn_model);
        }
        return;
    }
    Serial.println("GPS CFG-NAV5 poll timed out -- could not verify dynamic platform model");
}

// Sets the u-blox receiver's dynamic platform model to "Airborne <1g" by
// sending a UBX-CFG-NAV5 message. The receiver's default "Portable" model
// stops reporting valid fixes around 12 km altitude, well below stratospheric
// balloon float altitudes, so this must be set for the GPS to work in flight.
//
// This is sent to RAM only (no UBX-CFG-CFG save to flash) and re-sent on
// every boot: it's a small, fast, idempotent write, and avoids wearing out
// the receiver's flash with a save on every power-up.
static void configure_gps_airborne_mode(Stream &gpsSerial)
{
    // UBX-CFG-NAV5 (class 0x06, id 0x24), 36-byte payload. mask = 0x0001
    // selects only the dynModel field; all other payload fields are ignored
    // by the receiver and left zeroed.
    uint8_t msg[44] = {
        0xB5, 0x62,                     // sync chars
        0x06, 0x24,                     // class, id
        0x24, 0x00,                     // payload length = 36
        0x01, 0x00,                     // mask = dynModel only
        GPS_DYNMODEL_AIRBORNE_1G,       // dynModel
        // remaining payload bytes left at 0
    };

    // UBX checksum (8-bit Fletcher) over class, id, length, and payload.
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 42; i++) {
        ck_a += msg[i];
        ck_b += ck_a;
    }
    msg[42] = ck_a;
    msg[43] = ck_b;

    gpsSerial.write(msg, sizeof(msg));
    wait_for_gps_config_ack(gpsSerial, msg[2], msg[3], 1000);
    verify_gps_dynmodel(gpsSerial, GPS_DYNMODEL_AIRBORNE_1G, 1000);
}

bool initializeECU(int lora_report_interval_ms, RS41 &rs41)
{

    ECU_GPS_SERIAL.begin(ECU_GPS_BAUD);
    ECU_GPS_SERIAL.addMemoryForRead(&gps_serial_RX_buffer[0], ECU_GPS_BUFFSIZE);
    configure_gps_airborne_mode(ECU_GPS_SERIAL);

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
        ECULoRaConfig_t cfg = ecu_lora_get_config();
        Serial.print("  Freq: "); Serial.print(cfg.frequency); Serial.println(" Hz");
        Serial.print("  BW: "); Serial.print(cfg.bandwidth); Serial.println(" Hz");
        Serial.print("  SF: "); Serial.println(cfg.sf);
        Serial.print("  Power: "); Serial.println(cfg.power);
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

void update_lora_tx_suspend(LoraTxSuspend_t& suspend, bool gps_valid)
{
    bool duration_elapsed = suspend.timer >= suspend.duration_ms;
    bool gps_cleared = suspend.gps_override_enabled && gps_valid;
    if (suspend.active && (duration_elapsed || gps_cleared))
    {
        suspend.active = false;
    }
}

static bool RTCSet      = false;
static bool RTCSetByGPS = false;

bool isRTCSet()
{
    return RTCSet;
}

bool isRTCSetByGPS()
{
    return RTCSetByGPS;
}

void setRTCSetManually()
{
    RTCSet = true;
}

void update_rtc_from_gps(TinyGPSPlus& gps)
{
    // Gated on location validity (a real fix), not just date/time validity:
    // some GPS chips mark date/time "valid" (parseable, checksum-good) as
    // soon as they decode any satellite subframe, sometimes with a
    // placeholder/reset-default time, well before ever achieving a fix.
    // This matches the report's own gps_valid semantics (location-based)
    // and RPU's reference implementation.
    if (gps.location.isValid())
    {
        struct tm t = {};
        t.tm_year = gps.date.year() - 1900;
        t.tm_mon  = gps.date.month() - 1;
        t.tm_mday = gps.date.day();
        t.tm_hour = gps.time.hour();
        t.tm_min  = gps.time.minute();
        t.tm_sec  = gps.time.second();
        Teensy3Clock.set(mktime(&t));
        RTCSet      = true;
        RTCSetByGPS = true;
    }
}

void get_rtc_date_time(uint32_t& date, uint32_t& time)
{
    time_t rtc_epoch = Teensy3Clock.get();
    struct tm* t = gmtime(&rtc_epoch);
    uint32_t yy = (uint32_t)(t->tm_year + 1900) % 100;
    date = (uint32_t)t->tm_mday * 10000 + (uint32_t)(t->tm_mon + 1) * 100 + yy;
    time = (uint32_t)t->tm_hour * 1000000 + (uint32_t)t->tm_min * 10000 + (uint32_t)t->tm_sec * 100;
}

void process_lora(float& tempC_setpoint, RS41& rs41, bool& rs41_metadata_requested,
                   LoraTxSuspend_t& lora_tx_suspend) {
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
                        if (ecu_json_doc.containsKey("setTimeEpoch")) {
                            uint32_t epoch = ecu_json_doc["setTimeEpoch"] | 0UL;
                            if (epoch == 0)
                            {
                                Serial.println("Failed to decode setTimeEpoch from incoming LoRa message");
                            }
                            else if (isRTCSetByGPS())
                            {
                                Serial.println("setTimeEpoch: RTC already set by GPS; refusing");
                            }
                            else
                            {
                                Teensy3Clock.set(epoch);
                                setRTCSetManually();
                                Serial.println("RTC set from RATS: epoch=" + String(epoch));
                            }
                        }
                        if (ecu_json_doc.containsKey("loraSuspendSec")) {
                            int suspend_sec = ecu_json_doc["loraSuspendSec"] | -1;
                            if (suspend_sec >= 0)
                            {
                                lora_tx_suspend.active = true;
                                lora_tx_suspend.timer = 0;
                                lora_tx_suspend.duration_ms = (uint32_t)suspend_sec * 1000UL;
                                lora_tx_suspend.gps_override_enabled = false;
                                Serial.println("LoRa TX suspended for " + String(suspend_sec) + " s");
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
    SerialUSB.print(sensor_data.magX_mG);
    SerialUSB.print(",");
    SerialUSB.print(sensor_data.magY_mG);
    SerialUSB.print(",");
    SerialUSB.print(sensor_data.magZ_mG);
    SerialUSB.print(",");
    SerialUSB.print(sensor_data.accelX_mg);
    SerialUSB.print(",");
    SerialUSB.print(sensor_data.accelY_mg);
    SerialUSB.print(",");
    SerialUSB.print(sensor_data.accelZ_mg);
    SerialUSB.print(",");
    SerialUSB.print(sensor_data.roll_deg);
    SerialUSB.print(",");
    SerialUSB.print(sensor_data.pitch_deg);
    SerialUSB.print(",");
    SerialUSB.print(sensor_data.heading_deg);
    SerialUSB.print(",");
    SerialUSB.print(sensor_data.orientation_quality);
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