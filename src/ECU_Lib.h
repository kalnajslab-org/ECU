#ifndef ECU_LIB_H
#define ECU_LIB_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
#include "ECUHardware.h"
#include "ECULoRa.h"
#include "RS41.h"
#include "ECUReport.h"
#include "etl/vector.h"

// ECU_Lib uses the DalaasTemperature library to access the DS18B20 sensor
// It seems to be much more robust and functional than the Arduino DS18B20 library:
// https://github.com/milesburton/Arduino-Temperature-Control-Library.git
// The wiki for this library is very useful:
// https://www.milesburton.com/w/index.php/Dallas_Temperature_Control_Library
// The author mentions that Paul Stoffregren's OneWire library should be used
// because it fixes serious bugs in the standard Arduino OneWire library. 
// Fortunately this library is selected by default in the Teensyduino package.

// The hysteresis for the board heater control
#define BOARD_TEMP_HYSTERESIS_C 2.0
struct ECUBoardHealth_t {
    float V12;
    float V5;
    float V56;
    float ISW;
    float BoardTempC;
    float CpuTempC;
};

// The TSEN message length is 19 characters, plus one for the null terminator
#define TSEN_MSG_LEN 20
typedef etl::vector<char, TSEN_MSG_LEN> TSEN_DATA_VECTOR;

static JsonDocument ecu_json_doc;

// The ECU boots with LoRa TX suspended for this long, to protect GPS
// acquisition on a passive antenna. Ends early if GPS gets a fix first.
#define LORA_BOOT_SUSPEND_MS (2UL * 60UL * 1000UL)  // 2 minutes

// Tracks a LoRa TX suspend period. The boot-time suspend is cut short as
// soon as GPS acquires a fix (gps_override_enabled = true). A suspend
// requested via the "loraSuspendSec" command is unconditional: it runs for
// the full requested duration regardless of GPS validity
// (gps_override_enabled = false).
struct LoraTxSuspend_t {
    bool active = true;                        // starts suspended at boot
    elapsedMillis timer;
    uint32_t duration_ms = LORA_BOOT_SUSPEND_MS;
    bool gps_override_enabled = true;
};

// Clears `active` once the duration has elapsed, or (if gps_override_enabled)
// once GPS has a valid fix. Call once per loop iteration.
void update_lora_tx_suspend(LoraTxSuspend_t& suspend, bool gps_valid);

/**
 * @brief Initializes the ECU (Electronic Control Unit).
 * 
 * This function sets up the ECU with the specified LoRa report interval.
 * 
 * @param lora_report_interval_ms The shortest interval in milliseconds at which the ECU should report via LoRa.
 * @return true if the initialization was successful, false otherwise.
 */
bool initializeECU(int lora_report_interval_ms, RS41& rs41);

// Return the ecu_id
uint8_t ecu_id();

// Check for an incoming LoRa message, and process it if it is for this ECU.
// If the message is a request for RS41 metadata, set the flag indicating so.
// If the message is a "loraSuspendSec" command, (re)arm lora_tx_suspend.
void process_lora(float& tempC_setpoint, RS41& rs41, bool& rs41_metadata_requested,
                   LoraTxSuspend_t& lora_tx_suspend);

/**
 * @brief Gets the health of the ECU board.
 * 
 * This function reads the voltages and current values from the ECU board and stores
 * them in the ECUBoardHealth_t struct passed as a parameter.
 * 
 * @param boardVals The ECUBoardHealth_t struct to store the board health values.
 */
void getBoardHealth(ECUBoardHealth_t& boardVals);

void tsen_prompt();

TSEN_DATA_VECTOR tsen_read();

void print_tsen(TSEN_DATA_VECTOR& tsen_data);

// Manage the ECU board heater, based on the temperature setpoint and the
// current board temperature. The heater is turned on if the board temperature
// is below the setpoint, and turned off if the board temperature is above the
// setpoint. A hysteresis of 2 degrees is used to prevent rapid switching of
// the heater.
void manage_heater(float board_temp_C, float temp_setpoint_C);

void print_rs41(RS41::RS41SensorData_t& sensor_data);

void print_gps(TinyGPSPlus& gps);

void print_board_health(ECUBoardHealth_t& boardVals);

// Prepare an RS41 metadata ECUReport.
ECUReport_t rs41_report(RS41& rs41);    

#endif // ECU_LIB_H