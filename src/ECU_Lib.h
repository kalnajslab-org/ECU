#ifndef ECU_LIB_H
#define ECU_LIB_H

#include <Arduino.h>
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


/**
 * @brief Initializes the ECU (Electronic Control Unit).
 * 
 * This function sets up the ECU with the specified LoRa report interval.
 * 
 * @param lora_report_interval_ms The shortest interval in milliseconds at which the ECU should report via LoRa.
 * @return true if the initialization was successful, false otherwise.
 */
bool initializeECU(int lora_report_interval_ms, RS41& rs41);

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

void print_rs41(RS41::RS41SensorData_t& sensor_data);

void print_gps(TinyGPSPlus& gps);

void print_board_health(ECUBoardHealth_t& boardVals);

// Prepare an RS41 metadata ECUReport.
ECUReport_t rs41_report(RS41& rs41);    

#endif // ECU_LIB_H