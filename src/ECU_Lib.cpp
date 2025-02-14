#include "ECU_Lib.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include "ECUHardware.h"
#include "ECULoRa.h"

#define DS_RESOLUTION 12

static OneWire oneWire(DS18_TEMP);
static DallasTemperature ds18(&oneWire);

//Resistance values in kOhms
#define R5  499.0
#define R6  10.0
#define R13 30.0
#define R14 10.0
#define R15 48.7
#define R16 10.0

bool initializeECU(int lora_report_interval_ms) {

    bool success = true;

    // Initialize the digital pins
    pinMode(RS41_EN, OUTPUT);
    pinMode(V12_EN, OUTPUT);
    pinMode(SW_I_HRES_EN,OUTPUT);
    pinMode(SW_IMON_EN,OUTPUT);
    pinMode(V_ZEPHR_VMON,INPUT);
    pinMode(V12_MON,INPUT);
    pinMode(V5_MON,INPUT);
    pinMode(HEATER_DISABLE,OUTPUT);

    // Disable the 12V
    enable12V(false);

    // Configure the DS18B20 temperature sensor
    ds18.begin();
    ds18.setResolution(DS_RESOLUTION);
    ds18.setWaitForConversion(false);

    // Start the temperature conversion
    ds18.requestTemperatures();

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
            LORA_POWER)) {
        Serial.println("LoRa Initialization Failed!");
        success = false;
    } else {
        Serial.println("LoRa Initialized Successfully!");
    }

    return success;
}   

void getBoardHealth(ECUBoardHealth_t& boardVals) {
    static float last_temp = -1000.0;

    if (ds18.isConversionComplete()) {
        last_temp = ds18.getTempCByIndex(0);
        ds18.requestTemperatures();
    }
    boardVals.TempC = last_temp;

    float K_SNS;
    if (I_HRES) {
      K_SNS = K_SNS2;
      digitalWrite(SW_I_HRES_EN,HIGH);
    } else {
      K_SNS = K_SNS1;
    }

    digitalWrite(SW_IMON_EN, HIGH);

    boardVals.V56 = analogRead(V_ZEPHR_VMON) * (R5 + R6) * 3.3 / (1024.0  * R6);
    boardVals.V5    = analogRead(V5_MON) * (R13 + R14) * 3.3 / (1024.0 * R14);
    boardVals.V12   = analogRead(V12_MON) * (R15 + R16) * 3.3 / (1024.0 * R16);
    boardVals.ISW   = (analogRead(SW_IMON) / R_SNS) * K_SNS;
  
    digitalWrite(SW_IMON_EN, LOW);
    digitalWrite(SW_I_HRES_EN, LOW);
  }
  
  void enable12V(bool enable) {
    digitalWrite(V12_EN, enable);
  }
