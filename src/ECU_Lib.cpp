#include "ECU_Lib.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include "ECUHardware.h"
#include "ECULoRa.h"

#define DS_RESOLUTION 12

static OneWire oneWire(DS18_TEMP);
static DallasTemperature ds(&oneWire);

//Resistance values in kOhms
static float R5 = 499.0;
static float R6 = 10.0;
static float R13 = 30.0;
static float R14 = 10.0;
static float R15 = 48.7;
static float R16 = 10.0;

bool initializeECU() {

    bool success = true;
    pinMode(RS41_EN, OUTPUT);
    pinMode(V12_EN, OUTPUT);
    pinMode(SW_I_HRES_EN,OUTPUT);
    pinMode(SW_IMON_EN,OUTPUT);
    pinMode(V_ZEPHR_VMON,INPUT);
    pinMode(V12_MON,INPUT);
    pinMode(V5_MON,INPUT);
    pinMode(HEATER_DISABLE,OUTPUT);

    enable12V(false);

    ds.begin();
    ds.setResolution(DS_RESOLUTION);
  
    ds.setWaitForConversion(false);
    ds.requestTemperatures();

    if (!ECULoRaInit(
            LORA_LEADER, 
            1000, 
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

    if (ds.getCheckForConversion()) {
        last_temp = ds.getTempCByIndex(0);
        ds.requestTemperatures();
    }
    boardVals.TempC = last_temp;

    float K_SNS;
    if (I_HRES) {
      K_SNS = K_SNS2;
      digitalWrite(SW_I_HRES_EN,HIGH);
    }
    else {
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
