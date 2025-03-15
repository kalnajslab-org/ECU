#ifndef _ECUHARDWARE_H_
#define _ECUHARDWARE_H_

// LoRa Module
#define ECU_LORA_CS         10
#define ECU_LORA_MOSI       11
#define ECU_LORA_MISO       12
#define ECU_LORA_SCK        13
#define ECU_LORA_INT        14
#define ECU_LORA_RST        15

//LoRa Settings
#define LORA_FREQ           868E6
#define LORA_BW             250E3
#define LORA_SF             9
#define LORA_POWER          19

// GPS
#define ECU_GPS_SERIAL      Serial8
#define ECU_GPS_BAUD        9600
#define ECU_GPS_BUFFSIZE    4096    

// TSEN
#define ECU_TSEN_SERIAL     Serial4
#define ECU_TSEN_BAUD       9600
#define ENABLE_12V          5

// RS41
#define RS41_SERIAL         Serial1
#define RS41_BAUD           56700
#define RS41_EN             2

// A/D port for Zephr voltage monitor
#define V_ZEPHR_VMON        20
// A/D port for 12V monitor
#define V12_MON             21
// OneWire port for DS18B20 temperature sensor
#define DS18_TEMP           22
// A/D port for 5V monitor
#define V5_MON              24
// Digital output port to disable heater
#define HEATER_DISABLE      37

// Digital output port to enable 12V power supply
#define V12_EN              5

// The TPS281C100x high voltage power switch with current sensing.
//
// Current measurement is done by shunting a fraction of the current 
// through a user provided resistor. Two modes are available:
// 800:1 for standard accuracy and 24:1 for high accuracy.
// see p.9:
// https://www.ti.com/lit/ds/symlink/tps281c100.pdf?ts=1704232917558&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTPS281C100
// A/D port to monitor current sensing voltage
#define SW_IMON             23
// Digital output port to enable current sense on the switch
#define SW_IMON_EN          7
// Digital output port to enable high accuracy current sense on the switch
#define SW_I_HRES_EN        6
// Digital output port to enable fault detection on the switch
#define SW_FAULT            8
// Current sense resistor value
#define R_SNS               5000.0
// Enable/disable high resolution current sense
// Should not enable, since it has the potential to create an output voltage
// that exceeds the 3.3V, which will kill the Teensy
// Set to either true or false
#define SNS_I_HRES          false

#endif /* _ECUHARDWARE_H_ */
