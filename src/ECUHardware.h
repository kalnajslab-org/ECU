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

#define RS41_EN             2
#define V12_EN              5

#define SW_I_HRES_EN        6
#define SW_IMON_EN          7
#define SW_FAULT            8

#define V_ZEPHR_VMON        20
#define V12_MON             21
#define DS18_TEMP           22
#define SW_IMON             23
#define V5_MON              24
#define HEATER_DISABLE      37

//Standard current sense ratio, see p.9:
// https://www.ti.com/lit/ds/symlink/tps281c100.pdf?ts=1704232917558&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FTPS281C100
#define K_SNS1              800.0 
// High accuracy current sense ratio
#define K_SNS2              24.0
// Current sense resistor value
#define R_SNS               5000.0
// Enable/disable high resolution current sense
#define I_HRES              false

#endif /* _ECUHARDWARE_H_ */
