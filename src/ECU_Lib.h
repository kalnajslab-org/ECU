#ifndef ECU_LIB_H
#define ECU_LIB_H

#include <Arduino.h>
#include "ECULoRa.h"

struct ECUBoardHealth_t {
    float V12;
    float V5;
    float V56;
    float ISW;
    float TempC;
};

bool initializeECU();
void getBoardHealth(ECUBoardHealth_t& boardVals);

// Enable 12V power supply
void enable12V(bool enable);

#endif // ECU_LIB_H