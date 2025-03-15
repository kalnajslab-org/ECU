#include <Arduino.h>
#include "ECUHardware.h"

char device_type = ' ';

void setup()
{

    SerialUSB.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    delay(3000);
    SerialUSB.println("Build date: " __DATE__ " " __TIME__);
    SerialUSB.println("Starting ecu_passthrough...");
    bool valid_device = false;
    SerialUSB.println("R/r for RSS421, T/t for TSEN, G/g for GPS)");

    while (!valid_device)
    {
        if (SerialUSB.available())
        {
            device_type = SerialUSB.read();
            switch (device_type)
            {
            case 'R':
            case 'r':
                SerialUSB.println("RSS421 selected (RSD to read sensor data)");
                RS41_SERIAL.begin(RS41_BAUD);
                device_type = 'R';
                pinMode(RS41_EN, OUTPUT);
                digitalWrite(RS41_EN, HIGH);
                valid_device = true;
                break;
            case 'T':
            case 't':
                SerialUSB.println("TSEN selected (*01A? for data)");
                ECU_TSEN_SERIAL.begin(ECU_TSEN_BAUD);
                device_type = 'T';
                pinMode(ENABLE_12V, OUTPUT);
                delay(500);
                digitalWrite(ENABLE_12V, HIGH);
                valid_device = true;
                break;
            case 'G':
            case 'g':
                SerialUSB.println("GPS selected");
                ECU_GPS_SERIAL.begin(ECU_GPS_BAUD);
                device_type = 'G';
                valid_device = true;
                break;
            default:
                SerialUSB.println("R/r for RSS421, T/t for TSEN, G/g for GPS)");
            break;
            }
        }
    } 
}

void loop()
{
    static uint last_blink = 0;
    if ((millis() - last_blink) > 200)
    {
        last_blink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    if (SerialUSB.available())
    {
        char c = SerialUSB.read();
        switch (device_type)
        {
        case 'R':
            RS41_SERIAL.write(c);
            break;
        case 'T':
            ECU_TSEN_SERIAL.write(c);
            break;
        case 'G':
            ECU_GPS_SERIAL.write(c);
            break;
        default:
            break;
        }
    }

    switch (device_type)
    {
    case 'R':
        if (RS41_SERIAL.available())
        {
            SerialUSB.write(RS41_SERIAL.read());
        }
        break;
    case 'T':
        if (ECU_TSEN_SERIAL.available())
        {
            SerialUSB.write(ECU_TSEN_SERIAL.read());
        }
        break;
    case 'G':
        if (ECU_GPS_SERIAL.available())
        {
            SerialUSB.write(ECU_GPS_SERIAL.read());
        }
        break;
    default:
        break;
    }
}