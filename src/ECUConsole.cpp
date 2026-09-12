#include "ECUConsole.h"
#include "ECU_Lib.h"
#include <Arduino.h>
#include <etl/string.h>
#include <etl/vector.h>

void consoleRead()
{
    static etl::string<128> line;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (line.empty()) { return; }

            etl::vector<etl::string<32>, 8> tokens;
            etl::string<32> current;

            for (char ch : line) {
                if (ch == ' ' || ch == ',') {
                    if (!current.empty()) {
                        tokens.push_back(current);
                        current.clear();
                    }
                } else if (!current.full()) {
                    current.push_back(tolower((unsigned char)ch));
                }
            }
            if (!current.empty() && !tokens.full()) {
                tokens.push_back(current);
            }

            line.clear();

            if (tokens.empty()) { return; }

            if (tokens[0] == "h") {
                Serial.println("Commands:");
                Serial.println("  h        - help");
                Serial.println("  t        - print current RTC time (UTC)");
                Serial.println("  t <yyyy> <mm> <dd> <hh> <mm> <ss> - set RTC time (UTC);");
                Serial.println("           refused once GPS has set the RTC");
            } else if (tokens[0] == "t") {
                if (tokens.size() == 1) {
                    time_t rtc_epoch = Teensy3Clock.get();
                    struct tm* t = gmtime(&rtc_epoch);
                    Serial.printf("%04d-%02d-%02d %02d:%02d:%02d UTC (%s)\n",
                        t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec,
                        isRTCSet() ? "set" : "NOT SET");
                } else if (tokens.size() < 7) {
                    Serial.println("usage: t <yyyy> <mm> <dd> <hh> <mm> <ss>");
                } else if (isRTCSetByGPS()) {
                    Serial.println("ERROR: RTC already set by GPS; refusing manual set");
                } else {
                    int yr   = atoi(tokens[1].c_str());
                    int mon  = atoi(tokens[2].c_str());
                    int day_ = atoi(tokens[3].c_str());
                    int hr   = atoi(tokens[4].c_str());
                    int min_ = atoi(tokens[5].c_str());
                    int sec  = atoi(tokens[6].c_str());
                    if (yr < 1970 || mon < 1 || mon > 12 || day_ < 1 || day_ > 31 ||
                        hr < 0 || hr > 23 || min_ < 0 || min_ > 59 || sec < 0 || sec > 59) {
                        Serial.println("ERROR: invalid date/time");
                    } else {
                        struct tm t = {};
                        t.tm_year = yr - 1900;
                        t.tm_mon  = mon - 1;
                        t.tm_mday = day_;
                        t.tm_hour = hr;
                        t.tm_min  = min_;
                        t.tm_sec  = sec;
                        Teensy3Clock.set(mktime(&t));
                        setRTCSetManually();
                        Serial.printf("RTC set to %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                            yr, mon, day_, hr, min_, sec);
                    }
                }
            } else {
                Serial.printf("Unknown command: %s  (send 'h' for help)\n", tokens[0].c_str());
            }

        } else if (!line.full()) {
            line.push_back(c);
        }
    }
}
