#include <Arduino.h>
#include "ECUHardware.h"
#include "ECUReport.h"
#include "ECU_Lib.h"
#include "ECUConsole.h"
#include "RS41.h"
#include "ecu_version.h"
#include <Watchdog_t4.h>

// The minimum time between LoRa transmissions. This is important to prevent 
// flooding the LoRa channel, which can cause packet loss and communication //
// issues.
#define LORA_MIN_TX_MILLIS 1000
// The period for when a sample is transmitted
#define SAMPLE_MILLIS 2000
// The period for when the ECU report is printed to the serial console
#define PRINT_MILLIS 10000

elapsedMillis lora_tx_timer;
elapsedMillis sample_timer;
elapsedMillis print_timer;
LoraTxSuspend_t lora_tx_suspend;
TinyGPSPlus ecu_gps;
ECUReport_t ecu_report;
RS41 rs41(RS41_SERIAL, RS41_EN);
float tempC_setpoint = 0;
bool rs41_regen_active = false;
WDT_T4<WDT1> wdt;  // Use Watchdog Timer1 on Teensy 4.1
static bool rs41_metadata_requested = false;


void setup()
{
    Serial.begin(115200);
    delay(3000);
    Serial.println("ECU " ECU_VERSION " Build: " __DATE__ " " __TIME__);
    Serial.println("Starting ECU...");

    // Initialize WDT with a 10 second timeout. The callback is not used, 
    // but the WDT will reset the board if the timeout expires.
    WDT_timings_t config;
    config.timeout = 10; /* in seconds, 0->128 for watchdog reboot */
    wdt.begin(config);
  
    if (wdt.expired()) {
         SerialUSB.println("Reset caused by watchdog");
    }

    // Initialize the ECU and peripherals.
    initializeECU(1000, rs41);

    // Initialize the ECU report once at boot. loop() no longer
    // re-initializes it every iteration -- ecu_report_init() is only called
    // again after a report is actually transmitted (see below), so
    // add_gps()/add_rs41()/add_tsen() are the sole writers of their fields
    // and each report reflects the latest known data instead of flickering
    // to zero/invalid on a loop that misses a sensor read.
    ecu_report_init(ecu_report, ecu_id());

    // Seed the GPS sentinel state once.
    add_gps(false, 0.0, 0.0, 0.0, 0, 0, 0, 255, ecu_report);
}

void loop()
{
    static int missed_tsen = 0;

    // Reset the watchdog timer at the beginning of each loop iteration
    wdt.feed();

    // Handle LoRa incoming messages and set flags for actions to take in the main loop,
    // such as requesting RS41 metadata or changing the temperature setpoint.
    process_lora(tempC_setpoint, rs41, rs41_metadata_requested, lora_tx_suspend);

    // Read console commands (bench testing only; e.g. "t" to read/set the RTC).
    consoleRead();

    // GPS
    while (ECU_GPS_SERIAL.available() > 0)
    {
        if (ecu_gps.encode(ECU_GPS_SERIAL.read()))
        {
            // print_gps(ecu_gps);

            // Discipline the RTC whenever the GPS has a valid date/time
            // (which can happen before a location fix is acquired).
            update_rtc_from_gps(ecu_gps);

            // Report the GPS's date/time when valid; otherwise fall back to
            // the RTC's best-known time (from an earlier GPS fix this
            // power-up, a console "t" command, or a "setTimeEpoch" command
            // from RATS), if it has been set at all.
            uint32_t report_date = ecu_gps.date.value();
            uint32_t report_time = ecu_gps.time.value();
            if (!(ecu_gps.date.isValid() && ecu_gps.time.isValid()) && isRTCSet())
            {
                get_rtc_date_time(report_date, report_time);
            }

            add_gps(
                ecu_gps.location.isValid(),
                ecu_gps.location.lat(),
                ecu_gps.location.lng(),
                ecu_gps.altitude.meters(),
                ecu_gps.satellites.value(),
                report_date,
                report_time,
                ecu_gps.location.age() / 1000,
                ecu_report);
        }
    }

    // Update the LoRa TX suspend state (boot-time suspend, or a previous
    // loraSuspendSec command) based on current GPS validity.
    update_lora_tx_suspend(lora_tx_suspend, ecu_gps.location.isValid());

    // TSEN
    TSEN_DATA_VECTOR tsen_data = tsen_read();
    if (tsen_data.size() == 19 && tsen_data[0] == '#')
    {
        // print_tsen(tsen_data);
        // TSEN_DATA_VECTOR data format:
        // "#001 76FC44 80D4A2\r\0"
        // Extract substrings for each sensor
        TSEN_DATA_VECTOR airt_v; airt_v.assign(tsen_data.begin()+1, tsen_data.begin()+4); airt_v.push_back('\0');
        TSEN_DATA_VECTOR prest_v; prest_v.assign(tsen_data.begin()+5, tsen_data.begin()+11); prest_v.push_back('\0');
        TSEN_DATA_VECTOR pres_v; pres_v.assign(tsen_data.begin()+12, tsen_data.begin()+18); pres_v.push_back('\0');
        // Convert substrings to unsigned integers
        uint16_t airt_val = strtoul(airt_v.data(), NULL, 16);
        uint32_t prest_val = strtoul(prest_v.data(), NULL, 16);
        uint32_t pres_val = strtoul(pres_v.data(), NULL, 16);
        // Add to the ECU report
        add_tsen(airt_val, prest_val, pres_val, ecu_report);
    }
    else
    {
        missed_tsen++;
        if (missed_tsen > 5)
        {
            // Haven't received a TSEN message in a while, prompt the TSEN
            tsen_prompt();
            missed_tsen = 0;
        }
    }

    // RS41
    RS41::RS41SensorData_t sensor_data = rs41.decoded_sensor_data(false);
    if (sensor_data.valid)
    {
        //print_rs41(sensor_data);
        if (sensor_data.module_status==88 && sensor_data.module_error==88)
        {
            rs41_regen_active = true;
        } else {
            rs41_regen_active = false;
        }
        const RS41::RS41StatusFlags_t& rs41_flags = sensor_data.flags;
        uint8_t rs41_status = (rs41_flags.high_internal_temp ? ECU_RS41_HIGH_INTERNAL_TEMP : 0u)
                             | (rs41_flags.regen_temp_low     ? ECU_RS41_REGEN_TEMP_LOW     : 0u)
                             | (rs41_flags.ptu_failure        ? ECU_RS41_PTU_FAILURE        : 0u)
                             | (rs41_flags.flash_failure      ? ECU_RS41_FLASH_FAILURE      : 0u)
                             | (rs41_flags.low_input_voltage  ? ECU_RS41_LOW_INPUT_VOLTAGE  : 0u)
                             | (rs41_flags.not_calibrated     ? ECU_RS41_NOT_CALIBRATED     : 0u)
                             | (rs41_flags.no_pressure_module ? ECU_RS41_NO_PRESSURE_MODULE : 0u)
                             | (rs41_flags.disconnected_boom  ? ECU_RS41_DISCONNECTED_BOOM  : 0u);
        add_rs41(
            true,
            rs41_regen_active,
            sensor_data.air_temp_degC,
            sensor_data.humdity_percent,
            sensor_data.hsensor_temp_degC,
            sensor_data.pres_mb,
            sensor_data.heading_deg,
            rs41_status,
            sensor_data.pcb_heater_on,
            ecu_report
        );
    }
    else
    {
        SerialUSB.println("Unable to obtain RS41 sensor data");
    }

    // Board health
    ECUBoardHealth_t boardVals;
    getBoardHealth(boardVals);
    // print_board_health(boardVals);

    // Control ECU heater based on board temperature and setpoint
    manage_heater(boardVals.BoardTempC, tempC_setpoint);

    add_status(!digitalRead(HEATER_DISABLE), tempC_setpoint, rs41_regen_active, digitalRead(RS41_EN), digitalRead(V12_EN), ecu_report);
    add_ecu_health(
        boardVals.V5,
        boardVals.V12,
        boardVals.V56,
        boardVals.BoardTempC,
        boardVals.ISW,
        boardVals.CpuTempC,
        ecu_report
    );

    // Transmit reports. Only one type: regular ECU report or RS41 metadata report
    // will be sent per loop iteration. The periodic ECU report is gated by
    // lora_tx_suspend; a requested RS41 metadata reply is not, since it's a
    // direct reply to a received command and must always go out.
    if (!lora_tx_suspend.active && !rs41_metadata_requested && sample_timer > SAMPLE_MILLIS && lora_tx_timer > LORA_MIN_TX_MILLIS)
    {
        sample_timer = 0;
        lora_tx_timer = 0;
        // Serialize and transmit the ECU report
        auto payload = ecu_report_serialize(ecu_report);
        if (!ecu_lora_tx(payload.begin(), ecu_report_serialized_size(ecu_report)))
        {
            Serial.println("Failed to transmit LoRa.");
        } else {
            Serial.println("Sent ECUReport");
        }
        // Reset the report now that it's been sent, so the next report
        // accumulates fresh sensor data across however many loop
        // iterations it takes.
        ecu_report_init(ecu_report, ecu_id());
    }
    
    if (rs41_metadata_requested && lora_tx_timer > LORA_MIN_TX_MILLIS)
    {
        // RS41 metadata report
        rs41_metadata_requested = false;
        lora_tx_timer = 0;
        auto report = rs41_report(rs41);
        auto payload = ecu_report_serialize(report);
        Serial.print("Sending:");
        ecu_report_print_raw(report);
        Serial.println();
        if (!ecu_lora_tx(payload.begin(), ecu_report_serialized_size(report)))
        {
            Serial.println("Failed to transmit LoRa.");
        }
    }

    if (print_timer > PRINT_MILLIS)
    {
        print_timer = 0;
        // Deserialize (for verification) and print the ECU report
        auto payload = ecu_report_serialize(ecu_report);
        auto ecu_report_sent = ecu_report_deserialize(payload);
        ecu_report_print(ecu_report_sent, true);
        Serial.println("--------------------");
    }

}