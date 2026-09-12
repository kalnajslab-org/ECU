#ifndef _ECUCONSOLE_H_
#define _ECUCONSOLE_H_

/**
 * Reads and processes commands from the USB serial console, for bench
 * testing. Reads one buffered line at a time, split on spaces/commas
 * (converting to lowercase), and dispatches on the first token.
 *
 * Supported commands:
 *   h                                   - help
 *   t                                   - print current RTC time (UTC)
 *   t <yyyy> <mm> <dd> <hh> <mm> <ss>    - set RTC time (UTC); refused once
 *                                          GPS has set the RTC (see
 *                                          isRTCSetByGPS() in ECU_Lib.h)
 *
 * Call once per loop() iteration.
 */
void consoleRead();

#endif /* _ECUCONSOLE_H_ */
