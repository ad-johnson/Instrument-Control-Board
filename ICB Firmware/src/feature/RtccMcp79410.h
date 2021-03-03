/*******************************************************************************************
 * 
 * RtccMcp7940: Class to interact with a MCP79410 RTCC.
 *              Updated for Arduino using PlatformIO and C++17 support.
 * 
 * Author: Modified for MCP79410 only with RTCx and MCP79412RTC features mergedby Andrew Johnson
 * 
 * Attribution: https://github.com/stevemarple/RTCx  Arduino library by Steve Marple.
 * 				https://github.com/JChristensen/MCP79412RTC Arduino library by Jack Christensen
 * 
 * Datasheet: http://ww1.microchip.com/downloads/en/devicedoc/20002266h.pdf
 * February 2021
 * 
 *******************************************************************************************/
#ifndef RtccMcp79410_h
#define RtccMcp79410_h

#include "Arduino.h"
#include <stdint.h>

class RtccMcp79410 {
public:

	static const uint16_t rtccEpoch {1970}; // Base year for date calculations
	static const uint16_t rtccCentury {1900}; // Base Century for date calculations
	static const uint8_t rtccYearsSinceCenturyStart {rtccEpoch - rtccCentury};
	static const uint8_t epochDayOfTheWeek {4}; // Day of the week for the epoch date
	static const uint32_t secsPerDay {86400};
	static const uint32_t secsPer4Years {secsPerDay * (366 + 365 + 365 + 365)};

	static const uint8_t address {0x6F};
	static const uint8_t eepromAddress {0x57};

	static const uint8_t addressTop {0x5F};			// maximum address for registers/SRAM
	static const uint8_t sramStartAddress {0x20};	// first SRAM address
	static const uint8_t sramSize {64};         	// number of bytes of SRAM
	static const uint8_t eepromSize {128};      	// number of bytes of EEPROM
	static const uint8_t eepromPageSize {8};   		// number of bytes on an EEPROM page

	// Supported time functions, based on Register map (Table 5.1, page 13).  
	// Does not cover the CONTROL, OSCTRIM or EEUNLOCK registers
	enum timeFunc_t {
		TIME = 0,					// Timekeeping
		ALARM0 = 1,					// Alarms - 1
		ALARM1 = 2,					// Alarms - 2
		TIME_POWER_FAILED = 3,		// Power-down time stamp
		TIME_POWER_RESTORED = 4	// Power-up time stamp
	};

	enum freq_t {
		freq1Hz = 0,
		freq4096Hz = 1,
		freq8192Hz = 2,
		freq32768Hz = 3,
		freqCalibration = 4 // device-specific calibration for MCP79410
	};

	enum MFPLevel {
		low,
		high
	};

	// Alarm types for use with the enableAlarm() function
	enum alarmTypes {
		almMatchSeconds,
		almMatchMinutes,
		almMatchHours,
		almMatchDay,      // triggers alarm at midnight
		almMatchDate,
		ALM_RESERVED_5,     // do not use
		ALM_RESERVED_6,     // do not use
		almMatchDateTime,
		almDisable
	};

	struct tm {
		int tm_sec; // Seconds [0..59]
		int tm_min; // Minutes [0..59].
		int tm_hour; // Hour [0..23].
		int tm_mday; // Day of month [1..31].
		int tm_mon; // Month of year [0..11].
		int tm_year; // Years since 1900.
		int tm_wday; // Day of week [0..6] (Sunday=0).
		int tm_yday; // Day of year [0..365]. (-1=unset).
		int tm_isdst; // Daylight Savings flag (ignored).
	};

	typedef int32_t time_t;

	static bool isLeapYear(uint16_t year);
	static uint8_t daysInMonth(uint16_t year, uint8_t month);
	static uint16_t dayOfYear(uint16_t year, uint8_t month, uint8_t day);
	static time_t mktime(struct tm *tm);
	inline static time_t mktime(struct tm &tm);
	static struct tm *gmtime_r(const time_t *timep, struct tm *result);

	inline static int isotime(const time_t &t, char *buffer, size_t len);
	inline static int isotime(const struct tm &tm, char *buffer, size_t len);
	static int isotime(const struct tm *tm, char *buffer, size_t len);
	
	RtccMcp79410();

	void initialise() const;
	bool isConnected() const;
	bool isRunning() const;			// Indicates whether the oscillator is running.

	void stopClock() const;
	inline void startClock() const {
		startClock((int16_t)-1);
	}

	bool readClock(struct tm *tm, timeFunc_t function = TIME) const;
	inline bool readClock(struct tm &tm, timeFunc_t function = TIME) const;
	bool readClock(char *buffer, size_t len, timeFunc_t function = TIME) const;
	bool setClock(const struct tm *tm, timeFunc_t function = TIME) const;
	inline bool setClock(const struct tm &tm, timeFunc_t function = TIME) const;
	bool setClock(const char* ISOTimestamp, timeFunc_t function = TIME) const;
	bool adjustClock(time_t offset) const;

	// Battery functions
	void enableBatteryBackup(bool enable = true) const;
	bool getPowerFailFlag() const;
	void clearPowerFailFlag() const;

	// Alarm functions
	bool setAlarm(const struct tm *tm, timeFunc_t alarmNumber) const;
	bool readAlarm(struct tm *tm, timeFunc_t alarmNumber) const;
	bool enableAlarm(timeFunc_t alarmNumber, alarmTypes alarmType) const;
	bool isAlarmRaised(timeFunc_t alarmNumber) const;
	void alarmPolarity(MFPLevel polarity) const;

	// MFP functions, with calibration
	void setMFPRestingLevel(MFPLevel level = low) const;	// 'Resting' MFP level when not in use, High or Low
	bool setSquareWave(freq_t frequency) const; 	// Output a square wave at a given frequency
	int8_t getCalibration() const;
	bool setCalibration(int8_t newValue) const;

	// EEPROMfunctions
	bool eepromWrite(uint8_t address, uint8_t value) const;
	bool eepromWrite(uint8_t address, uint8_t *values, uint8_t len) const;
	bool eepromRead(uint8_t address, uint8_t *value) const;
	bool eepromRead(uint8_t address, uint8_t *values, uint8_t len) const;

	// SRAM functions
	bool sramWrite(uint8_t address, uint8_t value) const;
	bool sramWrite(uint8_t address, uint8_t *values, uint8_t len) const;
	bool sramRead(uint8_t address, uint8_t *value) const;
	bool sramRead(uint8_t address, uint8_t *values, uint8_t len) const;

private:
	static uint8_t bcdToDec(uint8_t bcdValue);
	static uint8_t decToBcd(uint8_t decimalValue);

	void startClock(int16_t bcdSec) const;
	uint8_t readData(uint8_t rtccRegister) const;
	bool writeData(uint8_t rtccRegister, uint8_t value) const;
	bool writeData(uint8_t rtccRegister, uint8_t *values, uint8_t len) const;
	uint8_t getRegister(timeFunc_t function, uint8_t &registerSize) const;
	bool readPowerTime(struct tm *tm, uint8_t powerFailRegister, uint8_t registerSize) const;
	bool waitUntilFree() const;
};

RtccMcp79410::time_t RtccMcp79410::mktime(struct tm &tm) {
	return mktime(&tm);
}

inline bool RtccMcp79410::readClock(struct tm &tm, timeFunc_t function) const {
	return readClock(&tm, function);
}

inline bool RtccMcp79410::setClock(const struct tm &tm, timeFunc_t function) const {
	return setClock(&tm, function);
}

inline int RtccMcp79410::isotime(const time_t &t, char *buffer, size_t len) {
	struct tm tm;
	gmtime_r(&t, &tm);
	return isotime(&tm, buffer, len);
}

inline int RtccMcp79410::isotime(const struct tm &tm, char *buffer, size_t len) {
	return isotime(&tm, buffer, len);
}

#endif // RtccMcp7940.h
