/*******************************************************************************************
 * 
 * RtccMcp7940: Class to interact with a MCP79410 RTCC.
 *              Updated for Arduino using PlatformIO and C++17 support.
 * 
 * Author: Modified for MCP3428 only by Andrew Johnson
 * 
 * Attribution: https://github.com/stevemarple/RTCx  Arduino library by Steve Marple.
 * 
 * Datasheet: http://ww1.microchip.com/downloads/en/devicedoc/20002266h.pdf
 * February 2021
 * 
 *******************************************************************************************/
#include "Wire.h"
#include "RtccMcp79410.h"

// #define DEBUG_RTCCMCP79410 // Uncomment to get serial UART debug strings

#ifdef ARDUINO_ARCH_AVR
#define SNPRINTF snprintf_P
#else
#define SNPRINTF snprintf
#endif


/********************************************************************************************
 * bool isLeapYear(uint16_t year)
 * 
 * Scope: public
 * Parameters:
 * @param year:	the year to determine if it is a leap year
 * Returns:
 * @return is a leap year:	true or false
 * 
 * Description:
 * Uses a simplistic check to determine if the year is a leap year or not.  We don't worry here
 * about running over the year 2100 as that is not going to happen.
 * 
 ********************************************************************************************/
bool RtccMcp79410::isLeapYear(uint16_t year) {
	// Since year is > 1900 and < 2100 can use simple method
	return (year % 4) == 0;
}

/********************************************************************************************
 * uint8_t daysInMonth(uint16_t year, uint8_t month)
 * 
 * Scope: public
 * Parameters:
 * @param year:		the year of the month, incase it is February and a leap year.
 * @param month:	the month to determine the number of days
 * Returns:
 * @return number of days
 * 
 * Description:
 * Determines the number of days for the given month.  If it's a leap year and we are checking
 * for February, adjusts the days accordingly.  Note that months start at 1 and not 0.
 * 
 ********************************************************************************************/
uint8_t RtccMcp79410::daysInMonth(uint16_t year, uint8_t month) {
	uint8_t daysInMonth[13] =
		{0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	if (month == 2 && isLeapYear(year))
		return 29;
	return daysInMonth[month];
}

/********************************************************************************************
 * uint16_t dayOfYear(uint16_t year, uint8_t month, uint8_t day)
 * 
 * Scope: public
 * Parameters:
 * @param year:		the year of the month, incase it is February and a leap year.
 * @param month:	the month of the year
 * @param day:		the day of the month 
 * Returns:
 * @return number of days
 * 
 * Description:
 * Converts a date into a day of the year - 0 to 365 - taking into account leap years.
 * 
 ********************************************************************************************/
uint16_t RtccMcp79410::dayOfYear(uint16_t year, uint8_t month, uint8_t day) {
	uint16_t dayOfYear = 0;
	uint8_t startMonth = 1;
	while (startMonth < month)
		dayOfYear += daysInMonth(year, startMonth++);
	dayOfYear += day;
	return dayOfYear;
}

/********************************************************************************************
 * time_t mkTime(struct tm *tm)
 * 
 * Scope: public
 * Parameters:
 * @param *tm:		pointer to a time structure.
 * Returns:
 * @return time in seconds: the total seconds represented by the time structure
 * 
 * Description:
 * Converts a date into seconds since the start of the Epoch.  Calculates additional date
 * information - day of week and day of year.
 * 
 ********************************************************************************************/
RtccMcp79410::time_t RtccMcp79410::mktime(struct tm *tm) {
	// Normalise the time - ensures that values in the time structure are formed into a valid time
	tm->tm_min += (tm->tm_sec / 60);
	tm->tm_sec = (tm->tm_sec % 60);
	if (tm->tm_sec < 0) {
		tm->tm_sec += 60;
		--(tm->tm_min);
	}

	tm->tm_hour += (tm->tm_min / 60);
	tm->tm_min = (tm->tm_min % 60);
	if (tm->tm_min < 0) {
		tm->tm_min += 60;
		--(tm->tm_hour);
	}

	tm->tm_mday += (tm->tm_hour / 24);
	tm->tm_hour = (tm->tm_hour % 24);
	if (tm->tm_hour < 0) {
		tm->tm_hour += 24;
		--(tm->tm_mday);
	}

	if (tm->tm_mon < 0 || tm->tm_mon > 11 || tm->tm_mday < 1)
		return -1;

	// Normalise the date - ensures that the value in the time structure are formed into a valid date
	while (true) {
		uint8_t daysInMon = daysInMonth(tm->tm_year+rtccCentury, tm->tm_mon+1); // Years are elapsed since the century start
		if (tm->tm_mday > daysInMon) {
			tm->tm_mday -= daysInMon;
			++(tm->tm_mon);
			if (tm->tm_mon == 12) {
				tm->tm_mon = 0;
				++(tm->tm_year);
			}
			continue;
		}

		if (tm->tm_mday < 1) {
			--(tm->tm_mon);
			if (tm->tm_mon == -1) {
				tm->tm_mon = 11;
				--(tm->tm_year);
			}

			uint8_t daysInMon = daysInMonth(tm->tm_year+rtccCentury, tm->tm_mon+1);
			tm->tm_mday += daysInMon;
			continue;
		}
		break;
	}

	// Compute day of year
	tm->tm_yday = dayOfYear(tm->tm_year+rtccCentury, tm->tm_mon+1, tm->tm_mday) - 1;

	uint8_t yearsSinceEpoch = tm->tm_year + rtccCentury  - rtccEpoch;
	time_t t = (yearsSinceEpoch * 365 * secsPerDay) // Whole years, leap days excluded
		+ ((yearsSinceEpoch / 4) * secsPerDay)  // Leap days in whole 4 year period
		// Leap days in partial 4 year period. Count only if in last year
		+ ((yearsSinceEpoch % 4) == 3 ? secsPerDay : 0L)
		+ (tm->tm_yday * secsPerDay)           // Whole days in current year
		+ (tm->tm_hour * 3600L)
		+ (tm->tm_min * (uint16_t)60)
		+ tm->tm_sec;

	// Compute day of week
	uint32_t daysSinceEpoch = (t / secsPerDay);
	tm->tm_wday = (daysSinceEpoch + epochDayOfTheWeek) % 7; // 1970-01-01 was Thursday (day 4)
	return t;
}

/********************************************************************************************
 * tm gmTime(const time_t *timep, struct tm *result)
 * 
 * Scope: public
 * Parameters:
 * @param *timep:	Number of seconds since the Epoch.
 * Returns:
 * @return time structure: a date structure created from the number of seconds
 * 
 * Description:
 * Given a number of seconds, converts that into a valid date and time.
 * 
 ********************************************************************************************/
struct RtccMcp79410::tm *RtccMcp79410::gmtime_r(const time_t *timep, struct tm *result) {
	time_t t = *timep;
	// Find multiples of 4 years since epoch
	int8_t fourYears = (int8_t)(t / secsPer4Years);
	if (t < 0)
		--fourYears; // Now remaining time will be positive and must add
	result->tm_year = (fourYears * 4) + (rtccYearsSinceCenturyStart); // years since 1900
	t -= (fourYears * secsPer4Years);

	// Split t into seconds in day and days remaining.
	int16_t days = (t / secsPerDay); // Fits into 16 bits
	time_t partialDay_s = (t % secsPerDay); // seconds

	// Calculate hours, minutes and seconds next so that the rest of the
	// calculations can be made in days using 16 bit arithmetic.
	result->tm_sec = (partialDay_s % 60);
	int16_t partialDay_m = partialDay_s / 60; // minutes
	result->tm_min = (partialDay_m % 60);
	result->tm_hour = partialDay_m / 60;

	if (days >= (365 + 365 + 366)) {
		// Third year in a four year block is a leap year
		days -= (365 + 365 + 366);
		result->tm_year += 3;
	}
	else
		while (days >= 365) {
			days -= 365;
			++(result->tm_year);
		}

	// days is now the day of year
	result->tm_yday = days;
	result->tm_mon = 0;
	result->tm_mday = 1 + days;

	while (true) {
		uint8_t dim = daysInMonth(result->tm_year+rtccCentury, result->tm_mon+1);
		if (result->tm_mday > dim) {
			result->tm_mday -= dim;
			++(result->tm_mon);
		}
		else
			break;
	}

	// Compute day of week
	uint16_t daysSinceEpoch = (*timep / 86400L);
	result->tm_wday = (daysSinceEpoch + epochDayOfTheWeek) % 7; // 1970-01-01 was Thursday (day 4)
	return result;
}

/********************************************************************************************
 * Constructor
 * 
 * Scope: public
 * Parameters: None
 * Returns: None
 * 
 * Description:
 * Creates an instance of the RtccMcp79410 class.
 * 
 ********************************************************************************************/
RtccMcp79410::RtccMcp79410() {
}

/********************************************************************************************
 * bool isConnected()
 * 
 * Scope: public
 * Parameters: None
 * Returns: 
 * @return connected: true or false depending on whether it is found at the configured address.
 * 
 * Description:
 * Request a read from the device and if it acknowledges, then it is connected.
 * 
 ********************************************************************************************/
bool RtccMcp79410::isConnected() const {
    // Ensure register address is valid
    Wire.beginTransmission(address);
    Wire.write(uint8_t(0));
    Wire.endTransmission();

    Wire.requestFrom(address, (uint8_t)1);
    if (!Wire.available()) {
    #ifdef DEBUG_RTCCMCP79410
        Serial.print("No ADC MCP3428 device found at address ");
        Serial.println(address, HEX);
    #endif // DEBUG_RTCCMCP79410
        return false;
    }
    return true;
}

/********************************************************************************************
 * void initialise()
 * 
 * Scope: public
 * Parameters: None
 * Returns: None
 * 
 * Description:
 * Initialises the RTCC to a default state: battery backup enabled, calibration reset to 0.
 * Note that if the RTCC is already running then this method should only be called when a 
 * reset is required and NOT as a general start up routine.
 * 
 ********************************************************************************************/
void RtccMcp79410::initialise() const {
	enableBatteryBackup();
	setCalibration(0);
	setMFPRestingLevel(low);
	startClock();
}

/********************************************************************************************
 * void stopClock()
 * 
 * Scope: public
 * Parameters: None
 * Returns: None
 * 
 * Description:
 * Stops the clock by setting the oscillator bit to a 0.  This is bit 8 in the RTCSEC (seconds)
 * Timekeeping register.
 * 
 ********************************************************************************************/
void RtccMcp79410::stopClock() const {
	uint8_t rtcsec = 0x00;
	uint8_t secondsByte = readData(rtcsec);
	secondsByte &= 0x7f;
	writeData(rtcsec, secondsByte);
}

/********************************************************************************************
 * void startClock(int16_t bcdSec)
 * 
 * Scope: private
 * Parameters:
 * @param bcdsec: 
 * Returns: None
 * 
 * Description:
 * Starts the oscillator running.  A starting seconds can be provided otherwise the current
 * seconds are used (pass in -1.)
 * 
 ********************************************************************************************/
void RtccMcp79410::startClock(int16_t bcdSec) const {
	uint8_t secondsByte;
	uint8_t rtcsec = 0;
	if (bcdSec < 0)
		secondsByte = readData(rtcsec);
	else
		secondsByte = uint8_t(bcdSec & 0x7f);

	uint8_t s2 = secondsByte;
	s2 |= 0x80; // Enable start bit

	// Write back the data if it is different to the contents of the
	// register.  Always write back if the data wasn't fetched with
	// readData as the contents of the stop bit are unknown.
	if (secondsByte != s2 || bcdSec >= 0)
		writeData(rtcsec, s2);
}

/********************************************************************************************
 * bool readClock(struct tm *tm, timeFunc_t func)
 * 
 * Scope: public
 * Parameters:
 * @param tm:		a Time structure to store the read date/time in
 * @param function:	the type of time to read (timekeeping, alarm0/1, Power fail) 
 * Returns: 
 * @return success/fail: an indiction of successful read.
 * 
 * Description:
 * Reads the requested time register and converts into a date/time.  Register bytes are read in
 * order from the starting address, i.e. Seconds, minutes, hours etc 
 * 
 ********************************************************************************************/
bool RtccMcp79410::readClock(struct tm *tm, timeFunc_t function) const {
	// Find which register to read from
	uint8_t registerSize = 0;
	uint8_t timeRegister = getRegister(function, registerSize);

	if (registerSize == 0)
		return false; // not supported

	if (function == TIME_POWER_FAILED || function == TIME_POWER_RESTORED)
		return readPowerTime(tm, timeRegister, registerSize);

	while (true) {
		// Reset the register pointer
		Wire.beginTransmission(address);
		Wire.write(timeRegister);
		Wire.endTransmission();

		Wire.requestFrom(address, registerSize);
		tm->tm_sec = bcdToDec(Wire.read() & 0x7f);
		tm->tm_min = bcdToDec(Wire.read() & 0x7f);
		uint8_t hour = Wire.read();
		if (hour & 0x40) {
			// Twelve hour mode
			tm->tm_hour = bcdToDec(hour & 0x1f);
			if (hour & 0x20)
				tm->tm_hour += 12; // Seems notation for AM/PM is user-defined
		} else {
			tm->tm_hour = bcdToDec(hour & 0x3f);
        }

		tm->tm_wday = (Wire.read() & 0x07) - 1; // Clock uses [1..7]
		tm->tm_mday = bcdToDec(Wire.read() & 0x3f);

		tm->tm_mon = bcdToDec(Wire.read() & 0x1f) - 1; // Clock uses [1..12]
		if (registerSize >= 7) // Alarms only have 6 registers
			tm->tm_year = bcdToDec(Wire.read()) + 100; // Assume 21st century
		else
			tm->tm_year = (rtccYearsSinceCenturyStart);
		tm->tm_yday = -1;
		Wire.endTransmission();

		if ((function != TIME) || (tm->tm_sec == bcdToDec(readData(timeRegister) & 0x7f)))
			break;
	}
	return true;
}

/********************************************************************************************
 * bool readClock(char* buffer, size_t len, timeFunc_t func)
 * 
 * Scope: public
 * Parameters:
 * @param buffer:	Will be loaded with a string representation of the date / time
 * @param function:	the type of time to read (timekeeping, alarm0/1, Power fail) 
 * Returns: 
 * @return success/fail: an indiction of successful read or truncation of the date
 * Description:
 * Reads the date / time for the given function and then converts to an ISO representation.
 * 
 ********************************************************************************************/
bool RtccMcp79410::readClock(char* buffer, size_t len, timeFunc_t function) const {
	// YYYY-MM-DDTHH:MM:SS
	// 12345678901234567890
	if (buffer == NULL || len < 20)
		return false;
	struct tm tm;
	if (!readClock(tm, function))
		return false;
	int n = isotime(tm, buffer, len);
	return size_t(n) < len; // If n >= len the string was truncated
}

/********************************************************************************************
 * bool setClock(const struct tm *tm, timeFunc_t function)
 * 
 * Scope: public
 * Parameters:
 * @param tm:		the date / time to set on the device
 * @param function:	the type of time to read (timekeeping, alarm0/1, Power fail) 
 * Returns: 
 * @return success/fail: an indiction of successful setting
 * Description:
 * Sets a new value in the time register determined by the function.  If it is the timekeeping
 * register, then the clock will be stopped and then restarted.
 * 
 ********************************************************************************************/
bool RtccMcp79410::setClock(const struct tm *tm, timeFunc_t function) const {
	// Find which register to read from
	uint8_t registerSize = 0;
	uint8_t timeRegister = getRegister(function, registerSize);

	if (registerSize == 0)
		return false; // Function not supported

	uint8_t osconEtc = 0;

	if (function == TIME) {
        stopClock();
        // Preserve OSCON, VBAT, VBATEN on MCP79410
        osconEtc = readData((uint8_t)0x03) & 0x38;

        Wire.beginTransmission(address);
        Wire.write(timeRegister);
        // Now ready to write seconds
	}

	// Wire.beginTransmission(address);
	// Wire.write(reg);
	Wire.write(decToBcd(tm->tm_sec));

	Wire.write(decToBcd(tm->tm_min));
	Wire.write(decToBcd(tm->tm_hour)); // Forces 24h mode

	Wire.write(decToBcd(tm->tm_wday + 1) | osconEtc); // Clock uses [1..7]
	Wire.write(decToBcd(tm->tm_mday));

	// Leap year bit on MCP79410 is read-only so ignore it
	Wire.write(decToBcd(tm->tm_mon + 1));

	if (registerSize >= 7) // Timekeeping register only
		Wire.write(decToBcd(tm->tm_year % 100));

	Wire.endTransmission();

	if (function == TIME)
		// startClock(decToBcd(tm->tm_sec));
		startClock();
	return true;
}

/********************************************************************************************
 * bool setClock(const char* s, timeFunc_t function)
 * 
 * Scope: public
 * Parameters:
 * @param buffer:	the date / time to set on the device in a string format
 * @param function:	the type of time to read (timekeeping, alarm0/1, Power fail) 
 * Returns: 
 * @return success/fail: an indiction of successful setting
 * Description:
 * Parses the timestamp into a date/time structure and sets the time.
 * 
 ********************************************************************************************/
bool RtccMcp79410::setClock(const char* ISOTimestamp, timeFunc_t function) const {
	if (ISOTimestamp == NULL || strlen(ISOTimestamp) < 19)
		return false;
	struct tm tm;
	tm.tm_year = atoi(ISOTimestamp) - rtccCentury;
	ISOTimestamp += 5;
	tm.tm_mon = atoi(ISOTimestamp) - 1;
	ISOTimestamp += 3;
	tm.tm_mday = atoi(ISOTimestamp);
	ISOTimestamp += 3;
	tm.tm_hour = atoi(ISOTimestamp);
	ISOTimestamp += 3;
	tm.tm_min = atoi(ISOTimestamp);
	ISOTimestamp += 3;
	tm.tm_sec = atoi(ISOTimestamp);
	mktime(&tm);
	return setClock(&tm, function);
}

/********************************************************************************************
 * bool adjustClock(RtccMcp79410::time_t offset)
 * 
 * Scope: public
 * Parameters:
 * @param offset: the offset to be applied to the current time, in seconds.
 * Returns: 
 * @return success/fail: an indiction of successful setting
 * Description:
 * Obtains the current time, converts to seconds and applies the offset before saving.
 * 
 ********************************************************************************************/
bool RtccMcp79410::adjustClock(RtccMcp79410::time_t offset) const {
	struct tm tm;
	if (!readClock(&tm))
		return false;
	time_t now = mktime(&tm);
	now -= offset;
	gmtime_r(&now, &tm);
	return setClock(&tm);
}

/********************************************************************************************
 * bool setSquareWave(freq_t frequency)
 * 
 * Scope: public
 * Parameters:
 * @param freqency: the freqency of the square wave to emit.
 * Returns: 
 * @return success/fail: an indiction of successful setting
 * Description:
 * Starts a square wave output on the MFP at the requested frequency.
 * 
 ********************************************************************************************/
bool RtccMcp79410::setSquareWave(freq_t frequency) const {
    if (frequency <= freqCalibration) {
        uint8_t ctrl = readData(0x07) & uint8_t(0xf8);
        ctrl |= frequency;
        ctrl |=0x40; // Enable square wave
        writeData(0x07, ctrl);
        return true;
    }
    return false;
}

/********************************************************************************************
 * void enableBatteryBackup(bool enable)
 * 
 * Scope: public
 * Parameters:
 * @param enable: enable or disable battery backup.
 * Returns: None

 * Description:
 * Enables or disables the battery backup.  Clock must be stopped and then re-started.
 * Battery enable bit is in the RTCWKDAY register, bit 3.
 * 
 ********************************************************************************************/
void RtccMcp79410::enableBatteryBackup(bool enable) const {
    // Writing to register 0x03 will clear the power-fail flag and
    // zero the power fail and power restored timestamps. Only
    // actually enable the bit if it is not already set.
    if (bool(readData(0x03) & 0x08) == enable)
        // State matches that requested
        return;

    stopClock();
    uint8_t rtcWkDay = readData(0x03);
    if (enable)
        rtcWkDay |= 0x08;
    else
        rtcWkDay &= 0xf7;
    writeData((uint8_t)0x03, rtcWkDay);
    startClock();
}

/********************************************************************************************
 * bool getPowerFailFlag()
 * 
 * Scope: public
 * Parameters: None
 * Returns: 
 * @return flag set or cleared
 * 
 * Description:
 * Reads the power fail flag from the RTCWKDAY register, bit 4.
 * 
 ********************************************************************************************/
bool RtccMcp79410::getPowerFailFlag() const {
	return readData(0x03) & 0x10;
}

/********************************************************************************************
 * void clearPowerFailFlag()
 * 
 * Scope: public
 * Parameters: None
 * Returns: None
 * 
 * Description:
 * Clears the power fail flag.  This is stored in the RTCWKDAY register, bit 4.  The clock
 * must be stopped and then restarted.
 * 
 ********************************************************************************************/
void RtccMcp79410::clearPowerFailFlag() const {
    stopClock();
    uint8_t rtcWkDay = readData((uint8_t)0x03);
    rtcWkDay &= 0xef;
    writeData((uint8_t)0x03, rtcWkDay);
    startClock();
}

/********************************************************************************************
 * int8_t getCalibration()
 * 
 * Scope: public
 * Parameters: None
 * Returns: 
 * @return the current calibration value
 * 
 * Description:
 * Returns the currently set calibration value: ranges -127 to +127.  This is stored in
 * register OSCTRIM.
 * 
 ********************************************************************************************/
int8_t RtccMcp79410::getCalibration() const {
    // Convert from signed magnitude to two's complement.
    uint8_t oscTrim = readData(0x08);
    int8_t r = oscTrim & 0x7Fu;
    return ((oscTrim & 0x80u) ? -r : r);
}

/********************************************************************************************
 * bool setCalibration(int8_t newValue)
 * 
 * Scope: public
 * Parameters:
 * @param newValue: the new calibration value, range -127 to +127
 * Returns: 
 * @return an indication that the value was set
 * 
 * Description:
 * Sets a new calibration value: ranges -127 to +127.  The value is constrained to this range.
 * This is stored in register OSCTRIM.
 * 
 ********************************************************************************************/
bool RtccMcp79410::setCalibration(int8_t newValue) const {
    // Convert two's complement to signed magnitude.
    if (newValue == -128)
        newValue = -127; // Out of range, use next best value.
    uint8_t oscTrim;
    if (newValue < 0)
        oscTrim = 0x80u | (uint8_t)(-newValue);
    else
        oscTrim = newValue;
    writeData(0x08, oscTrim);
    return true;
}

/********************************************************************************************
 * uint8_t bcdToDec(uint8_t bcdValue)
 * 
 * Scope: private
 * Parameters:
 * @param bcdValue: the BCD coded decimal to convert
 * Returns: 
 * @return the decimal representation
 * 
 * Description:
 * Converts a given BCD represented decimal into its actual decimal value.
 * 
 ********************************************************************************************/
uint8_t RtccMcp79410::bcdToDec(uint8_t bcdValue) {
	return ( ((bcdValue >> 4)*10) + (bcdValue%16) );
}

/********************************************************************************************
 * uint8_t decToBcd(uint8_t decimalValue)
 * 
 * Scope: private
 * Parameters:
 * @param decimalValue: the decimal value to encode as BCD
 * Returns: 
 * @return the BCD representation
 * 
 * Description:
 * Converts a given decimal value into its BCD representation.
 * 
 ********************************************************************************************/
uint8_t RtccMcp79410::decToBcd(uint8_t decimalValue) {
	return ( ((decimalValue/10) << 4) + (decimalValue%10) );
}

/********************************************************************************************
 * uint8_t readData(uint8_t rtccRegister)
 * Scope: private
 * Parameters:
 * @param rtccRegister: the register to read from the RTCC
 * Returns: 
 * @return the byte read
 * 
 * Description:
 * Reads the given register from the RTCC.
 * 
 ********************************************************************************************/
uint8_t RtccMcp79410::readData(uint8_t rtccRegister) const {
	Wire.beginTransmission(address);
	Wire.write(rtccRegister);
	Wire.endTransmission();
	Wire.requestFrom(address, uint8_t(1));
	uint8_t value = Wire.read();
//	Wire.endTransmission();
	return value;
}

/********************************************************************************************
 * void writeData(uint8_t rtccRegister, uint8_t value)
 * Scope: private
 * Parameters:
 * @param rtccRegister: the RTCC register to write into.
 * @param value:		the value to write.
 * 
 * Returns: 
 * @return an indication of successful writing
 * 
 * Description:
 * Writes the given value into the given RTCC register.  Defers to the multiple byte write.
 * 
 ********************************************************************************************/
bool RtccMcp79410::writeData(uint8_t rtccRegister, uint8_t value) const {

	return writeData(rtccRegister, &value, 1);
}

/********************************************************************************************
 * void writeData(uint8_t rtccRegister, uint8_t *values, uint8_t len)
 * Scope: private
 * Parameters:
 * @param rtccRegister: the RTCC register to start the write into.
 * @param value:		the value to write.
 * 
 * Returns:
 * @return an indication of successful writing
 * Description:
 * Writes the given values starting at the given RTCC register.
 * 
 ********************************************************************************************/
bool RtccMcp79410::writeData(uint8_t rtccRegister, uint8_t *values, uint8_t len) const {

	if((rtccRegister + len -1) > addressTop) 
		return false;

	Wire.beginTransmission(address);
	Wire.write(rtccRegister);
	for (uint8_t i=0; i < len; i++)
		Wire.write(values[i]);
	Wire.endTransmission();
	return true;
}
/********************************************************************************************
 * uint8_t getRegister(timeFunc_t function, uint8_t &registerSize)
 * 
 * Scope: private
 * Parameters:
 * @param function: 	the function to get the correct root address for.
 * @param registerSize:	set to be the size (number of bytes) that are held for the register.
 * 
 * Returns: 
 * @return the base address of the register that maps to the given function
 * 
 * Description:
 * For a given function, returns the root address of the register map - see table 5.1, page 13.
 * Also, the registerSize parameter is set to give the number of bytes available for that 
 * register.
 * 
 ********************************************************************************************/
uint8_t RtccMcp79410::getRegister(timeFunc_t function, uint8_t &registerSize) const {
	const uint8_t registerTable[5] = {0, 0x0a, 0x11, 0x18, 0x1C};
	const uint8_t sizeTable[5] = {7, 6, 6, 4, 4};
	registerSize = sizeTable[function];
	return registerTable[function];
}

/********************************************************************************************
 * bool readPowerTime(struct tm *tm, uint8_t powerFailRegister, uint8_t registerSize)
 * 
 * Scope: private
 * Parameters:
 * @param tm: 					the date/time structure to load with the Power-Fail timestamp.
 * @param powerFailRegister:	set to be the size (number of bytes) that are held for the register.
 * @param registerSize:			the number of bytes in the register.
 * 
 * Returns:
 * @return an indicator of successful read.
 * 
 * Description:
 * Reads the Power-Fail timestamp register, either Power-Down or Power-Up, returning as a 
 * Date/Time structure.
 * 
 ********************************************************************************************/
bool RtccMcp79410::readPowerTime(struct tm *tm, uint8_t powerFailRegister, uint8_t registerSize) const {
	// Reset the register pointer
	Wire.beginTransmission(address);
	Wire.write(powerFailRegister);
	Wire.endTransmission();

	Wire.requestFrom(address, registerSize);
	tm->tm_sec = 0;
	tm->tm_min = bcdToDec(Wire.read() & 0x7f);
	tm->tm_hour = bcdToDec(Wire.read() & 0x3f);
	tm->tm_wday = 0;
	tm->tm_mday = bcdToDec(Wire.read() & 0x3f);
	uint8_t wdayMonth = Wire.read();
	tm->tm_mon = bcdToDec(wdayMonth & 0x1f) - 1; // Clock uses [1..12]
	tm->tm_wday = (wdayMonth >> 5) - 1; // Clock uses [1..7]
	tm->tm_year = (rtccYearsSinceCenturyStart); // not stored
	tm->tm_yday = -1;
//	Wire.endTransmission();
	return true;
}

/********************************************************************************************
 * int isotime(const struct tm *tm, char *buffer, size_t len)
 * 
 * Scope: public
 * Parameters:
 * @param tm: 		the date/time structure to conert to ISO format.
 * @param buffer:	the ISO formatted date/time.
 * @param len:		the length of the buffer.
 * 
 * Returns:
 * @return an indicator of successful conversion: -1 if encoding fails; a number of characters
 * that would have been encoded if the buffer was long enough.
 * 
 * Description:
 * Converts the given date/time structure into an ISO formatted string.
 * 
 ********************************************************************************************/
int RtccMcp79410::isotime(const struct tm *tm, char *buffer, size_t len) {
	return SNPRINTF(buffer, len, PSTR("%04d-%02d-%02dT%02d:%02d:%02d"),
					  tm->tm_year + rtccCentury,
					  tm->tm_mon + 1,
					  tm->tm_mday,
					  tm->tm_hour,
					  tm->tm_min,
					  tm->tm_sec);
}

/********************************************************************************************
 * bool isRunning()
 * 
 * Scope: public
 * Parameters: None
 * 
 * Returns: 
 * @return an indication that the clock (oscillator) is running.
 * 
 * Description:
 * Checks the Oscillator status bit 7 in RTCSEC register.
 * 
 ********************************************************************************************/
bool RtccMcp79410::isRunning() const {
	uint8_t rtcsec = 0x00;
	uint8_t secondsByte = readData(rtcsec);
	return secondsByte &= 0x80;
}

/********************************************************************************************
 * void setMFPRestingLevel(MFPLevel level)
 * 
 * Scope: public
 * Parameters: 
 * @param level: Set to low or high as desired.
 * 
 * Returns: None
 * 
 * Description:
 * Sets the MFP pin output level.  This is bit 7 in the CONTROL register.
 * 
 ********************************************************************************************/
void RtccMcp79410::setMFPRestingLevel(MFPLevel level) const {
    uint8_t control = readData((uint8_t)0x07);
	if(level) 
    	control |= 0x80;
	else
    	control &= 0x7f;
    writeData((uint8_t)0x07, control);
}

/********************************************************************************************
 * void setAlarm(const struct tm *tm, timeFunc_t alarmNumber)
 * 
 * Scope: public
 * Parameters: 
 * @param tm: Date/Time structure containing the alarm time.
 * @param alarmNumber: the Time Function for the required alarm (alarm0 or alarm1)
 * 
 * Returns: 
 * @return an indication that the alarm was set successfully
 * 
 * Description:
 * Helper method for alarms - defers to setClock() with the correct function.
 * 
 ********************************************************************************************/
bool RtccMcp79410::setAlarm(const struct tm *tm, timeFunc_t alarmNumber) const {
	if ((alarmNumber == ALARM0) || (alarmNumber == ALARM1))
		return setClock(tm, alarmNumber);
	return false;
}

/********************************************************************************************
 * void readAlarm(const struct tm *tm, timeFunc_t alarmNumber)
 * 
 * Scope: public
 * Parameters: 
 * @param tm: Date/Time structure containing the alarm time.
 * @param alarmNumber: the Time Function for the required alarm (alarm0 or alarm1)
 * 
 * Returns: 
 * @return an indication that the alarm was read successfully
 * 
 * Description:
 * Helper method for alarms - defers to readClock() with the correct function.
 * 
 ********************************************************************************************/
bool RtccMcp79410::readAlarm(struct tm *tm, timeFunc_t alarmNumber) const {
	if ((alarmNumber == ALARM0) || (alarmNumber == ALARM1))
		return readClock(tm, alarmNumber);
	return false;
}

/********************************************************************************************
 * void enableAlarm(timeFunc_t alarmNumber, uint8_t alarmType)
 * 
 * Scope: public
 * Parameters: 
 * @param alarmNumber: the Time Function for the required alarm (alarm0 or alarm1)
 * @param alarmType: The type of alarm to set or disable.
 * 
 * Returns: 
 * @return an indication that the alarm was enabled successfully
 * 
 * Description:
 * Enables the requested alarm to trigger on the given criteria.  The enable bits are 4 and 5
 * in the Control register; the ALMxWKDAY register contains the alarm type (mask) bits 4, 5 and 6 
 * 
 ********************************************************************************************/
bool RtccMcp79410::enableAlarm(timeFunc_t alarmNumber, alarmTypes alarmType) const {
	uint8_t alarmAddress;
	if (alarmNumber == ALARM0)
		alarmAddress = 0x0D;
	else if (alarmNumber == ALARM1)
		alarmAddress = 0x14;
	else
		return false; // invalid alarmNumber passed

	uint8_t control = readData((uint8_t)0x07);
	if (alarmType < almDisable) {
		uint8_t almwkday = readData(alarmAddress);
		// Reset the interrupt flag and add the alarm type to mask bits
		almwkday = (almwkday & 0x87) | alarmType << 4;
		writeData(alarmAddress, almwkday);
		control |= ((uint8_t)alarmNumber << 4);
	} else {
		control |= ~((uint8_t)alarmNumber << 4);  // Disabled
	}
	writeData((uint8_t)0x07, control);
	return true;
}

/********************************************************************************************
 * bool isAlarmRaised(timeFunc_t alarmNumber)
 * 
 * Scope: public
 * Parameters: 
 * @param alarmNumber: the Time Function for the required alarm (alarm0 or alarm1)
 * 
 * Returns: 
 * @return Whether or not the alarm has been raised.
 * 
 * Description:
 * The device will set the interrupt flag (not actually an interrupt) for an alarm that is
 * triggered.  This is bit 3 in the ALMxWKDAY register.  If this has been set, then reset it.
 * 
 ********************************************************************************************/
bool RtccMcp79410::isAlarmRaised(timeFunc_t alarmNumber) const {
	uint8_t alarmAddress;
	if (alarmNumber == ALARM0)
		alarmAddress = 0x0D;
	else if (alarmNumber == ALARM1)
		alarmAddress = 0x14;
	else
		return false; // invalid alarmNumber passed
	
	uint8_t almwkday = readData(alarmAddress);
	if (almwkday & 0x10) {
		almwkday &= ~(uint8_t)0x10;
		writeData(alarmAddress, almwkday);
		return true;
	}
	return false;
}

/********************************************************************************************
 * bool alarmPolarity(bool polarity)
 * 
 * Scope: public
 * Parameters: 
 * @param polarity: whether the MFP pin should go low or high when alarm(s) triggered.
 * 
 * Returns: None
 * 
 * Description:
 * Specifies the logic level on the MFP pin when an alarm is triggered.  The default is LOW.
 * When both alarms are active, the two are ORd together to determine the level of the MFP.  
 * With alarm priority set to LOW, this causes the MFP to go low only when BOTH alarms are 
 * triggered.  With alarm priority to HIGH, the MFP pin will go high when either alarm is
 * triggered.
 * 
 * The MFP pin and the Alarm interrupt flag are independent of each other.  The polarit is
 * bit 7 in ALM0WKDAY register (for both alarms.)
 * 
 ********************************************************************************************/
void RtccMcp79410::alarmPolarity(MFPLevel polarity) const {
	uint8_t alm0wkday = readData((uint8_t)0x0D);

	if (polarity)
		alm0wkday |= 0x80;
	else 
		alm0wkday &= ~0x80;
	writeData((uint8_t)0x0D, alm0wkday);
}

/********************************************************************************************
 * bool eepromWrite(uint8_t address, uint8_t value)
 * 
 * Scope: public
 * Parameters: 
 * @param address:	the address into which to write the value.
 * @param value:	the value to write into the eeprom.
 * 
 * Returns: 
 * @return an indicator of successful writing to EEPROM
 * 
 * Description:
 * Writes the given value into the EEPROM at the given address.  Defers to the page write
 * implementation.
 * 
 ********************************************************************************************/
bool RtccMcp79410::eepromWrite(uint8_t address, uint8_t value) const {

	return eepromWrite(address, &value, 1);
}

/********************************************************************************************
 * bool eepromWrite(uint8_t address, uint8_t *values, uint8_t len)
 * 
 * Scope: public
 * Parameters: 
 * @param address:	the address into which to write the value.
 * @param values:	an array of values to write into the eeprom.
 * @param len:		the number of values to write
 * 
 * Returns: 
 * @return an indicator of successful writing to EEPROM
 * 
 * Description:
 * Writes a number of bytes into the EEPROM starting at the given address.  This implementation
 * does not allow wrap-around - i.e. trying to write bytes beyond the 8-byte page boundary.  If
 * this would occur, nothing is written and the write fails.
 * 
 ********************************************************************************************/
bool RtccMcp79410::eepromWrite(uint8_t address, uint8_t *values, uint8_t len) const {

	// For the address, calculate the upper limit of the page.  E.g. writing 4 bytes starting 
	// at address 9 gives an upper address of (1 + 1) * 8 - 1 = 15.  Page boundary is address
	// 8 to 15.
	uint8_t upperAddress = ((((uint8_t)(address / 8)) + 1) * 8) - 1; 

	// Bytes can not be written beyond the page boundary otherwise wrap-around occurs.  E.g.
	// following on 9 + 4 - 1 = 12 which is less than 15 so ok.
	if ((address + len - 1) > (upperAddress - 1 - len)) 
		return false; // requesting an address beyond that allowed
	
	Wire.beginTransmission(eepromAddress);
	Wire.write(address);
	for (uint8_t i=0; i < len; i++)
		Wire.write(values[i]);
	Wire.endTransmission();
	return waitUntilFree();
}

/********************************************************************************************
 * uint8_t eepromRead(uint8_t address)
 * 
 * Scope: public
 * Parameters: 
 * @param address:	the address from which to read the value.
 * @param value:	the variable in which to store the read byte.
 * 
 * Returns: 
 * @return an indicator that a byte was read successfully.
 * 
 * Description:
 * Reads a byte from EEPROM at the given address. Defers to the multiple bytes 
 * implementation
 * 
 ********************************************************************************************/
bool RtccMcp79410::eepromRead(uint8_t address, uint8_t *value) const {
	return eepromRead(address, value, 1);
}

/********************************************************************************************
 * bool eepromRead(uint8_t address, uint8_t *values, uint8_t len)
 * 
 * Scope: public
 * Parameters: 
 * @param address:	the address from which to read the value.
 * @param values:	an array into which the read bytes are loaded.
 * @param len:		the number of values to read.
 * 
 * Returns: 
 * @return an indicator that bytes were read successfully.
 * 
 * Description:
 * Reads a number of bytes from the given address and loads these into the Values array.
 * This implementation does not allow wrap-around - i.e. trying to read bytes beyond the 8-byte 
 * page boundary.  If this would occur, nothing is read and the read fails.
 ********************************************************************************************/
bool RtccMcp79410::eepromRead(uint8_t address, uint8_t *values, uint8_t len) const {
	uint8_t upperAddress = ((((uint8_t)(address / 8)) + 1) * 8) - 1; 

	// Bytes can not be read beyond the page boundary otherwise wrap-around occurs.  
	if ((address + len - 1) > (upperAddress - 1 - len)) 
		return false; // requesting an address beyond that allowed
	
	Wire.beginTransmission(eepromAddress);
	Wire.write(address);
	Wire.endTransmission();

	Wire.requestFrom(eepromAddress, len);
	for (uint8_t i=0; i < len; i++)
		values[i] =Wire.read();
	return true;
}

/********************************************************************************************
 * bool waitUnitlFree()
 * 
 * Scope: private
 * Parameters: None 
 *
 * Returns: 
 * @returns true if the EEPROM became free, false if it timedout
 * 
 * Description:
 * Waits for 2 seconds for the EEPROM to finish its current write before timing out.
 ********************************************************************************************/
bool RtccMcp79410::waitUntilFree() const {

	uint8_t busy {4};
    uint32_t timeStarted = millis();

	while (busy && (millis() - timeStarted < 2000) ) {
		Wire.beginTransmission(eepromAddress);
		Wire.write((uint8_t)0);
		busy = Wire.endTransmission();
		if (millis() < timeStarted) { // millis overflowed
			// reset timestarted, taking account of time elapsed so far.
			timeStarted = (UINT32_MAX - timeStarted) + millis();
		}
    }

    if (busy) // busy == 1..4
		return false; 
	else // busy == 0
		return true; 
}

/********************************************************************************************
 * bool sramWrite(uint8_t address, uint8_t value)
 * 
 * Scope: public
 * Parameters: 
 * @param address:	the address into which to write the value.
 * @param value:	the value to write into sram.
 * 
 * Returns: 
 * @return an indicator of successful writing to sram
 * 
 * Description:
 * Writes the given value into sram at the given address.  Defers to the multiple write
 * implementation.
 * 
 ********************************************************************************************/
bool RtccMcp79410::sramWrite(uint8_t address, uint8_t value) const {
	return sramWrite(address, &value, 1);
}

/********************************************************************************************
 * bool sramWrite(uint8_t address, uint8_t *values, len)
 * 
 * Scope: public
 * Parameters: 
 * @param address:	the address into which to write the value.
 * @param values:	the values to write into sram starting at address.
 * @param len:		the number of bytes to write.
 * 
 * Returns: 
 * @return an indicator of successful writing to sram
 * 
 * Description:
 * Writes the given values into sram starting at the given address.  The data must lie within
 * the SRAM memory block.
 * 
 ********************************************************************************************/
bool RtccMcp79410::sramWrite(uint8_t address, uint8_t *values, uint8_t len) const {
	if (address < sramStartAddress || (address + len -1) > addressTop)
		return false;

	return writeData(address, values, len);
}

/********************************************************************************************
 * bool sramRead(uint8_t address, uint8_t *value)
 * 
 * Scope: public
 * Parameters: 
 * @param address:	the address from which to read the value.
 * @param value:	the value read from sram.
 * 
 * Returns: 
 * @return an indicator of successful reading of sram
 * 
 * Description:
 * Reads a value from sram from the given address.  Defers to the multiple read
 * implementation.
 * 
 ********************************************************************************************/
bool RtccMcp79410::sramRead(uint8_t address, uint8_t *value) const {
	return sramRead(address, value, 1);
}

/********************************************************************************************
 * bool sramRead(uint8_t address, uint8_t *values, uint8_t len)
 * 
 * Scope: public
 * Parameters: 
 * @param address:	the address from which to read the value.
 * @param value:	the value read from sram.
 * @param len:		the number of bytes to read.
 * 
 * Returns: 
 * @return an indicator of successful reading of sram
 * 
 * Description:
 * Reads a number of values from sram starting at the given address.  Reading must occur within
 * the SRAM memory block.
 * 
 ********************************************************************************************/
bool RtccMcp79410::sramRead(uint8_t address, uint8_t *values, uint8_t len) const {

	if (address < sramStartAddress || (address + len -1) > addressTop)
		return false;

	for (uint8_t i=0; i<len; i++)
		values[i] = readData(address + i);

	return true;
}
