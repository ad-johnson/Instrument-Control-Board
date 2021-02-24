/*******************************************************************************************
 * 
 * EepromCAT24M01: Class to interact with a CAT24M01 EEPROM (or any *24M01 EEPROM.)
 * Created for Arduino using PlatformIO and C++17 support.
 * 
 * Author: Andrew Johnson
 * 
 * Attribution: https://github.com/CascoLogix/CAT24M01  This solution is based heavily on the 
 * work done by CascoLogix.
 * 
 * Datasheet: https://www.onsemi.com/pub/Collateral/CAT24M01-D.PDF
 * February 2021
 * 
 *******************************************************************************************/
#include "Arduino.h"
#include <Wire.h>
#include "EepromCAT24M01.h"

//#define DEBUG_EEPROMCAT24M01			// Uncomment to get serial UART debug strings

/********************************************************************************************
 * Constructor
 * 
 * Scope: public
 * Parameters: 
 * @param deviceAddress:    This is the I2C bus address of the device; will use the default if not 
 *                          provided.
 * @param shouldInitialise: If true, then the instance will join the I2C bus as a master.
 * Returns: None
 * 
 * Description:
 * Creates an instance of the EepromCAT24M01 class and creates the root I2C address for all
 * methods that need it.  If requested, the class initialises the I2C bus and joins as a 
 * master.  Typically, this class is intended to be used as part of a larger application that
 * will manage the I2C bus so no initialisation will be required.
 * 
 ********************************************************************************************/
EepromCAT24M01::EepromCAT24M01(uint8_t deviceAddress)
{
	buildRootAddress(deviceAddress);
 }

/***********************************************************************************
 * 
 * void setTimeoutInMillis(uint16_t newTimeout)
 * 
 * Scope: public
 * 
 * Parameters:
 * @param newTimeout: The new timout value in milliseconds.
 *
 * Returns: None
 * 
 * Description:
 * 		Sets a new timeout value for interacting with the EEPROM.  Before a read or
 *      write operation, the operation will wait until the EEPROM finishes any 
 *      previous operation OR a timout occurs.  The default is 2000 millis set when
 *      an instance is obtained.
 * 
 **********************************************************************************/
void EepromCAT24M01::setTimeoutInMillis(uint16_t newTimeout) {
    timeout = newTimeout;
}				

/***********************************************************************************
 * 
 * uint8_t write(uint16_t pageNumber, uint8_t byteNumber, uint8_t data)
 * 
 * Scope: public
 * Parameters:
 * @param pageNumber:   This is the internal EEPROM page to be read from, ( 0 - 511.)
 * @param byteNumber:   This is the internal EEPROM byte to be read, (0 - 255.) 
 * @param data:         This is the data being sent to the EEPROM to be written at the 
 *                      Page / Byte address
 *
 * Returns:
 * @return write status: Returns 1 if the byte was written, or 0 if there was an error
 * 
 * Description:
 * Converts the passed byte to a buffer and proceeds as if a multi-byte write was
 * requested.
 * 
 **********************************************************************************/
uint8_t EepromCAT24M01::write(uint16_t pageNumber, uint8_t byteNumber, uint8_t data) {

    uint8_t buffer[] {data};

    return write(pageNumber, byteNumber, buffer, 1);
}

/***********************************************************************************
 * 
 * uint8_t write(uint16_t pageNumber, uint8_t startByteNumber, uint8_t* buffer, uint8_t numberOfBytes)
 * 
 * Scope: public
 * Parameters:
 * @param pageNumber:       This is the internal EEPROM page to be read from, ( 0 - 511.)
 * @param startByteNumber:  This is the internal EEPROM byte to be read, (0 - 255.) 
 * @param buffer:           Data read is placed in this buffer
 *
 * Returns:
 * @return	Bytes read: the number of bytes read, or 0 if there was an error
 * 
 * Description:
 * The pageNumber and byteNumber are converted into a memory address for the EEPROM,
 * with the most significant bit of pageNumber written into A16 bit of the slave 
 * address.  The bytes stored in the buffer are written and the total written is returned.
 * 
 * Written bytes:
 * <slave address><page address><byte address><data byte><data byte>...<last data byte><stop byte>
 * 
 **********************************************************************************/
uint8_t EepromCAT24M01::write(uint16_t pageNumber, uint8_t startByteNumber, uint8_t* buffer, uint8_t numberOfBytes) {

	uint8_t returnValue {0};
	uint8_t bytesWritten {0};

    // wait for the EEPROM to become free before continuing.  Return error if timeout occurs.
    if (waitUntilFree()) { 
    #ifdef DEBUG_EEPROMCAT24M01
        Serial.println("EEPROM busy and timeout occurred whilst waiting.");
    #endif // DEBUG_EEPROMCAT24M01
        return 0;
    }

	// A16 is updated from bit 9 of the Page Number to form the slave address
    Wire.beginTransmission(buildI2CAddress(pageNumber));
    Wire.write((uint8_t)pageNumber); // write bits 0 to 7 of the page number
    Wire.write(startByteNumber);

#ifdef DEBUG_EEPROMCAT24M01
    if (startByteNumber + numberOfBytes > pageSize) {
        Serial.println("Writing past the last available byte in the page so wraparound will occur.");
        Serial.print(pageSize - (startByteNumber +numberOfBytes));
        Serial.println(" bytes will be overwritten.");
    }
#endif // DEBUG_EEPROMCAT24M01

	bytesWritten = Wire.write(buffer, numberOfBytes);
    returnValue = Wire.endTransmission();
	
    if (returnValue) {
#ifdef DEBUG_EEPROMCAT24M01
        Serial.print("Write error occurred: ");
        Serial.println(returnValue);
#endif // DEBUG_EEPROMCAT24M01
    }
	
	return bytesWritten;
}

/***********************************************************************************
 * 
 * uint8_t read(uint32_t memAddress, uint8_t* buffer)
 * 
 * Scope: public
 * 
 * Parameters:
 * @param pageNumber:   This is the internal EEPROM page to be read from, ( 0 - 511.)
 * @param byteNumber:   This is the internal EEPROM byte to be read, (0 - 255.) 
 * @param buffer:       Data read is placed in this buffer
 *
 * Returns:
 * @return	Bytes read: the number of bytes read, or 0 if there was an error
 * 
 * Description:
 * 		This method defers to the multi-byte read.
 * 
 **********************************************************************************/
uint8_t EepromCAT24M01::read(uint16_t pageNumber, uint8_t byteNumber, uint8_t* buffer) {

    return read(pageNumber, byteNumber, buffer, 1);
}


/***********************************************************************************
 * 
 * uint8_t read(uint32_t memAddress, uint8_t* buffer, uint8_t numberOfBytes)
 * 
 * Scope: public
 * 
 * Parameters:
 * @param pageNumber:   This is the internal EEPROM page to be read from, ( 0 - 511.)
 * @param byteNumber:   This is the internal EEPROM byte to be read, (0 - 255.) 
 * @param buffer:       Data read is placed in this buffer
 * @param numBytes:     This is the number of bytes to read
 *
 * Returns:
 * @return	Bytes read: the number of bytes read, or 0 if there was an error
 * 
 * Description:
 *      The pageNumber and byteNumber are converted into a memory address for the EEPROM,
 *      with the most significant bit of pageNumber written into A16 bit of the slave 
 *      address.  Bytes are read into the Wire framework's buffer and read from there.
 * 
 * Written bytes:
 * <slave address><page address><byte address><stop byte>
 * 
 **********************************************************************************/
uint8_t EepromCAT24M01::read(uint16_t pageNumber, uint8_t startByteNumber, uint8_t* buffer, uint8_t numberOfBytes) {
	uint8_t returnValue {0};
	uint8_t I2CAddress {buildI2CAddress(pageNumber)};

    // wait for the EEPROM to become free before continuing.  Return error if timeout occurs.
    if (waitUntilFree()) { 
    #ifdef DEBUG_EEPROMCAT24M01
        Serial.println("EEPROM busy and timeout occurred whilst waiting.");
    #endif // DEBUG_EEPROMCAT24M01
        return 0;
    }

	// A16 is updated from bit 9 of the Page Number to form the slave address
    Wire.beginTransmission(I2CAddress);
    Wire.write((uint8_t)pageNumber); // write bits 0 to 7 of the page number
    Wire.write(startByteNumber);
	
	returnValue = Wire.endTransmission(0);				// Send restart condition (false)

#ifdef DEBUG_EEPROMCAT24M01
	if (returnValue) {
		Serial.print("NACK from endTransmission. Error number = ");
		Serial.println(returnValue);
	}
#endif // DEBUG_EEPROMCAT24M01

    returnValue = Wire.requestFrom(I2CAddress, numberOfBytes);
	
    uint8_t index;
	if (returnValue) {
#ifdef DEBUG_EEPROMCAT24M01
		Serial.print("Read error: ");
		Serial.println(returnValue);
#endif // DEBUG_EEPROMCAT24M01
	} else {
        for (index = 0; index < numberOfBytes; index++ ) {
            if (Wire.available()) {
                buffer[index] = Wire.read();    // Read bytes from I2C receive buffer
            }
    #ifdef DEBUG_EEPROMCAT24M01
            else {
                Serial.print("I2C read buffer empty with ");
                Serial.print(numberOfBytes - index);
                Serial.println(" bytes remaining to read.");
            }
    #endif // DEBUG_EEPROMCAT24M01
	    }
    }
	return index;   // Return number of bytes read
}
  
/***********************************************************************************
 * 
 * uint8_t getBusyStatus(uint32_t memAddress)
 * 
 * Scope: public
 * 
 * Parameters: None
 *
 * Returns:
 * @return busy status: whether or not the EEPROM is free: 1 is busy.
 * 
 * Description:
 * 		Sends a start request to the EEPROM, the memory address being irrelevant, and
 *      returns the returned code.
 * 
 **********************************************************************************/
uint8_t EepromCAT24M01::getBusyStatus() {
	uint8_t busyStatus {0};
	
    Wire.beginTransmission(rootAddress); // no need for an A16 bit.
	busyStatus = Wire.endTransmission();

	return busyStatus;									// Return busy status
}

/***********************************************************************************
 * 
 * void buildRootAddress(uint8_t deviceAddress)
 * 
 * Scope: private
 * Parameters:
 * @param deviceAddress:    This is the unique address of the device on the bus, determined
 *                          by its A1 and A2 pins, so 00, 01, 10, 11.  Only 4 devices are
 *                          supported.
 *
 * Returns: None
 * 
 * Description:
 * An actual address consists of 8 bits:
 * - 1010 (base address)
 * - deviceAddress, 2 bits
 * - A16, 1 bit: MSB of the page to be written
 * - RW, 1 bit: read / write bit
 * A16 and the RW bit are added at the time of actually reading/writing data so the 
 * root address is just the base address and device address, e.g: 1010xx00 where xx
 * is the device address.  In actual fact, the read/write bit is added by the Wire
 * framework so the root address is stored as 01010xx0
 * 
 **********************************************************************************/
void EepromCAT24M01::buildRootAddress(uint8_t deviceAddress) {
#ifdef DEBUG_EEPROMCAT24M01
    if (deviceAddress > 3) {
        Serial.print("Device Address error: ");
        Serial.println(deviceAddress);
        Serial.println("Only values 0 to 3 are valid.  Changed to 0.")
        Serial.println();
        deviceAddress = 0;
    }
#endif // DEBUG_EEPROMCAT24M01

    rootAddress = (uint8_t)((baseI2CAddress << 3) | (deviceAddress << 1));
}

/***********************************************************************************
 * 
 * uint8_t buildI2CAddress(uint16_t pageNumber)
 * 
 * Scope: private
 * Parameters:
 * @param pageNumber:   This is the page to be read/written, 0 to 511.  The 9th bit of
 *                      the page number is stored in bit A16 of the slave address and
 *                      is thus added into bit 0 of the root address.
 *
 * Returns:
 * @return I2C Address: the correct I2C address of the slave device, minus the read/
 *                      write bit.
 * 
 * Description:
 * An actual address consists of 8 bits:
 * - 1010 (base address)
 * - deviceAddress, 2 bits
 * - A16, 1 bit: MSB of the page to be written
 * - RW, 1 bit: read / write bit
 * Takes bit 9 of the page number and inserts it into the root address as bit A16.
 * The read/write bit is added by the Wire framework so the returned address will
 * be, e.g., 01010111 for device 3 (11) and page number 256.
 * 
 **********************************************************************************/
uint8_t EepromCAT24M01::buildI2CAddress(uint16_t pageNumber) {
#ifdef DEBUG_EEPROMCAT24M01
    if (pageNumber > pages) {
        Serial.print("Page Number error: ");
        Serial.println(pageNumber);
        Serial.println("Page range is 0 to 511.  EEProm wraparound will occur.")
        Serial.println();
    }
#endif // DEBUG_EEPROMCAT24M01    

    // isolate bit 9 and ensure remaining bits in the word are 0.
    return (uint8_t)(rootAddress | ((pageNumber >> 8) & 0x01));
}

/***********************************************************************************
 * 
 * uint8_t waitUntilFree()
 * 
 * Scope: private
 * Parameters: None
 *
 * Returns:
 * @return busy: 1 if busy and timedout, 0 otherwise
 * 
 * Description:
 *      Checks the EEPROM busy status until free or the timeout passes.  In the latter
 *      case, 1 will be returned.  The method only waits as long as it needs to.  NOTE:
 *  
 **********************************************************************************/
uint8_t EepromCAT24M01::waitUntilFree() {
    uint8_t busy {getBusyStatus()};
    if (busy) {
        uint32_t timeStarted = millis();
        while (busy && (millis() - timeStarted >= timeout) ) {
            busy = getBusyStatus();
            if (millis() < timeStarted) { // millis overflowed
                // reset timestarted, taking account of time elapsed so far.
                timeStarted = (UINT32_MAX - timeStarted) + millis();
            }
        }
    }

    return busy; // will be 1 if it timedout, 0 otherwise.
}
