/*******************************************************************************************
 * 
 * EepromCAT24M01:	Class to interact with a CAT24M01 EEPROM (or any *24M01 EEPROM.)
 * 					Created for Arduino using PlatformIO and C++17 support.
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

/*******************************************************************************************
 * 
 * Addressing
 * 
 * The slave address for the CAT24M01 is constructed from 4 elements and is 8-bits long:
 * 
 * Base address, 4 bits: 1010 always
 * Device address, 2 bits: up to 4 devices can be placed on the bus with address derived from
 * 							pins A1 and A2 tied to Ground or VCC: 00, 01, 10, 11
 * A16, 1 bit: Most significant bit of the memory address to be accessed, see below
 * Read/Write, 1 bit: 1 = read; 0 = Write
 * 
 * E.g: 10100001
 * Note that the Arduino Wire library will shift and add the read/write bit so the code here
 * only needs to deal with 7 most significant bits.
 * 
 * Memory Addressing
 * The CAT24M01 has 512 pages of 256 bytes each, individually addressable through a 17-bit
 * address - most significant 9-bits identify the page (0 to 511) and the least significant
 * 8-bits identify the first byte.  Note that the 17th-bit (bit 16) goes in the slave address
 * as A16.
 * 
 *******************************************************************************************/

#ifndef EEPROM_CAT24M01_H
#define EEPROM_CAT24M01_H

#include <inttypes.h>
class EepromCAT24M01 {
public:

	static constexpr	uint8_t		baseI2CAddress {0x0A}; 			// 1010 as defined in the datasheet
	static constexpr	uint8_t		defaultDeviceAddress {0x00};	// 00 - both address pins tied to ground
	static constexpr	uint8_t		maxDeviceAddress {0x03};		// in bits: 00, 01, 10, 11.
	static constexpr	uint16_t	pageSize {256};					// Number of bytes available in a page
	static constexpr	uint16_t	pages {512};					// Number of pages available in the EEPROM
	static constexpr	uint16_t	defaultTimeout {2000};			// Wait time for EEPROM to come free

	// Default constructor taking the device address on the I2C bus
	EepromCAT24M01(uint8_t deviceAddress = defaultDeviceAddress);
	
	// Write a byte to EEPROM
	uint8_t write(uint16_t pageNumber, uint8_t byteNumber, uint8_t data) const;

	// Write a buffer of bytes to EEPROM	
	uint8_t write(uint16_t pageNumber, uint8_t startByteNumber, uint8_t* buffer, uint8_t numberOfBytes) const;
					
	// Read a byte from EEPROM into a buffer
	uint8_t read(uint16_t pageNumber, uint8_t byteNumber, uint8_t& data) const;

	// Read multiple bytes from EEPROM into a buffer
	uint8_t read(uint16_t pageNumber, uint8_t byteNumber, uint8_t* buffer, uint8_t numberOfBytes) const;

	// Return the busy status of the EEPROM
	uint8_t getBusyStatus() const;

	// Set a timeout
	void setTimeoutInMillis(uint16_t newTimeout);				

private:
	uint8_t		deviceAddress {};			// Implemented Device Address on PCB
	uint8_t 	rootAddress {};				// Configured I2C address for the device but without the A16 bit
	uint16_t	timeout {defaultTimeout}; 	// How long to wait whilst EEPROM is busy before timing out

	void 	buildRootAddress(uint8_t deviceAddress); 	// Pre-build the rootAddress to speed up writes and reads
	uint8_t buildI2CAddress(uint16_t pageNumber) const;
	uint8_t waitUntilFree() const; 							// wait for the EEPROM to finish last operation, or a timout to occur
};

#endif // EEPROM_CAT24M01_H