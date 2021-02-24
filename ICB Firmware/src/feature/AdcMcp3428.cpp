/*******************************************************************************************
 * 
 * AdcMcp3428:  Class to interact with a MCP3428 ADC.
 *              Updated for Arduino using PlatformIO and C++17 support.
 * 
 * Author: Modified for MCP3428 only - 12-, 14- and 16-bit - by Andrew Johnson
 * 
 * Attribution: https://github.com/stevemarple/MCP342x  Arduino library by Steve Marple.
 * 
 * Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/22226a.pdf
 * February 2021
 * 
 *******************************************************************************************/
#include "Arduino.h"
#include "Wire.h"
#include "AdcMcp3428.h"

// #define DEBUG_ADCMCP3428 // Uncomment to get serial UART debug strings

const AdcMcp3428::Channel AdcMcp3428::channel1 = Channel(0x00);
const AdcMcp3428::Channel AdcMcp3428::channel2 = Channel(0x20);
const AdcMcp3428::Channel AdcMcp3428::channel3 = Channel(0x40);
const AdcMcp3428::Channel AdcMcp3428::channel4 = Channel(0x60);

const AdcMcp3428::Mode AdcMcp3428::oneShot = Mode(0x00);
const AdcMcp3428::Mode AdcMcp3428::continous = Mode(0x10);

const AdcMcp3428::Resolution AdcMcp3428::resolution12 = Resolution(0x00);
const AdcMcp3428::Resolution AdcMcp3428::resolution14 = Resolution(0x04);
const AdcMcp3428::Resolution AdcMcp3428::resolution16 = Resolution(0x08);

const AdcMcp3428::Gain AdcMcp3428::gain1 = Gain(0x00);
const AdcMcp3428::Gain AdcMcp3428::gain2 = Gain(0x01);
const AdcMcp3428::Gain AdcMcp3428::gain4 = Gain(0x02);
const AdcMcp3428::Gain AdcMcp3428::gain8 = Gain(0x03);

/********************************************************************************************
 * uint8_t generalCallReset()
 * 
 * Scope: public
 * Parameters: None
 * Returns: 
 * @return error code
 * 
 * Description:
 * Reset the device and sample and latch the address pins.  Typically, this should be called
 * once at startup to ensure address pins are latched.
 * 
 ********************************************************************************************/
uint8_t AdcMcp3428::generalCallReset() {
    Wire.beginTransmission(broadcastAddress);
    Wire.write(resetCode);
    return Wire.endTransmission();
}
    
/********************************************************************************************
 * uint8_t generalCallLatch()
 * 
 * Scope: public
 * Parameters: None
 * Returns: 
 * @return error code
 * 
 * Description:
 * Sample and latch the address pins, but no device reset.
 * 
 ********************************************************************************************/
uint8_t AdcMcp3428::generalCallLatch() {
    Wire.beginTransmission(broadcastAddress);
    Wire.write(latchCode);
    return Wire.endTransmission();
}
				
/********************************************************************************************
 * Constructor
 * 
 * Scope: public
 * Parameters: 
 * @param deviceAddress:    This is the I2C bus address of the device; will use the default if not 
 *                          provided.
 * Returns: None
 * 
 * Description:
 * Creates an instance of the AdcMCP3428 class and creates the I2C address for all
 * methods that need it. 
 * 
 ********************************************************************************************/
AdcMcp3428::AdcMcp3428(uint8_t deviceAddress) {
	  buildI2CAddress(deviceAddress);
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
bool AdcMcp3428::isConnected() {
    Wire.requestFrom(address, (uint8_t)1);
    if (!Wire.available()) {
    #ifdef DEBUG_ADCMCP3428
        Serial.print("No ADC MCP3428 device found at address ");
        Serial.println(address, HEX);
    #endif // DEBUG_ADCMCP3428
        return false;
    }
    return true;
}

/********************************************************************************************
 * error_t convert(Channel channel, Mode mode, Resolution resolution, Gain gain)
 * error_t convert(const Config &configuration) const
 * 
 * Scope: public
 * Parameters:
 * @param channel:        The ADC channel, one of channel0, channel1, channel2 or channel3.
 * @param mode:           The conversion mode, oneShot or continous.
 * @param gain:           The gain setting of the programmable gain amplifier, one of gain1,
 *                        gain2, gain4 or gain8.
 * @param configuration:  A Configuration instance with parameters for the ADC. 
 * 
 * Returns: 
 * @return Value indicating error (if any).
 * 
 * Description:
 * Builds a configuration instance and defers to the Convert with that parameter.
 * 
 ********************************************************************************************/
AdcMcp3428::error_t AdcMcp3428::convert(Channel channel, Mode mode, Gain gain) {
    return convert(Configuration(channel, mode, resolution16, gain));
}

AdcMcp3428::error_t AdcMcp3428::convert(const Configuration& configuration) const {
    Wire.beginTransmission(address);
    Wire.write(configuration.value | newConversionMask);
    if (Wire.endTransmission())
        return errorConvertFailed;
    else
        return errorNone;
}

/********************************************************************************************
 * error_t read(int32_t& result, Configuration& status) const
 * 
 * Scope: public
 * Parameters:
 * @param result:         The signed result.
 * @param configuration:  The contents of the configuration register.
 * 
 * Returns: 
 * @return Value indicating error (if any).
 * 
 * Description:
 * Reads the sample value from the ADC and sets the code in result.
 * 
 ********************************************************************************************/
AdcMcp3428::error_t AdcMcp3428::read(int16_t& result, Configuration& status) const {
    // Read 3 bytes, bytes 1 and 2 will be data and the 3rd byte will be configuration.
    const uint8_t len {3};
    uint8_t buffer[len] {};

    Wire.requestFrom(address, len);
    if (Wire.available() != len)
        return errorReadFailed;
    
    for (uint8_t i = 0; i < len; ++i)
        buffer[i] = Wire.read();

    status = Configuration(buffer[2]);

    if ((status & notReadyMask) != 0)
        return errorConversionNotReady;

    result = (((int16_t)buffer[0]) << 8) | (int16_t)buffer[1];

    return errorNone;  
}

/********************************************************************************************
 * error_t readVoltage(float& voltage, int32_t& result, Configuration& status) const
 * 
 * Scope: public
 * Parameters:
 * @param voltage:        The converted result.
 * @param configuration:  The contents of the configuration register.
 * 
 * Returns: 
 * @return Value indicating error (if any).
 * 
 * Description:
 * Reads the sample value from the ADC and converts it into a voltage.
 * 
 ********************************************************************************************/
AdcMcp3428::error_t AdcMcp3428::readVoltage(float& voltage, Configuration& configuration) {

    double lsb {0.0};
    int16_t maxCode {0}; 
    uint16_t signBit {0};
    int16_t result {0};
    float volts {0.0};

    error_t errorCode {read(result, configuration)}; // Read the code on the configured channel

    if (errorCode != errorNone) 
        return errorCode;

    // Set conversion parameters dependent upon resolution
    switch (int(configuration.getResolution())) {
    case 12:
      lsb = 2 * 2.048 / 4096; // 1mV for 12-bit ADC
      maxCode = 2047;
      signBit = 0x800;
      break;
    case 14:
      lsb = 2 * 2.048 / 16384; // 1mV for 14-bit ADC
      maxCode = 8191;
      signBit = 0x2000;
      break;
    case 16:
      lsb = 2 * 2.048 / 65536; // 62.5uV for 16-bit ADC
      maxCode = 32767;
      signBit = 0x8000;
      break;
    }

    if ((result & signBit) != 0) { // -ve value read
        volts = (~result & maxCode) + 1;
        volts *= -1;
    } else {
        volts = (float)result;
    }
    voltage = volts * lsb / (int(configuration.getGain()));

    return errorNone;  
}

/***********************************************************************************
 * 
 * void buildI2CAddress(uint8_t deviceAddress)
 * 
 * Scope: private
 * Parameters:
 * @param deviceAddress:    This is the unique address of the device on the bus, 
 *                          determined by its adr0 and adr1 pins converted to a 3-bit  
 *                          address.  Upto 8 devices can be supported on one bus.
 *
 * Returns: None
 * 
 * Description:
 * An actual address consists of 8 bits:
 * - 1011 (base address)
 * - deviceAddress, 3 bits
 * - RW, 1 bit: read / write bit
 * The RW bit is added at the time of actually communicating with the device so the 
 * I2C address is just the base address and device address, e.g: 1011xxx0 where xxx
 * is the device address.  In actual fact, the read/write bit is added by the Wire
 * framework so the root address is stored as 01010xxx.
 * 
 **********************************************************************************/
void AdcMcp3428::buildI2CAddress(uint8_t deviceAddress) {
#ifdef DEBUG_ADCMCP3428
    if (deviceAddress > 7) {
        Serial.print("Device Address error: ");
        Serial.println(deviceAddress);
        Serial.println("Only values 0 to 7 are valid.  Changed to 0.")
        Serial.println();
        deviceAddress = 0;
    }
#endif // DEBUG_ADCMCP3428

    address = (uint8_t)(baseI2CAddress | deviceAddress);
}
