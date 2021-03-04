/*******************************************************************************************
 * 
 * DacAd5696R:  Class to interact with a AD5696R ADC.
 *              Created for Arduino using PlatformIO and C++17 support.
 * 
 * Author: Andrew Johnson
 * 
 * Attribution: None
 * 
 * Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/AD5696R_5695R_5694R.pdf
 * March 2021
 * 
 *******************************************************************************************/
#ifndef DacAd5696R_h
#define DacAd5696R_h

#include "Arduino.h"
#include <stdint.h>

class DacAd5696R {
public:

    enum dacAddress:uint8_t {
        none = 0,
        DAC_A = 1,
        DAC_B = 2,
        DAC_C = 4,
        DAC_D = 8
    };

    enum powerMode:uint8_t {
        normal,         // normal operation
        lowImpedance,   // powered down 1kOhm to Gnd
        highImpedance,  // powered down 100kOhm to Gnd
        open            // three-state (open-circuited)
    };

    DacAd5696R(uint8_t address = defaultDeviceAddress, uint8_t gain = defaultGain);

    void writeToInputChannel(uint8_t dacs, uint16_t outputCode, double outputVolts) const; // Writes values to input register
    void updateDACRegisterFromInputChannel(uint8_t dacs) const; // Copies input register to DAC register
    void updateDACChannelDirectly(uint8_t dacs, uint16_t outputCode, double outputVolts) const; // Write value to DAC register

    void powerDAC(powerMode mode, uint8_t dacs) const; // Set selected dacs to the required power state
    void powerDAC(powerMode modeDacA, powerMode modeDacB, powerMode modeDacC, powerMode modeDacD) const; // Set Dacs to required power state
    void setLDACMask(uint8_t mask) const; // sets channels to respond to or ignore LDAC pin state.  OR values together.

    void resetAllDACs() const; // Set all DACS to power-up state based on Reset pin
    void setInternalReference(bool state) const; // Turn internal reference on (TRUE) or off (FALSE)

    void readDACs(dacAddress startDac, uint8_t numberToRead, uint16_t *values) const; // Read the Input Register values of one or more DACs.

private:
    enum command {
        nop,                // No operation
        writeInputRegister, // Write to input register n (dependent on LDAC)
        updateDacRegister,  // Update DAC register n with contents of input register n
        updateDacChannel,   // Write to and update DAC channel n
        powerDac,           // Power down/up DAC
        ldacMaskRegister,   // Hardware LDAC mask register
        resetAll,              // Software reset (power-on reset)
        internalRefRegister // Internal reference setup register
    };

	static constexpr uint8_t baseI2CAddress {0x03}; 		// 00011 as defined in the datasheet
	static constexpr uint8_t defaultDeviceAddress {0x00};   // 00 - both address pins tied to ground
	static constexpr uint8_t maxDeviceAddress {0x03};		// in bits: 00, 01, 10, 11.
    static constexpr uint8_t defaultGain {2};               // Gain pin tied to VDD

	uint8_t	deviceAddress {};   // Implemented Device Address on PCB
    uint8_t gain {};            // Gain for the device
    double  lsb {}; // The minimum output voltage per binary value

    void writeDAC(command command, dacAddress dac, uint16_t value) const;
    void writeDACs(command command, uint8_t dacs, uint16_t value) const;
    void setDACsOutput(command command, uint8_t dacs, uint16_t outputCode, double outputVolts) const;
};

#endif // DacAd5696R_h