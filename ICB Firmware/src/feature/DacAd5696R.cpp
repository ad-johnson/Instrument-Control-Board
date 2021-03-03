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
#include "Wire.h"
#include "DacAd5696R.h"

// #define DEBUG_DACAD5696R // Uncomment to get serial UART debug strings

DacAd5696R::DacAd5696R(uint8_t address, uint8_t gain) {
    if (address > maxDeviceAddress)
        address = defaultDeviceAddress;
    if (gain > 2)
        gain = 1;
    deviceAddress = address;
    this->gain = gain;
    lsb = 2.5 * gain * (1 / (2^16)); // The output voltage per binary code value

    powerDAC(normal, (DAC_A | DAC_B | DAC_C | DAC_D));  // Ensure all DACs are powered up.
}

/********************************************************************************************
 * void writeToInputChannel(uint8_t dacs, uint16_t outputCode, double outputVolts)
 * 
 * Scope: public
 * Parameters:
 * @param dacs:         The DACS to set.  Multiple DACS can be set by ORing their addresses together.
 * @param outputCode:   The code for the DACS to emit.
 * @param outputVolts:  Alternatively, a voltage for the DACS to emit.
 * 
 * Returns: None
 * 
 * Description:
 * Writes the output code to the Input Register of the DACS.  These values are not moved to the 
 * DAC Register until LDAC goes high OR the LDAC mask makes the LDAC pin state transparent.
 * A code or volts can be passed - volts are converted to the right code.  The code takes 
 * precedence if both are passed.
 * 
 ********************************************************************************************/
void DacAd5696R::writeToInputChannel(uint8_t dacs, uint16_t outputCode, double outputVolts) const {

    setDACsOutput(writeInputRegister, dacs, outputCode, outputVolts);
}

/********************************************************************************************
 * void writeToInputChannel(uint8_t dacs, uint16_t outputCode, double outputVolts)
 * 
 * Scope: public
 * Parameters:
 * @param dacs: The DACS to set.  Multiple DACS can be set by ORing their addresses together.
 * 
 * Returns: None
 * 
 * Description:
 * Updates the DAC registers with the contents of their corresponding Input Registers; these will
 * be output immediately.
 * 
 ********************************************************************************************/
void DacAd5696R::updateDACRegisterFromInputChannel(uint8_t dacs) const {

    setDACsOutput(updateDacRegister, dacs, 0x00, 0.00);
}

/********************************************************************************************
 * void updateDACChannelDirectly(uint8_t dacs, uint16_t outputCode, double outputVolts)
 * 
 * Scope: public
 * Parameters:
 * @param dacs:         The DACS to set.  Multiple DACS can be set by ORing their addresses together.
 * @param outputCode:   The code for the DACS to emit.
 * @param outputVolts:  Alternatively, a voltage for the DACS to emit.
 * 
 * Returns: None
 * 
 * Description:
 * Writes the output code to the DAC Register of the DACS.  This immediately changes the DAC output
 * irrespective of the state of the LDAC pin or the LDAC register setting for the DAC.
 * A code or volts can be passed - volts are converted to the right code.  The code takes 
 * precedence if both are passed.
 * 
 ********************************************************************************************/
void DacAd5696R::updateDACChannelDirectly(uint8_t dacs, uint16_t outputCode, double outputVolts) const {

    setDACsOutput(updateDacChannel, dacs, outputCode, outputVolts);
}

/********************************************************************************************
 * void readDacs(dacAddress startDac, uint8_t numberToRead, uint16_t *values)
 * 
 * Scope: public
 * Parameters:
 * @param startDac:     The DAC to start reading from.
 * @param numberToRead: The number of DACs to read, max 4.
 * @param values:       Pointer to an array to hold read values.
 * 
 * Returns: None
 * 
 * Description:
 * Reads the Input Register values for requested DACs.  In this function, it is not possible to
 * OR the required DACS together, instead a starting DAC must be given follwed by the number of
 * DACS required to be read.  This wraps around, thus, after reading DAC D, it will read DAC A.
 * E.g. if starting DAC is DAC C for numberTo read 3, values will be loaded, from subscript 0,
 * with DAC C, DAC D, DAC A.  If numberToRead is > 4, then it is constrained to 4.
 * 
 * NOTE: no boundary checking is performed on values thus the caller must ensure that memory
 * is not going to be overwritten.
 * 
 ********************************************************************************************/
void DacAd5696R::readDACs(dacAddress startDac, uint8_t numberToRead, uint16_t *values) const {

    if (numberToRead > 4)
        numberToRead = 4;
    
    Wire.beginTransmission(deviceAddress);
    Wire.write(nop<<4 | startDac);
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, (uint8_t)(2 * numberToRead));
    for (uint8_t i; i < numberToRead; i++)
        values[i] = ((Wire.read() <<8) | (Wire.read()));    
}

/********************************************************************************************
 * void powerDAC(powerMode mode, uint8_t dacs)
 * 
 * Scope: public
 * Parameters:
 * @param mode: the power state to set the DACS to
 * @param dacs: the DACS to set.  Multiple DACS can be set by ORing their addresses together.
 * 
 * Returns: None
 * 
 * Description:
 * Sets the given DACS to a power state defined by mode.
 * 
 ********************************************************************************************/
void DacAd5696R::powerDAC(powerMode mode, uint8_t dacs) const {

    uint16_t dacsToPower {0};
    // Determine the DACS to set power state
    if (dacs & 0x08) // DAC D
        dacsToPower |= (mode << 6);
    if (dacs & 0x04) // DAC C
        dacsToPower |= (mode << 4);
    if (dacs & 0x02) // DAC C
        dacsToPower |= (mode << 2);
    if (dacs & 0x01) // DAC C
        dacsToPower |= mode;

    writeDAC(powerDac, none, dacsToPower); // Dac address is don't care; MSB is don't care.
}

/********************************************************************************************
 * void powerDAC(powerMode modeDacA, powerMode modeDacB, powerMode modeDacC, powerMode modeDacD)
 * 
 * Scope: public
 * Parameters:
 * @param modeDacA: the power state to set the DAC A to
 * @param modeDacB: the power state to set the DAC B to
 * @param modeDacC: the power state to set the DAC C to
 * @param modeDacD: the power state to set the DAC D to
 * 
 * Returns: None
 * 
 * Description:
 * Sets each DAC to a specific power state.
 * 
 ********************************************************************************************/
void DacAd5696R::powerDAC(powerMode modeDacA, powerMode modeDacB, powerMode modeDacC, powerMode modeDacD) const {
    writeDAC(powerDac, none, (modeDacD << 6) | (modeDacC << 4) | (modeDacB << 2) | modeDacA);
}

/********************************************************************************************
 * void setLDACMask(uint8_t mask)
 * 
 * Scope: public
 * Parameters:
 * @param mask: a 4-bit mask to set a DAC to ignore/respond to the LDAC hardware pin
 * 
 * Returns: None
 * 
 * Description:
 * Sets the LDAC register for each DAC (D, C, B, A; 8, 4, 2, 1).  Set bit to 1 to ignore and
 * 0 to respond to the state of the LDAC hardware pin.  This will allow one or more DACS to
 * instantaneously output a voltage irrespective of the state of LDAC. 
 * 
 ********************************************************************************************/
void DacAd5696R::setLDACMask(uint8_t mask) const {
    writeDAC(ldacMaskRegister, none, mask); // DAC address is don't care.
}

/********************************************************************************************
 * void resetAllDacs() const
 * 
 * Scope: public
 * Parameters: None
 * 
 * Returns: None
 * 
 * Description:
 * Resets all DACs to output their power-up value based on the RESET pin.
 * 
 ********************************************************************************************/
void DacAd5696R::resetAllDACs() const {
    writeDAC(resetAll, none, 0x00); // Address and value are don't care
}

/********************************************************************************************
 * void setInternalReference(bool state)
 * 
 * Scope: public
 * Parameters:
 * @param state: boolean to turn the internal reference on or off.
 * 
 * Returns: None
 * 
 * Description:
 * Turning the reference voltage off can save power (but no DAC will work whilst off)
 * 
 ********************************************************************************************/
void DacAd5696R::setInternalReference(bool state) const {

    writeDAC(internalRefRegister, none, (uint16_t)state); // DAC address is don't care
}

/********************************************************************************************
 * void setDacsOutput(command command, uint8_t dacs, uint16_t outputCode, double outputVolts)
 * 
 * Scope: private
 * Parameters:
 * @param command:      The specific output command to run.
 * @param dacs:         The DACS to set.  Multiple DACS can be set by ORing their addresses together.
 * @param outputCode:   The code for the DACS to emit.
 * @param outputVolts:  Alternatively, a voltage for the DACS to emit.
 * 
 * Returns: None
 * 
 * Description:
 * Writes the output code to the DAC Register of the DACS.  A code or volts can be passed - 
 * volts are converted to the right code.  The code takes precedence if both are passed.
 * 
 ********************************************************************************************/
void DacAd5696R::setDACsOutput(command command, uint8_t dacs, uint16_t outputCode, double outputVolts) const {

    if (outputCode == 00 && outputVolts >= 5) // Constrain to largest code
        outputCode = 65535;
    else if (outputCode == 0 && outputVolts > 0.00)
        outputCode = (uint16_t)(outputVolts / lsb); // Convert voltage to a binary value

    writeDACs(command, dacs, outputCode);
}

/********************************************************************************************
 * void writeDAC(command command, dacAddress dac, uint16_t value)
 * 
 * Scope: private
 * Parameters:
 * @param command:  The DAC command to execute.
 * @param dac:      The DAC to execute on (one of four)
 * @param value:    The value to write to the DAC
 * 
 * Returns: None
 * 
 * Description:
 * Sends the command to the DAC with the given value.  The value is sent as two bytes.
 * 
 ********************************************************************************************/
void DacAd5696R::writeDAC(command command, dacAddress dac, uint16_t value) const {

    writeDACs(command, dac, value);
}

/********************************************************************************************
 * void writeDacs(command command, dacAddress dac, uint16_t value)
 * 
 * Scope: private
 * Parameters:
 * @param command:  The DAC command to execute.
 * @param dacs:     The DACs to execute on.  DAC addresses can be ORed together
 * @param value:    The value to write to the DAC
 * 
 * Returns: None
 * 
 * Description:
 * Sends the command to the DAC with the given value.  The value is sent as two bytes.
 * 
 ********************************************************************************************/
void DacAd5696R::writeDACs(command command, uint8_t dacs, uint16_t value) const {

    Wire.beginTransmission(deviceAddress);
    Wire.write((command << 4) | dacs);
    Wire.write((uint8_t)(value >> 8)); // MSB
    Wire.write((uint8_t)(value & 0x00FF)); // LSB
    Wire.endTransmission();
}