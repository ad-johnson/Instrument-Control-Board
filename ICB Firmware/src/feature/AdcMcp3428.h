/*******************************************************************************************
 * 
 * AdcMcp3428:  Class to interact with a MCP3428 ADC.
 *              Updated for Arduino using PlatformIO and C++17 support.
 * 
 * Author: Modified for MCP3428 only by Andrew Johnson
 * 
 * Attribution: https://github.com/stevemarple/MCP342x  Arduino library by Steve Marple.
 * 
 * Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/22226a.pdf
 * February 2021
 * 
 *******************************************************************************************/
#ifndef AdcMcp3428_h
#define AdcMcp3428_h

#include <stdint.h>

class AdcMcp3428 {
public:
    class Configuration;
    class Channel;
    class Mode;
    class Resolution;
    class Gain;
  
    static constexpr	uint8_t		baseI2CAddress {0x68}; 			  // 1101 as defined in the datasheet
    static constexpr	uint8_t		defaultDeviceAddress {0x06};  // 06 - both address pins tied to ground
    static constexpr	uint8_t		maxDeviceAddress {0x07};
    static constexpr uint8_t broadcastAddress {0x00};
    static constexpr uint8_t resetCode {0x06};
    static constexpr uint8_t latchCode {0x04};

    static const Channel channel1;
    static const Channel channel2;
    static const Channel channel3;
    static const Channel channel4;

    static const Mode oneShot;
    static const Mode continous;

    static const Resolution resolution12;
    static const Resolution resolution14;
    static const Resolution resolution16;

    static const Gain gain1;
    static const Gain gain2;
    static const Gain gain4;
    static const Gain gain8;

    static const uint8_t notReadyMask = 0x80;
    static const uint8_t newConversionMask = 0x80;
    static const uint8_t numChannels = 4;
    static const uint8_t maxResolution = 16;
    static const uint8_t maxGain = 8;
    static const int writeTimeout_us = 250;

    enum error_t {
        errorNone,
        errorConvertFailed,
        errorReadFailed,
        errorReadTimeout,
        errorConversionNotReady,
        errorConfigureFailed,
    };

    // Reset the device and sample and latch the address pins
    static uint8_t generalCallReset();

    // Sample and latch the address pins
    static uint8_t generalCallLatch();

    // Constructor
    AdcMcp3428(uint8_t deviceAddress = defaultDeviceAddress);
  
    // Return the I2C address used for communicating with this device.
    uint8_t getAddress() const {
        return address;
    }

    // Check the device is alive
    bool isConnected() const;

    // Instruct the MCP342x device to begin a conversion.
    error_t convert(Channel channel, Mode mode, Gain gain) const; 
    error_t convert(const Configuration& configuration) const;
    
    // Read the sample value from the MCP342x device.
    error_t read(int16_t& result, uint8_t& configuration) const;
    error_t read(int16_t& result, Configuration& Configuration) const;

    // Obtain a voltage reading
    error_t readVoltage(float& voltage, Configuration& configuration);

private:
    uint8_t address; // I2C address of the device

    void buildI2CAddress(uint8_t deviceAddress);  // Store the actual I2C address for this device
};

class AdcMcp3428::Channel {
    friend class AdcMcp3428;
    friend class AdcMcp3428::Configuration;
public:
    inline operator int() const {
        return (value >> 5) + 1;
    }
private:
    uint8_t value;

    inline Channel(uint8_t newValue) : value(newValue & 0x60) {
    };    
};

class AdcMcp3428::Mode {
    friend class AdcMcp3428;
    friend class AdcMcp3428::Configuration;
public:
private:
    uint8_t value;

    inline Mode(uint8_t newValue) : value(newValue & 0x10) {
    };
};

class AdcMcp3428::Resolution {
    friend class AdcMcp3428;
    friend class AdcMcp3428::Configuration;
public:
    inline operator int() const {
        return (value >> 1) + 12;
    }
private:
    uint8_t value;

    inline Resolution(uint8_t newValue) : value(newValue & 0x0C) {
    };
};

class AdcMcp3428::Gain {
    friend class AdcMcp3428;
    friend class AdcMcp3428::Configuration;
public:
      inline operator int() const {
          return (1 << value);
      }
    inline uint8_t log2() const {
        return value;
    }
private:
    uint8_t value;

    inline Gain(uint8_t newValue) : value(newValue & 0x03) {
    };
};

class AdcMcp3428::Configuration {
    friend class AdcMcp3428;
public:
    inline Configuration() : value(0) {
    };
    inline Configuration(uint8_t newValue) : value(newValue) {
    };
    inline Configuration(Channel channel, Mode mode, Resolution resolution, Gain gain) :
        value(channel.value | mode.value | resolution.value | gain.value) {
    };    
    inline Configuration(uint8_t channel, bool continuous, uint8_t resolution, uint8_t gain) :
        value((((channel-1) & 3) << 5)
        | (uint8_t)(continuous ? 0x10 : 0)
        | ((((resolution-12) & 0x1e) << 1) & 0xc)) {
          switch(gain) {
          case 2:
            value |= 0x01;
            break;
          case 4:
            value |= 0x02;
            break;
          case 8:
            value |= 0x03;
            break;
        };
    }    
    inline operator int() const {
        return value;
    }
    inline Channel getChannel() const {
        return Channel(value);
    }
    inline Resolution getResolution() const {
        return Resolution(value);
    }
    inline Gain getGain() const {
        return Gain(value);
    }
    inline bool isReady() const {
        return !(value & notReadyMask); 
    }

private:
    uint8_t value;
};

#endif //AdcMcp3428_h