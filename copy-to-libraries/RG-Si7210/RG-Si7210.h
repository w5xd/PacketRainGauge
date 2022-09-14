#pragma once
class Si7210 {
public:
    enum {
        FIELD_REGISTER_ADDRESS = 0xC1,
        AUTOINC_REGISTER_ADDRESS = 0xC5,
        SWOP_REGISTER_ADDRESS = 0xC6,
        SLEEPTIMER_REGISTER_ADDRESS = 0xC9,
        POWER_CONTROL_ADDRESS = 0xC4
    };

    Si7210(uint8_t addr) : SlaveAddress(addr), AmSleeping(false)
    {}

    uint8_t setup()
    {
        wakeup();
        resetToOTP();
        wakeup();   
        // grab the sw_op from OTP cuz misreading it later leads to permanent sleep
        Wire.beginTransmission(SlaveAddress);
        Wire.write(SWOP_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
        if (Wire.available())
            OTP_sw_op = Wire.read() & 0x7F;
        AmSleeping = false;
        return OTP_sw_op;
    }

    bool isSleeping() const { return AmSleeping;}

    void wakeup()
    {   // an empty transmission that addresses the device
        Wire.beginTransmission(SlaveAddress);
        Wire.endTransmission();
        delayMicroseconds(10);
        AmSleeping = false;
    }

    uint8_t sleep()
    {
    // see 12.2 Setting an Interrupt: 
    // https://www.silabs.com/documents/public/application-notes/an1018-si72xx-sensors.pdf
        static const char TIMER_BIT = 1;
        static const char FAST_BIT = 2;
        static const char USE_CURRENT_DETECT_SETTINGS = 8;
        Wire.beginTransmission(SlaveAddress);
        Wire.write(POWER_CONTROL_ADDRESS);
        Wire.write(USE_CURRENT_DETECT_SETTINGS);
        Wire.endTransmission();
        AmSleeping = true;
        return 0;
    }

    void resetToOTP()
    {   // sleep mode, but from OTP parameters
        Wire.beginTransmission(SlaveAddress);
        Wire.write(POWER_CONTROL_ADDRESS);
        Wire.write(0);
        Wire.endTransmission();
        AmSleeping = true;
    }

    void one()
    {
        static const char ONEBURST_BIT = 4;
        Wire.beginTransmission(SlaveAddress);
        Wire.write(POWER_CONTROL_ADDRESS);
        Wire.write(ONEBURST_BIT);
        Wire.endTransmission();
    }

    int16_t readMagField()
    {   // chip can read ±20.47 mT 
        Wire.beginTransmission(SlaveAddress);
        Wire.write(FIELD_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(2));
        if (Wire.available() >= 2)
        {
            uint8_t dspsigm = Wire.read();
            uint8_t dspsigl = Wire.read();
            dspsigm &= 0x7Fu; // top bit is "fresh bit"
            return ((dspsigm << 8u) | dspsigl) - 0x4000;
        }
        return 0x8000; // not a possible result
    }

    int16_t toggleOutputSense()
    {
        static const unsigned char sw_low4field_BIT = 0x80;
        Wire.beginTransmission(SlaveAddress);
        Wire.write(SWOP_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
        if (Wire.available())
        {
            uint8_t current = Wire.read();
            current ^= sw_low4field_BIT; // toggle the sw_low4field bit
            current &= sw_low4field_BIT;    // keep only that bit
            current |= OTP_sw_op;
            Wire.beginTransmission(SlaveAddress);
            Wire.write(SWOP_REGISTER_ADDRESS);
            Wire.write(current);
            Wire.endTransmission();
            return current;
        }
        return -1;
    }

    void dump()
    {
        static const char AUTOINC_BIT = 1;
        Wire.beginTransmission(SlaveAddress);
        Wire.write(AUTOINC_REGISTER_ADDRESS);
        Wire.write(AUTOINC_BIT); // set auto-increment bit
        Wire.endTransmission();
        Wire.beginTransmission(SlaveAddress);
        Wire.write(0xC0);
        Wire.endTransmission(false);
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(0xD0-0xC0+1));
        for (int i = 0xC0; i <= 0xD0; i ++)
        {
            if (Wire.available())
            {
                int r = Wire.read();
                Serial.print("reg = 0x");
                Serial.print(i, HEX);
                Serial.print(" = 0x");
                Serial.println(r, HEX);
            }
            else
            {
                Serial.print("No response at i=");
                Serial.println(i, HEX);
                break;
            }
        }
    }
private:
    const uint8_t SlaveAddress;
    bool AmSleeping;
    uint8_t OTP_sw_op; // The "switch" setting as in OTP
};

