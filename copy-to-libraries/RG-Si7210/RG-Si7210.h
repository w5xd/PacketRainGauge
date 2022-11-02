#pragma once
class Si7210 {
    // https://www.silabs.com/documents/public/data-sheets/si7210-datasheet.pdf
public:
    enum {
        FIELD_REGISTER_ADDRESS = 0xC1,
        AUTOINC_REGISTER_ADDRESS = 0xC5,
        SWOP_REGISTER_ADDRESS = 0xC6,
        SWHYST_REGISTER_ADDRESS = 0xC7,
        SWHYST_MASK = 0x3f,
        SLEEPTIMER_REGISTER_ADDRESS = 0xC9,
        SWTAMPER_ADDRESS = SLEEPTIMER_REGISTER_ADDRESS,
        SWTAMPER_SHIFT = 2,
        POWER_CONTROL_ADDRESS = 0xC4,
        INVALID_FIELD_READING = 0x8000,
        CONVERSION_TIME_MICROSECONDS = 11,
        LOW_FIELD_BIT_NUMBER = 7
    };

    typedef int16_t MagField_t;

    Si7210(uint8_t addr) : SlaveAddress(addr), AmSleeping(false)
    {}

    static uint16_t threshold(uint8_t sw_op) {
        sw_op &= 0x7f;
        uint16_t ret = 16 + (sw_op & 0xF);
        ret <<= (sw_op >> 4) & 0x7;
        return ret;
    }
    static uint16_t hysteresis(uint8_t sw_hyst)
    {
        sw_hyst &= 0x3f;
        uint16_t ret = 8 + (sw_hyst & 7);
        ret <<= (sw_hyst >> 3) & 7;
        return ret;
    }

    uint16_t setup()
    {
        uint16_t ret = 0;
        wakeup();
        resetToOTP();
        wakeup();   
        // grab the sw_op from OTP cuz misreading it later leads to permanent sleep
        Wire.beginTransmission(SlaveAddress);
        Wire.write(SWOP_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
        if (Wire.available())
            ret = threshold(Wire.read());
        AmSleeping = false;
        return ret;
    }

    bool isSleeping() const { return AmSleeping;}

    void wakeup()
    {   // an empty transmission that addresses the device
        Wire.beginTransmission(SlaveAddress);
        Wire.endTransmission();
        AmSleeping = false;
    }

    uint8_t sleep(bool fromOTP = true)
    {
    // see 12.2 Setting an Interrupt: 
    // https://www.silabs.com/documents/public/application-notes/an1018-si72xx-sensors.pdf
        static const char USE_OTP_DETECT_SETTINGS = 8;
        Wire.beginTransmission(SlaveAddress);
        Wire.write(POWER_CONTROL_ADDRESS);
        Wire.write(fromOTP ? 0 : USE_OTP_DETECT_SETTINGS );
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

    MagField_t readMagField()
    {   // chip can read ±20.47 mT 
        Wire.beginTransmission(SlaveAddress);
        Wire.write(FIELD_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(2));
        if (Wire.available() >= 2)
        {
            uint8_t dspsigm = Wire.read();
            uint8_t dspsigl = Wire.read();
            if (dspsigm & 0x80u) // top bit is "fresh bit"
            {
                dspsigm &= 0x7Fu; 
                return ((dspsigm << 8u) | dspsigl) - 0x4000;
            }
        }
        return INVALID_FIELD_READING; // not a possible result
    }

    int16_t toggleOutputSense()
    {
        Wire.beginTransmission(SlaveAddress);
        Wire.write(SWOP_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
        if (Wire.available())
        {
            uint8_t current = Wire.read();
            current ^= 1 << LOW_FIELD_BIT_NUMBER; // toggle the sw_low4field bit
            Wire.beginTransmission(SlaveAddress);
            Wire.write(SWOP_REGISTER_ADDRESS);
            Wire.write(current);
            Wire.endTransmission();
            return current;
        }
        return -1;
    }

    uint8_t getSwTamper()
    {
        uint8_t ret = 0xff;
        Wire.beginTransmission(SlaveAddress);
        Wire.write(SWTAMPER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
        if (Wire.available())
            ret = Wire.read() >> SWTAMPER_SHIFT;
        return ret;
    }

    void setSwTamper(uint8_t v)
    {
        const uint8_t NOT_TAMPER_MASK = 0x3;
        Wire.beginTransmission(SlaveAddress);
        Wire.write(SWTAMPER_ADDRESS);
        Wire.endTransmission(false);
        uint8_t curValue;
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
        if (Wire.available())
        {
            curValue = Wire.read();
            curValue &= NOT_TAMPER_MASK;
            curValue |= v << SWTAMPER_SHIFT;
            Wire.beginTransmission(SlaveAddress);
            Wire.write(SWTAMPER_ADDRESS);
            Wire.write(curValue);
            Wire.endTransmission();
        }
    }

    uint16_t setSwOp(uint8_t v)
    {
        uint16_t ret = 0;
        Wire.beginTransmission(SlaveAddress);
        Wire.write(SWOP_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
        if (Wire.available())
        {
            uint8_t curValue = Wire.read();
            curValue &= 1 << LOW_FIELD_BIT_NUMBER;
            curValue |= v;
            Wire.beginTransmission(SlaveAddress);
            Wire.write(SWOP_REGISTER_ADDRESS);
            Wire.write(curValue);
            Wire.endTransmission();
            ret = threshold(curValue);
        }
        return ret;
    }

    uint8_t getSwOp()
    {
        Wire.beginTransmission(SlaveAddress);
        Wire.write(SWOP_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
        if (Wire.available())
            return ~(1 << LOW_FIELD_BIT_NUMBER) & Wire.read();
        return 0xffu;
    }

    uint16_t setSwHyst(uint8_t v)
    {
        uint16_t ret = 0;
        Wire.beginTransmission(SlaveAddress);
        Wire.write(SWHYST_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
        if (Wire.available())
        {
            uint8_t curValue = Wire.read();
            Wire.beginTransmission(SlaveAddress);
            Wire.write(SWHYST_REGISTER_ADDRESS);
            curValue &= ~SWHYST_MASK;
            curValue |= v;
            Wire.write(curValue);
            Wire.endTransmission();
            ret = hysteresis(curValue);
        }
        return ret;
    }

    uint8_t getSwHyst()
    {
        Wire.beginTransmission(SlaveAddress);
        Wire.write(SWHYST_REGISTER_ADDRESS);
        Wire.endTransmission(false);
        Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
        if (Wire.available())
            return SWHYST_MASK & Wire.read();
        return 0xff;
    }
    static MagField_t getMaxAmplitude() { return static_cast<MagField_t>(MAX_MAGFIELD_MAGNITUDE); }

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
    enum { MAX_MAGFIELD_MAGNITUDE = 16384 };
    const uint8_t SlaveAddress;
    bool AmSleeping;
};

