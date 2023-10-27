#pragma once
// 2wire interface for TMP175 temperature sensor
class TMP175 {
public:
    TMP175(uint8_t addr, unsigned SettleTimeMsec = SETTLE_TIME_MSEC) : SlaveAddress(addr), SettleTimeMsec(SettleTimeMsec)
    {}

    void startReadTemperature()
    {
        Wire.beginTransmission(SlaveAddress);
        writePointerRegister(CONFIGURATION_REG);
        // R0 without R1 means 10 bit conversion (1/4 degree C) in 55msec
        // OS means one-shot
        // SD means shut down immediately after the one-shot conversion
        uint8_t config = OS | SD | R0;
        Wire.write(config);
        Wire.endTransmission(); 
        MsecWhenStartedTemperature = millis();
    }

    int16_t finishReadTempCx16()
    {
        long tmp175Delay = millis() - MsecWhenStartedTemperature;
        tmp175Delay = static_cast<long>(SettleTimeMsec) - tmp175Delay;
        if (tmp175Delay > 0)
            delay(tmp175Delay);

        Wire.beginTransmission(SlaveAddress);
        writePointerRegister(TEMPERATURE_REG);
        Wire.endTransmission(false);

        int16_t tempCtimes256 = 0;
        static const uint8_t READ_TEMPERATURE_BYTE_COUNT = 2;
        Wire.requestFrom(SlaveAddress, READ_TEMPERATURE_BYTE_COUNT);
        for (int i = 0; i < READ_TEMPERATURE_BYTE_COUNT; i++)
        {
            if (Wire.available())
            {
                unsigned char v = Wire.read();
                tempCtimes256 <<= 8;
                tempCtimes256 |= v;
            }
            else
                return 0xFFFF;
        }
        return tempCtimes256 >> 4;
    }

    void delayForADC()
    {
        delay(SettleTimeMsec); // let temperature ADC settle--depends on R0/R1
    }

    int16_t readTempCx16()
    {
        startReadTemperature();
        delayForADC();
        return finishReadTempCx16();
    }
    float scale() const { return 1/16.f;}
    void setup()
    {
        Wire.beginTransmission(SlaveAddress);
        writePointerRegister(CONFIGURATION_REG);
        uint8_t config = SD; // set shutdown mode to minimize battery drain
        Wire.write(config);
        Wire.endTransmission();
    }

    void dump()
    {
        for (int i = 0; i < NUM_POINTERS; i += 1)
        {
            Wire.beginTransmission(SlaveAddress);
            writePointerRegister(i);
            Wire.endTransmission(false);
            Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(2));
            Serial.print("Dump config="); 
            Serial.println(i);
            while (Wire.available())
            {
                Serial.print(" 0x"); 
                Serial.print((int)Wire.read(), HEX);
            }
            Serial.println();
        }
    }
private:
    enum {SETTLE_TIME_MSEC = 60 };
    const uint8_t SlaveAddress;
    enum Pointer_t { TEMPERATURE_REG, CONFIGURATION_REG, T_LOW_REG, T_HIGH_REG, NUM_POINTERS};
    enum Configuration_t {
        SD = 1, TM = 1 << 1, POL = 1 << 2,
        F0 = 1 << 3, F1 = 1 << 4,
        R0 = 1 << 5, R1 = 1 << 6, OS = 1 << 7
    };
    void writePointerRegister(Pointer_t pointerReg)
    {
        Wire.write(static_cast<uint8_t>(pointerReg));
    }
    unsigned long MsecWhenStartedTemperature;
    unsigned SettleTimeMsec;
};
