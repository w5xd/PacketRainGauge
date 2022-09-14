/* Packet rain gauge
* This sketch is to run on a Pro Mini on a PCB with a hall effect magnet sensor, Si7210.
* It originates a packet for both the arrival and departure of a magnet
* to its sensor. The usual application is to mount a magnet on
* a rocker under a rain gauge funnel.
* 
* This design also reports a temperature
* from a TMP175 sensor and a battery voltage level. It also originates packets
* on a timed interval in the absence of rainfail.
* 
* The Si7210 has a built in temperature sensor, specified as +/-4C. The TMP175 is
* specified as +/-1C
*/

#include <RadioConfiguration.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>

// SparkFun's part numbers are:
// 915MHz:  https://www.sparkfun.com/products/13909
// 434MHz:  https://www.sparkfun.com/products/13910

// Parts of the code in this sketch are taken from these sparkfun pages,
// as are all the wiring instructions:
// https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide

// Uses the RFM69 library by Felix Rusu, LowPowerLab.com
// Original library: https://github.com/LowPowerLab/RFM69

// Include the RFM69 and SPI libraries:
#define USE_RFM69
//#define SLEEP_RFM69_ONLY /* for testing only */
#define USE_SERIAL
#define TELEMETER_BATTERY_V

#if defined(USE_RFM69)
#include <RFM69.h>
#include <RFM69registers.h>
#endif

namespace {
    const int BATTERY_PIN = A0; // digitize (fraction of) battery voltage
    const int TIMER_RC_CHARGE_PIN = 8; // sleep uProc using RC circuit on this pin
    const int RC_RLY_INTERRUPT_PIN = 3;
    const int ROCKER_INPUT_PIN = 17;

    /* The Pro Mini has two interrupt pinsthat have the right features
    ** for responding to the three sources in this design, INT0 and INT1:
    ** 1. the RFM69
    ** 2. the R1/C1 sleep timer
    ** 3. the Si7210 AL pin that indicates the approach or departure of a magnet
    **
    ** The RFM69 gets INT0
    ** The RC circuit and the Si7210 share INT1 through the 74HCS27 configured as an AND gate.
    ** Both are also routed to dedicated pins on the Arduino, D8 and D17, respectively,
    ** so the sketch can tell which was the cause.
    */

    const unsigned long FirstListenAfterTransmitMsec = 20000;// at system reset, listen Serial/RF for this long
    const unsigned long NormalListenAfterTransmit = 300;// after TX, go to RX for this long

#if defined(USE_RFM69)
// RFM69 frequency, uncomment the frequency of your module:

//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ

// AES encryption (or not):
    const bool ENCRYPT = true; // Set to "true" to use encryption
    // Use ACKnowledge when sending messages (or not):
    const bool USEACK = true; // Request ACKs or not
    const int RFM69_RESET_PIN = 9;
    const uint8_t GATEWAY_NODEID = 1;

    class SleepRFM69 : public RFM69
    {
        /* SleepRFM69 has specializations to power it down and power it back up
        ** after a long sleep. */
    public:
        void startAsleep()
        {
            digitalWrite(_slaveSelectPin, HIGH);
            pinMode(_slaveSelectPin, OUTPUT);
            SPI.begin();
            SPIoff();
        }

        void SPIoff()
        {
            // this command drops the idle current by about 100 uA...maybe
            // I could not get consistent results. so I left it in
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP | RF_OPMODE_LISTENABORT);
            _mode = RF69_MODE_STANDBY; // force base class do the write
            sleep();
            SPI.end();

            // set high impedance for all pins connected to RFM69
            // ...except VDD, of course
            pinMode(PIN_SPI_MISO, INPUT);
            pinMode(PIN_SPI_MOSI, INPUT);
            pinMode(PIN_SPI_SCK, INPUT);
            pinMode(PIN_SPI_SS, INPUT);
            pinMode(_slaveSelectPin, INPUT);
        }
        void SPIon()
        {
            digitalWrite(_slaveSelectPin, HIGH);
            pinMode(_slaveSelectPin, OUTPUT);
            SPI.begin();
        }
    };
    // Create a library object for our RFM69HCW module:
    SleepRFM69 radio;
#endif

#if defined(TELEMETER_BATTERY_V)
    void ResetAnalogReference();
#endif
    const unsigned MAX_SLEEP_LOOP_COUNT = 5000;
    const unsigned MONITOR_ROCKER_MSEC = 100; // go at least this long after waking before assuming it was RC was the cause

    RadioConfiguration radioConfiguration;
    bool enableRadio = false;
    unsigned long TimeOfWakeup;
    unsigned SleepLoopTimerCount = 30; // approx R1 * C1 seconds per Count (= 100seconds)

    class TMP175 {
    public:
        TMP175(byte addr) : SlaveAddress(addr)
        {}
        float readTempC()
        {
            Wire.beginTransmission(SlaveAddress);
            writePointerRegister(CONFIGURATION_REG);
            // R1 without R0 means 10 bit conversion (1/4 degree C) in 55msec
            // OS means one-shot
            // SD means shut down immediately after the one-shot conversion
            uint8_t config = OS | SD | R1; 
            Wire.write(config);
            Wire.endTransmission();
            delay(60); // let temperature ADC settle--depends on R0/R1

            Wire.beginTransmission(SlaveAddress);
            writePointerRegister(TEMPERATURE_REG);
            Wire.endTransmission();

            int16_t tempCtimes16 = 0;
            static const uint8_t READ_TEMPERATURE_BYTE_COUNT = 2;
            Wire.requestFrom(SlaveAddress,READ_TEMPERATURE_BYTE_COUNT); 
            for (int i = 0; i < READ_TEMPERATURE_BYTE_COUNT; i++)
            {
                if (Wire.available())
                {
                    unsigned char v = Wire.read();
                    tempCtimes16 <<= 8;
                    tempCtimes16 |= v;
                }
                else
                    return -99.0f;
            }
            return static_cast<float>(tempCtimes16) * Scale;
        }
        void setup()
        {
            Wire.beginTransmission(SlaveAddress); 
            writePointerRegister(CONFIGURATION_REG);
            byte config = SD; // set shutdown mode to minimize battery drain
            Wire.write(config);
            Wire.endTransmission();
        }
    private:
        const uint8_t SlaveAddress;
        enum Pointer_t { TEMPERATURE_REG, CONFIGURATION_REG, T_LOW_REG, T_HIGH_REG };
        enum Configuration_t { SD = 1, TM = 1 << 1, POL = 1 << 2, 
            F0 = 1 << 3, F1 = 1 << 4, 
            R0 = 1 << 5, R1 = 1 << 6, OS = 1 << 7 };
        void writePointerRegister(Pointer_t pointerReg)
        {Wire.write(static_cast<byte>(pointerReg));}

        static const float Scale;
    };
    const float TMP175::Scale = 1 / 16.f; // One count equals 1/16 degree C.

    TMP175 tmp175(0b1001000); //0b1001000 per TMP175 docs, is I2C address with all addressing pins low.

    class Si7210 {
    public:
        Si7210(uint8_t addr) : SlaveAddress(addr)
        {}

        void setup()
        {
            wakeup();

            Wire.beginTransmission(SlaveAddress);
            Wire.write(0xC0);
            Wire.endTransmission(false);
            Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
            while (Wire.available())
                Wire.read();
        }

        void wakeup()
        {
            Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
            while (Wire.available())
                Wire.read();
            Wire.beginTransmission(SlaveAddress);
            Wire.write(0xC5);
            Wire.write(1); // set auto-increment bit
            Wire.endTransmission();
        }

        void sleep()
        {
            static const char SLEEP_REGISTER_ADDRESS = 0xC4;
            static const char SLEEP_BIT = 1;

            Wire.beginTransmission(SlaveAddress);
            Wire.write(SLEEP_REGISTER_ADDRESS);
            Wire.write(SLEEP_BIT);
            Wire.endTransmission();
        }

        int16_t readMagField()
        {   // chip can read �20.47 mT 
            while (Wire.available())
                Wire.read();
            static const char FIELD_REGISTER_ADDRESS = 0xC1;
            Wire.beginTransmission(SlaveAddress);
            Wire.write(FIELD_REGISTER_ADDRESS);
            Wire.endTransmission(false);
            Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(2));
            if (Wire.available() >= 2)
            {
                uint8_t dspsigm = Wire.read();
                uint8_t dspsigl = Wire.read();
                dspsigm &= 0x7Fu; // top bit is "fresh"
                return (dspsigm << 8u | dspsigl) - 0x4000;
            }
            return 0x8000; // not a possible result
        }

        uint8_t toggleOutputSense()
        {
            while (Wire.available())
                Wire.read();
            static const char SWOP_REGISTER_ADDRESS = 0xC6;
            static const unsigned char sw_low4field_BIT = 0x80;
            Wire.beginTransmission(SlaveAddress);
            Wire.write(SWOP_REGISTER_ADDRESS);
            Wire.endTransmission(false);
            Wire.requestFrom(SlaveAddress, static_cast<uint8_t>(1));
            if (Wire.available())
            {
                uint8_t current = Wire.read();
                current ^= sw_low4field_BIT; // toggle the sw_low4field bit
                Wire.beginTransmission(SlaveAddress);
                Wire.write(SWOP_REGISTER_ADDRESS);
                Wire.write(current);
                Wire.endTransmission();
                return current & 0x7F;
            }
            return 0xff;
        }
    private:
        const uint8_t SlaveAddress;
    };

    Si7210 si7210(0x33);
}

void setup()
{
#if defined(USE_SERIAL)
    // Open a serial port so we can send keystrokes to the module:
    Serial.begin(9600);
    Serial.print("Node ");
    Serial.print(radioConfiguration.NodeId(), DEC);
    Serial.print(" on network ");
    Serial.print(radioConfiguration.NetworkId(), DEC);
    Serial.print(" band ");
    Serial.print(radioConfiguration.FrequencyBandId(), DEC);
    Serial.print(" key ");
    radioConfiguration.printEncryptionKey(Serial);
    Serial.println(" ready");
#endif

#if defined(USE_RFM69)

#if !defined(SLEEP_RFM69_ONLY)
    // Initialize the RFM69HCW:
    auto nodeId = radioConfiguration.NodeId();
    auto networkId = radioConfiguration.NetworkId();
    auto fbId = radioConfiguration.FrequencyBandId();
    auto ok = nodeId != 0xff &&
            networkId != 0xff &&
            fbId != 0xff &&
            radio.initialize(fbId, nodeId, networkId);
#if defined(USE_SERIAL)
    Serial.println(ok ? "Radio init OK" : "Radio init failed");
    if (ok)
    {
        enableRadio = true;
        uint32_t freq;
        if (radioConfiguration.FrequencyKHz(freq))
            radio.setFrequency(1000*freq);
        Serial.print("Freq= "); Serial.print(radio.getFrequency()/1000); Serial.println(" KHz");
    }
#endif   

    radio.setHighPower(); // Always use this for RFM69HCW
    // Turn on encryption if desired:
    if (ENCRYPT)
        radio.encrypt(radioConfiguration.EncryptionKey());
#else
    radio.startAsleep();
#endif

#endif

    pinMode(RC_RLY_INTERRUPT_PIN, INPUT);
    pinMode(ROCKER_INPUT_PIN, INPUT);

#if defined(TELEMETER_BATTERY_V)
    ResetAnalogReference();
#endif

    TimeOfWakeup = millis(); // start loop timer now

    unsigned eepromLoopCount(0);
    EEPROM.get(RadioConfiguration::TotalEpromUsed(), eepromLoopCount);
    if (eepromLoopCount && eepromLoopCount <= MAX_SLEEP_LOOP_COUNT)
    	SleepLoopTimerCount = eepromLoopCount;

#if defined(USE_SERIAL)
    Serial.print("SleepLoopTimerCount = ");
    Serial.println(SleepLoopTimerCount);
#endif

    Wire.begin();
    tmp175.setup();
    si7210.setup();
}

/* Power management:
 * For ListenAfterTransmitMsec we stay awake and listen on the radio and Serial.
 * Then we power down all: temperature sensor, radio and CPU and CPU
 * sleep using SleepTilNextInterrupt.
 */

namespace {
    unsigned SleepTilNextInterrupt();

    unsigned long ListenAfterTransmitMsec = FirstListenAfterTransmitMsec;
    unsigned int sampleCount;

    bool processCommand(const char *pCmd)
    {
        static const char SET_LOOPCOUNT[] = "SetDelayLoopCount";
        if (strncmp(pCmd, SET_LOOPCOUNT, sizeof(SET_LOOPCOUNT) - 1) == 0)
        {
            pCmd = RadioConfiguration::SkipWhiteSpace(
            		pCmd +	sizeof(SET_LOOPCOUNT)-1);
            if (pCmd)
            {
                unsigned v = RadioConfiguration::toDecimalUnsigned(pCmd);
                // don't allow zero, nor more than MAX_SLEEP_LOOP_COUNT
                if (v && v < MAX_SLEEP_LOOP_COUNT)
                {
                    SleepLoopTimerCount = v;
                    EEPROM.put(RadioConfiguration::TotalEpromUsed(), SleepLoopTimerCount);
                    return true;
                }
            }
        }
        return false;
    }
}

void loop()
{
    unsigned long now = millis();
    static bool wakedByRocker = false;
    static uint8_t sw_op;

    if (!wakedByRocker && digitalRead(ROCKER_INPUT_PIN) == LOW)
    { /* I only have one job under this funnel, and I'm going to do it.*/
        wakedByRocker = true;
        sw_op = si7210.toggleOutputSense(); // do this only once per wakeup
        TimeOfWakeup = now; // extend sleep timer
    }

#if defined(USE_SERIAL)
    // Set up a "buffer" for characters that we'll send:
    static char sendbuffer[62];
    static int sendlength = 0;
    // In this section, we'll gather serial characters and
    // send them to the other node if we (1) get a carriage return,
    // or (2) the buffer is full (61 characters).

    // If there is any serial input, add it to the buffer:

    if (Serial.available() > 0)
    {
        TimeOfWakeup = now; // extend timer while we hear something
        char input = Serial.read();

        if (input != '\r') // not a carriage return
        {
            sendbuffer[sendlength] = input;
            sendlength++;
        }

        // If the input is a carriage return, or the buffer is full:

        if ((input == '\r') || (sendlength == sizeof(sendbuffer) - 1)) // CR or buffer full
        {
            sendbuffer[sendlength] = 0;
            if (processCommand(sendbuffer))
            {
                Serial.print(sendbuffer);
                Serial.println(" command accepted for thermometer");
            }
            else if (radioConfiguration.ApplyCommand(sendbuffer))
            {
                Serial.print(sendbuffer);
                Serial.println(" command accepted for radio");
            }
            Serial.println("ready");
            sendlength = 0; // reset the packet
        }
    }
#endif

#if defined(USE_RFM69) && !defined(SLEEP_RFM69_ONLY)
    // RECEIVING
    // In this section, we'll check with the RFM69HCW to see
    // if it has received any packets:

    if (radio.receiveDone()) // Got one!
    {
        // Print out the information:
        TimeOfWakeup = now; // extend sleep timer
#if defined(USE_SERIAL)
        Serial.print("received from node ");
        Serial.print(radio.SENDERID, DEC);
        Serial.print(", message [");

        // The actual message is contained in the DATA array,
        // and is DATALEN bytes in size:

        for (byte i = 0; i < radio.DATALEN; i++)
            Serial.print((char)radio.DATA[i]);

        // RSSI is the "Receive Signal Strength Indicator",
        // smaller numbers mean higher power.

        Serial.print("], RSSI ");
        Serial.println(radio.RSSI);
#endif
        // RFM69 ensures trailing zero byte, unless buffer is full...so
        radio.DATA[sizeof(radio.DATA) - 1] = 0; // ...if buffer is full, ignore last byte
        if (processCommand((const char *)&radio.DATA[0]))
        {
#if defined(USE_SERIAL)
        	Serial.println("Received command accepted");
#endif
        }
        if (radio.ACKRequested())
        {
            radio.sendACK();
#if defined(USE_SERIAL)
            Serial.println("ACK sent");
#endif
        }
    }
#endif

    static bool TransmittedSinceSleep = false;
    if (!TransmittedSinceSleep)
    {
        if (wakedByRocker || (now - TimeOfWakeup >= MONITOR_ROCKER_MSEC))
        {   /* transmit once per wakeup.
            ** report rocker movement on any poll of ROCKER_INPUT_PIN LOW
            ** during first MONITOR_ROCKER_MSEC. */
            TransmittedSinceSleep = true;
            int batt(0);
#if defined(TELEMETER_BATTERY_V)
            // 10K to VCC and (wired on board) 2.7K to ground
            pinMode(BATTERY_PIN, INPUT_PULLUP); // sample the battery
            batt = analogRead(BATTERY_PIN);
            /* batt result means little in absolute terms, but its
            ** history is telling. Failing cells have a pattern.*/
            pinMode(BATTERY_PIN, INPUT); // turn off battery drain
#endif

            auto magField = si7210.readMagField();

            // read temperature data
            float temperature = tmp175.readTempC();

            char sign = '+';
            static char buf[64];
            if (temperature < 0.f) {
                temperature = -temperature;
                sign = '-';
            }
            else if (temperature == 0.f)
                sign = ' ';

            int whole = (int)temperature;

            sprintf(buf, "RG:%d C:%u, B:%d, T:%c%d.%02d F:%7d DP:0x%x",
                (int)(wakedByRocker ? 1 : 0), 
                sampleCount++,
                batt,
                sign, whole,
                (int)(100.f * (temperature - whole),
                magField,
                (int)sw_op)
            );
#if defined(USE_SERIAL)
            Serial.println(buf);
#endif
#if defined(USE_RFM69) && !defined(SLEEP_RFM69_ONLY)
            if (enableRadio)
                radio.sendWithRetry(GATEWAY_NODEID, buf, strlen(buf));
#endif
        }
    }

    if (now - TimeOfWakeup > ListenAfterTransmitMsec)
    {
        wakedByRocker = false;
        TransmittedSinceSleep = false;
        SleepTilNextInterrupt();
        TimeOfWakeup = millis();
        ListenAfterTransmitMsec = NormalListenAfterTransmit;
    }
}

#if !defined(SLEEP_WITH_TIMER2)
void sleepPinInterrupt()	// requires 10uF and 10M between two pins
{
    // run at interrupt level. disables further interrupt from this source
    detachInterrupt(digitalPinToInterrupt(RC_RLY_INTERRUPT_PIN));
}
#else
#endif

namespace {
    unsigned SleepTilNextInterrupt()
    {

#if defined(USE_SERIAL)
        Serial.print("sleep for count=");
        Serial.println(SleepLoopTimerCount);
        Serial.end();// wait for finish and turn off pins before sleep
        pinMode(0, INPUT); // Arduino libraries have a symbolic definition for Serial pins?
        pinMode(1, INPUT);
#endif

#if defined(USE_RFM69) && !defined(SLEEP_RFM69_ONLY)
        radio.SPIoff();
#endif

#if defined(TELEMETER_BATTERY_V)
        analogReference(EXTERNAL); // This sequence drops idle current by 30uA
        analogRead(BATTERY_PIN); // doesn't shut down the band gap until we USE ADC
#endif

        si7210.sleep();

        // sleep mode power supply current measurements indicate this appears to be redundant
        power_all_disable(); // turn off everything

        unsigned count = 0;

#if !defined(SLEEP_WITH_TIMER2)
        // this uses R1 and C1 to ground
        while (count < SleepLoopTimerCount)
        {
            power_timer0_enable(); // delay() requires this
            pinMode(TIMER_RC_CHARGE_PIN, OUTPUT);
            digitalWrite(TIMER_RC_CHARGE_PIN, HIGH);
            delay(10); // Charge the 1uF
            pinMode(TIMER_RC_CHARGE_PIN, INPUT);
            cli();
            power_timer0_disable(); // timer0 powered down again
            attachInterrupt(digitalPinToInterrupt(RC_RLY_INTERRUPT_PIN), sleepPinInterrupt, LOW);
            set_sleep_mode(SLEEP_MODE_PWR_DOWN);
            sleep_enable();
            sleep_bod_disable();
            sei();
            sleep_cpu(); // about 300uA, average. About 200uA and rises as cap discharges
            sleep_disable();
            sei();
            count += 1;
        }

#else
#endif

        power_all_enable();

#if defined(TELEMETER_BATTERY_V)
        ResetAnalogReference();
#endif

#if defined(USE_SERIAL)
        Serial.begin(9600);
        Serial.print(count, DEC);
        Serial.println(" wakeup");
#endif

#if defined(USE_RFM69) && !defined(SLEEP_RFM69_ONLY)
        radio.SPIon();
#endif
        si7210.wakeup();

        return count;
    }

#if defined(TELEMETER_BATTERY_V)
	void ResetAnalogReference()
	{
		analogReference(INTERNAL);
		pinMode(BATTERY_PIN, INPUT);
		analogRead(BATTERY_PIN);
		delay(10); // let ADC settle
	}
#endif
}