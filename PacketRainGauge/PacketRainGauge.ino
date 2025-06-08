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
* 
* The default magnetic field amplitude threshold and hysteresis give more interrupts
* than these settings:
* EEPROM raw =0x65 threshold=1344
* EEPROM raw=0x30 hysteresis = 512
* si7210 tamper= 63
* 
* Serial port Commands to set the si7210 as above:
* SetSWOP 65
* SetSWHYST 30
*/

#include <RadioConfiguration.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <RG-Si7210.h>
#include <RG-Tmp175.h>

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
//#define SERIAL_DEBUG_OUTPUT

#define FAR_THRESHOLD(X) X/8
#define NEAR_THRESHOLD(x) x / 2

#define REVISION "REV04"

#if defined(USE_RFM69)
#include <RFM69.h>
#include <RFM69registers.h>
#endif

namespace {
    enum class EepromAddresses {
        PACKET_RAINGAUGE_START = RadioConfiguration::EepromAddresses::TOTAL_EEPROM_USED,
        PACKET_RAINGAUGE_SLEEP_LOOP_COUNT = PACKET_RAINGAUGE_START,
        PACKET_RAINGAUGE_DISABLE_SERIAL = PACKET_RAINGAUGE_SLEEP_LOOP_COUNT + sizeof(unsigned),
        PACKET_RAINGAUGE_SWOP = PACKET_RAINGAUGE_DISABLE_SERIAL + 1,
        PACKET_RAINGAUGE_SWHYST = PACKET_RAINGAUGE_SWOP + 1,
        PACKET_RAINGAUGE_END = PACKET_RAINGAUGE_SWHYST + 1
    };
    const int BATTERY_PIN = A0; // digitize (fraction of) battery voltage
    const int TIMER_RC_CHARGE_PIN = 8; // sleep uProc using RC circuit on this pin
    const int TXD_PIN = 1;
    const int RC_RLY_INTERRUPT_PIN = 3;
    const int RC_STATUS_PIN = 16;   // not used, but on REV02 of the PCB
    const int ROCKER_INPUT_PIN = 17;

    /* The Pro Mini has two interrupt pins that have the right features
    ** for responding to the three sources in this design, INT0 and INT1:
    ** 1. the RFM69
    ** 2. the R1/C1 sleep timer
    ** 3. the Si7210 AL pin that indicates the approach or departure of a magnet
    **
    ** The RFM69 is on INT0
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

    bool getOnloopSerialDisable()
    {
        int addr = static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_DISABLE_SERIAL);
        uint8_t ret = EEPROM.read(addr);
        if (ret == static_cast<uint8_t>(0xff))
            return 0;
        return ret != 0;
    }
    void setOnloopSerialDisable(bool b)
    {
        int addr = static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_DISABLE_SERIAL);
        uint8_t v = b ? 1 : 0;
        EEPROM.write(addr, v);
    }
    uint8_t getRaingaugeSwop()
    {
        int addr = static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_SWOP);
        return EEPROM.read(addr);
    }
    void setRaingaugeSwop(uint8_t v)
    {
        int addr = static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_SWOP);
        EEPROM.write(addr, v);
    }
    uint8_t getRaingaugeSwHyst()
    {
        int addr = static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_SWHYST);
        return EEPROM.read(addr);
    }
    void setRaingaugeSwHyst(uint8_t v)
    {
        int addr = static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_SWHYST);
        EEPROM.write(addr, v);
    }


#if defined(TELEMETER_BATTERY_V)
    void ResetAnalogReference();
#endif
    const unsigned MAX_SLEEP_LOOP_COUNT = 5000;
    const unsigned MONITOR_ROCKER_MSEC = 100; // go at least this long after waking before assuming it was RC was the cause

    RadioConfiguration radioConfiguration;
    unsigned long TimeOfWakeup;
    unsigned SleepLoopTimerCount = 30; // approx R1 * C1 seconds per Count (= 100seconds)

    TMP175 tmp175(0b1001000); //0b1001000 per TMP175 docs, is I2C address with all addressing pins low.

    Si7210 si7210(0x33);
    bool prevSentMagnetClose;
    const int MAX_WAIT_FOR_TOGGLE_MSEC = 100;
    const long SERIAL_PORT_BAUDS = 38400;
    const int MAX_MAGFIELD_POLL = 10;

    bool enableSerial = true;
    const char SET_SERIALONLOOP[] = "DisableSerialOnLoop";
    bool radioOK = false;
}

void setup()
{
#if defined(USE_SERIAL)
    // Open a serial port so we can send keystrokes to the module:
    enableSerial = true;
    Serial.begin(SERIAL_PORT_BAUDS);
    delay(100);
    Serial.println("Packet Rain Gauge " REVISION);
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
    radioOK = nodeId != 0xff &&
            networkId != 0xff &&
            fbId != 0xff &&
            radio.initialize(fbId, nodeId, networkId);
#if defined(USE_SERIAL)
    Serial.println(radioOK ? "Radio init OK" : "Radio init failed");
    if (radioOK)
    {
        uint32_t freq;
        if (radioConfiguration.FrequencyKHz(freq))
            radio.setFrequency(1000*freq);
        Serial.print("Freq= "); Serial.print(radio.getFrequency()/1000); Serial.println(" KHz");
    }
#endif   

    if (radioOK)
    {
        radio.setHighPower(); // Always use this for RFM69HCW
    // Turn on encryption if desired:
        if (radioConfiguration.encrypted() && ENCRYPT)
            radio.encrypt(radioConfiguration.EncryptionKey());
    }
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
    EEPROM.get(static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_SLEEP_LOOP_COUNT), eepromLoopCount);
    if (eepromLoopCount && eepromLoopCount <= MAX_SLEEP_LOOP_COUNT)
    	SleepLoopTimerCount = eepromLoopCount;

#if defined(USE_SERIAL)
    Serial.print("SleepLoopTimerCount = ");
    Serial.println(SleepLoopTimerCount);
#endif

    Wire.begin();
    tmp175.setup();
    auto OTPthreshold = si7210.setup();

    Serial.print("si7210 OTP threshold=");
    Serial.println(OTPthreshold);

    auto swop = getRaingaugeSwop();
    uint16_t threshold;
    if (swop != 0xffu)
    {
        threshold = si7210.setSwOp(swop);
        Serial.print("EEPROM raw =0x"); 
        Serial.print((int)swop, HEX);
        Serial.print(" threshold=");
        Serial.println(threshold);
    }

    auto hyst = getRaingaugeSwHyst();
    uint16_t hysteresis;
    if (hyst != 0xffu)
    {
        hysteresis = si7210.setSwHyst(hyst);
        Serial.print("EEPROM raw=0x");
        Serial.print((int) hyst, HEX);
        Serial.print(" hysteresis = ");
    }
    else
    {
        Serial.print("OTP hysteresis = ");
        hysteresis = si7210.hysteresis(si7210.getSwHyst());
    }
    Serial.println(hysteresis);

    /* This sketch allows setting the si7210 parameters arbitrarily....
    ** but here are some hints for setting it up.
    ** The 3D prints arrange for the sensor to be very close to magnet when one
    ** side of the bucket is down. That gives a maximum, or near maximum, reading to the sensor
    ** The other buckets gives some lower reading, depending on how far away the 3D print can make the
    ** magnet travel.
    ** So...set the "Swop" (which is actually the threshold) to a large number. 0x7E is the largest. 
    ** That means that the magnet needs to be near enough to get that large reading.
    ** Set the hysteresis LOW (a nice low number is 0). That will give an interrupt very close to the
    ** the threshold value. That is, an interrupt as the bucket brings the magnet close, and then again
    ** as the bucket starts to move away.
    */
    int tamper = si7210.getSwTamper();
    Serial.print("si7210 tamper= ");
    Serial.println(tamper);

    si7210.setFieldPolSel(0);
    si7210.one();
    if (prevSentMagnetClose = (digitalRead(ROCKER_INPUT_PIN) == LOW))
    {
        si7210.toggleOutputSense();
        Serial.println("Magnet close on startup.");
        delay(MAX_WAIT_FOR_TOGGLE_MSEC);
    }

    Serial.print(SET_SERIALONLOOP);
    Serial.print(" ");
    Serial.println(getOnloopSerialDisable() ? "ON" : "OFF");

    tmp175.startReadTemperature();
    Serial.println("Setup complete");
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

    void SetSerialEnabled(bool newVal)
    {
        if (newVal != enableSerial)
        {
            if (newVal)
            {
                Serial.begin(SERIAL_PORT_BAUDS);
                Serial.println("Serial enabled");
            }
            else
            {
                Serial.flush();
                Serial.end();
            }
            enableSerial = newVal;
        }
    }

    bool processCommand(const char *pCmd)
    {
        static const char SET_LOOPCOUNT[] = "SetDelayLoopCount";
        static const char SET_SERIAL[] = "SetSerial";
        static const char SET_THRESHOLD[] = "SetSWOP"; // HEX number range 00 : 7F where 7f is special case of "latch mode" where  
        static const char SET_HYSTERESIS[] = "SetSWHYST"; // argument is HEX number range 00 : 3F, 3F is special case for ZERO
        static const char SET_TOOTP[] = "SetToOTP";
        static const char VER[] = "VER";
        static const char DUMP_RFM69[] = "DumpRFM69";
        const char *q = pCmd;

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
                    EEPROM.put(static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_SLEEP_LOOP_COUNT), SleepLoopTimerCount);
                    return true;
                }
            }
        }
        else if (strncmp(pCmd, VER, sizeof(VER) - 1) == 0)
        {
#if defined(USE_SERIAL)
            Serial.println("Revision 03");
#endif
            return true;
        }
        else if (q = strstr(pCmd, SET_SERIAL))
        {
            q += sizeof(SET_SERIAL)-1;
            while (isspace(*q)) q += 1;
            SetSerialEnabled(*q == '1');
            return true;
        }
        else if (q = strstr(pCmd, SET_SERIALONLOOP))
        {
            q += sizeof(SET_SERIALONLOOP) - 1;
            while (isspace(*q)) q += 1;
            setOnloopSerialDisable(*q == '1');
            return true;
        }
        else if (q = strstr(pCmd, SET_THRESHOLD))
        {
            q += sizeof(SET_THRESHOLD) - 1;
            uint8_t v = strtol(q, 0, 16);
            setRaingaugeSwop(v);
            if (enableSerial)
            {
                Serial.print("Set threshold=");
                Serial.println(si7210.setSwOp(v));
            }
            return true;
        }
        else if (q = strstr(pCmd, SET_HYSTERESIS))
        {
            q += sizeof(SET_HYSTERESIS) - 1;
            uint8_t v = strtol(q, 0, 16);
            setRaingaugeSwHyst(v);
            if (enableSerial)
            {
                Serial.print("Set hyteresis=");
                Serial.println(si7210.setSwHyst(v));
            }
            return true;
        }
        else if (q = strstr(pCmd, SET_TOOTP))
        {
            setRaingaugeSwHyst(0xff);
            setRaingaugeSwop(0xff);
            si7210.resetToOTP();
            si7210.wakeup(); // resetToOTP set sleep()
            return true;
        }
        else if (q = strstr(pCmd, DUMP_RFM69))
        {
            radio.SPIon();
            radio.readAllRegs();
            return true;
        }
        return false;
    }
}

void loop()
{
    static bool TransmittedSinceSleep = false;
    unsigned long now = millis();

#if defined(USE_RFM69) && !defined(SLEEP_RFM69_ONLY)
    // RECEIVING
    // In this section, we'll check with the RFM69HCW to see
    // if it has received any packets:

    if (radioOK && radio.receiveDone()) // Got one!
    {
        // Print out the information:
        TimeOfWakeup = now; // extend sleep timer
#if defined(USE_SERIAL)
        if (enableSerial)
        {
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
        }
#endif
        // RFM69 ensures trailing zero byte, unless buffer is full...so
        radio.DATA[sizeof(radio.DATA) - 1] = 0; // ...if buffer is full, ignore last byte
        if (processCommand((const char*)&radio.DATA[0]))
        {
#if defined(USE_SERIAL)
            if (enableSerial)
                Serial.println("Received command accepted");
#endif
        }
        if (radio.ACKRequested())
        {
            radio.sendACK();
#if defined(USE_SERIAL)
            if (enableSerial)
                Serial.println("ACK sent");
#endif
        }
    }
#endif
	
#if defined(USE_SERIAL)
    // Set up a "buffer" for characters that we'll send:
    static char sendbuffer[62];
    static int sendlength = 0;
    // In this section, we'll gather serial characters and
    // send them to the other node if we (1) get a carriage return,
    // or (2) the buffer is full (61 characters).

    // If there is any serial input, add it to the buffer:

    if (enableSerial && Serial.available() > 0)
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
                if (enableSerial)
                {
                    Serial.print(sendbuffer);
                    Serial.println(" command accepted for rain gauge");
                }
            }
            else if (radioConfiguration.ApplyCommand(sendbuffer))
            {
                if (enableSerial)
                {
                    Serial.print(sendbuffer);
                    Serial.println(" command accepted for radio");
                }
            }
            if (enableSerial)
                Serial.println("ready");
            sendlength = 0; // reset the packet
        }
    }
#endif

    /* Using the si7210 manufacturer defaults for threshold and hysteresis 
    ** controlling its output pin, experiments in the raingauge assembly gave 
    ** some false positive output toggles when the rocker toggles.
    ** This sketch, to avoid those false positive counts, does not necessarily 
    ** notify the gateway on each output toggle. Instead, this sketch enables the 
    ** device to sleep and then interrupt (so battery life benefits 
    ** from the very low power sleep mode) but sends rocker notifications based 
    ** on the magnitude of the value of the magnetic field as read from the device.
    ** On assembly, the magnet should be arranged so that its magnetic axis 
    ** penetrates the sensor and maxes out its reading at plus or minus 
    ** 16384. This sketch notifies when the output pin wakes it up, and, within
    ** a few hundred milliseconds, the reading either exceedes NearThreshold (below),
    ** or is below FarThreshold. It withholds notification even when 
    ** the device interrupts, if the reading is between those, or has not switched 
    ** from one extreme to the other.
    */

    bool rainActivated = false;
    
    // if magnetic sensor interrupt is active, switch it to its other sense
    if (digitalRead(ROCKER_INPUT_PIN) == LOW)
    { /* I only have one job under this funnel, and I'm going to do it.*/
        si7210.toggleOutputSense(); // do this only once per wakeup
        for (int j = 0; j < MAX_WAIT_FOR_TOGGLE_MSEC; j++)
        {
            delay(1); // give ROCKER_INPUT_PIN time to respond
            if (digitalRead(ROCKER_INPUT_PIN) != LOW)
            {
                rainActivated = true; // note: this is overridden below if the Far/Near logic is compiled
                break; // normally this happens with j == 0
            }
        }
        TimeOfWakeup = now; // extend sleep timer
    }

    si7210.one();
    delayMicroseconds(si7210.CONVERSION_TIME_MICROSECONDS);
    Si7210::MagField_t magField;
    
    for (int i = 0;;)
    {
        magField = si7210.readMagField();
        if (magField != Si7210::INVALID_FIELD_READING)
            break;
        if (++i >= MAX_MAGFIELD_POLL)
            return; // do loop() again
    }

#if 1 // compute rainActivated using the magnetic field amplitude rather than the si7210 output pin
    const Si7210::MagField_t FarThreshold = FAR_THRESHOLD(Si7210::getMaxAmplitude());
    const Si7210::MagField_t NearThreshold = NEAR_THRESHOLD(Si7210::getMaxAmplitude());
    Si7210::MagField_t amplitude = magField;
    if (amplitude < 0)
        amplitude = -amplitude;
    bool MagIsClose = amplitude > NearThreshold;
    bool MagIsFar = amplitude < FarThreshold;

#if defined(SERIAL_DEBUG_OUTPUT) && defined(USE_SERIAL) // for debugging
    static auto lastPrinted = millis();
    static const unsigned PRINT_INTERVAL_MSEC = 200;
    auto toPrint = millis();
    if (toPrint - lastPrinted > PRINT_INTERVAL_MSEC)
    {
        Serial.print("magField = "); Serial.println(magField);
    }
#endif
     
    rainActivated = !(!MagIsClose && !MagIsFar); // only report farther than FAR_THRESHOLD or nearer than NEAR_THRESHOLD
    if (!rainActivated)
        TimeOfWakeup = now; // extend sleep timer while magfield is "in between"
    else
        rainActivated = prevSentMagnetClose ^ MagIsClose; // only report when its changed
    if (rainActivated)
        prevSentMagnetClose = MagIsClose;
#endif

#if defined(SERIAL_DEBUG_OUTPUT) && defined(USE_SERIAL) // for debugging
    if (toPrint - lastPrinted > PRINT_INTERVAL_MSEC)
    {
    Serial.print("rainActivated = "); Serial.println(static_cast<int>(rainActivated));
    lastPrinted = toPrint;
    }
#endif

    if (rainActivated ||
        (!TransmittedSinceSleep && (now - TimeOfWakeup >= MONITOR_ROCKER_MSEC)))
    {   
        TransmittedSinceSleep = true; /* Transmit at least once per wakeup. */
        int batt(0);
#if defined(TELEMETER_BATTERY_V)
        // 10K to VCC and (wired on board) 2.7K to ground
        pinMode(BATTERY_PIN, INPUT_PULLUP); // sample the battery
        batt = analogRead(BATTERY_PIN);
        /* batt result means little in absolute terms, but its
        ** history is telling. Failing cells have a pattern.*/
        pinMode(BATTERY_PIN, INPUT); // turn off battery drain
#endif
        // read temperature data
        auto temperature = tmp175.finishReadTempCx16();

        char sign = '+';
        if (temperature < 0) {
            temperature = -temperature;
            sign = '-';
        }
        else if (temperature == 0.f)
            sign = ' ';

        int whole = temperature >> 4;
        int frac = temperature & 0xF;
        frac *= 100;
        frac >>= 4;

        static char buf[64];
        sprintf(buf, "C:%u, B:%d, T:%c%d.%02d, RG: %d F: %d",
            sampleCount++,
            batt,
            sign, whole,
            frac,
            static_cast<int>(rainActivated ? 1 : 0),
            static_cast<int>(magField)
        );
#if defined(USE_SERIAL)
        if (enableSerial)
            Serial.println(buf);
#endif
#if defined(USE_RFM69) && !defined(SLEEP_RFM69_ONLY)
        if (radioOK)
            radio.sendWithRetry(GATEWAY_NODEID, buf, strlen(buf));
#endif
    }

    if (now - TimeOfWakeup > ListenAfterTransmitMsec)
    {	// go to sleep. with R1/C1 as specified will be about 100 seconds
        TransmittedSinceSleep = false;
        if (getOnloopSerialDisable())
            SetSerialEnabled(false);
        SleepTilNextInterrupt();
        TimeOfWakeup = millis();
        ListenAfterTransmitMsec = NormalListenAfterTransmit;
    }

    #if 0
    #if defined(USE_SERIAL)
    {
        auto pinInt = digitalRead(RC_RLY_INTERRUPT_PIN);
        auto pinSi7210 = digitalRead(ROCKER_INPUT_PIN);
        Serial.print("pinInt = ");
        Serial.print(pinInt);
        Serial.print(", pin7210=");
        Serial.println(pinSi7210);
    }
    #endif
    #endif
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
        if (enableSerial)
        {
            Serial.print("sleep for count=");
            Serial.println(SleepLoopTimerCount);
            Serial.flush();// wait for finish and turn off pins before sleep
            Serial.end();
        }
        else
        {
            // hold TXD steady 
            pinMode(0, INPUT); // Arduino libraries have a symbolic definition for Serial pins?
            digitalWrite(TXD_PIN, HIGH);
            pinMode(TXD_PIN, OUTPUT); // TXD hold steady
        }
#endif

#if defined(USE_RFM69) && !defined(SLEEP_RFM69_ONLY)
        if (radioOK)
            radio.SPIoff();
#endif

#if defined(TELEMETER_BATTERY_V)
        analogReference(EXTERNAL); // This sequence drops idle current by 30uA
        analogRead(BATTERY_PIN); // doesn't shut down the band gap until we USE ADC
#endif

        si7210.sleep(false);

        // sleep mode power supply current measurements indicate this appears to be redundant
        power_all_disable(); // turn off everything

        unsigned count = 0;

#if !defined(SLEEP_WITH_TIMER2)
        // this uses R1 and C1 to ground
        while (count < SleepLoopTimerCount)
        {
            if (digitalRead(ROCKER_INPUT_PIN) == LOW)
                break;
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
            sleep_cpu(); 
            sleep_disable();
            sei();
            count += 1;
        }
#endif

        power_all_enable();

#if defined(TELEMETER_BATTERY_V)
        ResetAnalogReference();
#endif

#if defined(USE_SERIAL)
        if (enableSerial)
        {
            Serial.begin(SERIAL_PORT_BAUDS);
            Serial.println(F("******waked up*******"));
        }
#endif

#if defined(USE_RFM69) && !defined(SLEEP_RFM69_ONLY)
        if (radioOK)
            radio.SPIon();
#endif
        tmp175.startReadTemperature();
        si7210.wakeup();
        si7210.one(); // Output pin check won't be updated in loop() without this
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
