/* Packet rain gauge
* This sketch is to run on a Pro Mini on a PCB with a hall effect magnet sensor.
* It originates a packet for both the arrival and departure of a magnet
* to its sensor. The usual application is to mount a magnet on
* a rocker under a rain gauge funnel.
* 
* This design also reports a temperature from a TMP175 sensor and a battery 
* voltage level. It also originates packets on a timed interval in the absence of rainfail.
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
#define TELEMETER_BATTERY_V

// because the LED is on the SPI clock (SPK) line, access to the RFM69 flashes it dimly
//#define FLASH_LED_ON_STARTUP // for debugging. flashes it longer

#define FAR_THRESHOLD(X) (X)/32
#define NEAR_THRESHOLD(x) (x)/6

#define REVISION "REV09"
#define PCB_REV_NUMBER 3 // Sketch supports versions 2 or 3 only

// Only one of the following sensors may be defined
#define USE_AH1383
//#define USE_S7210

#if defined(USE_RFM69)
#include <RFM69.h>
#include <RFM69registers.h>
#endif

#if PCB_REV_NUMBER >= 3 || !defined(USE_AH1383)
#define EXPERIMENTAL_AH1383_ON_PCBV2 0
#else
/* Special case for the newly supported AH1383 on the older, Version 2 PCB.
** EXPERIMENTAL_AH1383_ON_PCBV2 is not a viable solution for permanent install because it telemeters 
** NOTHING while the bucket magnet is direcly over the sensor. But it works normally if the bucket 
** happens to come to rest at the stop far away from the magnet.*/
#define EXPERIMENTAL_AH1383_ON_PCBV2 1 
#endif

//signed/unsigned arithmetic is involved in timer compares and easy to get wrong
typedef  decltype(millis()) msec_time_stamp_t;
typedef  long msec_time_diff_t;
static_assert(sizeof(msec_time_diff_t) == sizeof(msec_time_stamp_t), "time stamp and time diff must have same precision");
template <typename D>
bool TimerCompleted(msec_time_stamp_t now, msec_time_stamp_t started, D interval)
{ return static_cast<msec_time_diff_t>(now - started) >= static_cast<msec_time_diff_t>(interval);}

class Pcb3Si7210
{   // The Si7210 is an I2C part that can read mag fields and has various parameters
    // Its "alert" output is wired to ROCKER_INPUT_PIN
 public:
    Pcb3Si7210() : si7210(0x33)
    {}
    bool isSleeping() const { return si7210.isSleeping();}
    bool isInterrupting();
    void wakeup() {  si7210.wakeup();}
    void one() {  si7210.one();}
    uint8_t sleep(bool fromOTP = true) { return si7210.sleep(fromOTP);}
    void resetToOTP() {  si7210.resetToOTP();}
    int16_t toggleOutputSense(){return si7210.toggleOutputSense();}
    uint16_t setSwHyst(uint8_t v){return si7210.setSwHyst(v);}
    uint16_t setSwOp(uint8_t v){return si7210.setSwOp(v);}

    void Setup();
    Si7210::MagField_t PollOutput();
    bool loop(unsigned long, bool &rainActivated, Si7210::MagField_t&magField);
    void dump(){ si7210.dump();}
protected:
    static uint16_t threshold(uint8_t sw_op) { return Si7210::threshold(sw_op);}
    static uint16_t hysteresis(uint8_t sw_hyst){return Si7210::hysteresis(sw_hyst);}
    Si7210::MagField_t readMagField(){return si7210.readMagField();}
    void setOutputSense(bool v){si7210.setOutputSense(v);}
    uint8_t getSwTamper(){return si7210.getSwTamper();}
     uint8_t getSwOp(){return si7210.getSwOp();}
    void SetupSwHyst(uint8_t hyst);
    void setFieldPolSel(uint8_t v){return si7210.setFieldPolSel(v);}
    uint8_t getSwHyst(){return si7210.getSwHyst();}
    static Si7210::MagField_t getMaxAmplitude(){return Si7210::getMaxAmplitude();}
    void FinishSetup();
    void SetupSwOp(uint8_t swop);
    void DoReadMode(unsigned long, Si7210::MagField_t);
    bool CheckThresholds(unsigned long now,Si7210::MagField_t);
    
    Si7210 si7210;
};

class Ah1383 {
    // The AH1383 is a hall effect uni-polar switch that is either ON or OFF,
    // which is read on the ROCKER_INPUT_PIN
    public:
    int16_t toggleOutputSense();
    bool loop(unsigned long, bool &rainActivated, Si7210::MagField_t&magField);
    void Setup(){};
    bool isInterrupting();
    uint16_t setSwOp(uint8_t v){return 0;}
    uint16_t setSwHyst(uint8_t v){return 0;}
    void resetToOTP(){}
    void wakeup(){}
    void one(){}
    uint8_t sleep(bool=false){return 0;}
    protected:
    void DoReadMode(unsigned long, Si7210::MagField_t);
};

#if defined(USE_AH1383)
typedef Ah1383 Sensor_t;
#elif defined(USE_S7210)
typedef Pcb3Si7210 Sensor_t;
#endif

namespace {
    enum class EepromAddresses {
        PACKET_RAINGAUGE_START = RadioConfiguration::EepromAddresses::TOTAL_EEPROM_USED,
        PACKET_RAINGAUGE_SLEEP_LOOP_COUNT = PACKET_RAINGAUGE_START,
        PACKET_RAINGAUGE_DISABLE_SERIAL = PACKET_RAINGAUGE_SLEEP_LOOP_COUNT + sizeof(unsigned),
        PACKET_RAINGAUGE_SWOP = PACKET_RAINGAUGE_DISABLE_SERIAL + 1,
        PACKET_RAINGAUGE_SWHYST = PACKET_RAINGAUGE_SWOP + 1,
        PACKET_RAINGAUGE_NEAR = PACKET_RAINGAUGE_SWHYST + 1,
        PACKET_RAINGAUGE_FAR = PACKET_RAINGAUGE_NEAR + sizeof(Si7210::MagField_t),
        PACKET_RAINGAUGE_SIGNED = PACKET_RAINGAUGE_FAR + sizeof(Si7210::MagField_t),
        PACKET_RAINGAUGE_END = PACKET_RAINGAUGE_SIGNED + 1
    };
    const int BATTERY_PIN = A0; // digitize (fraction of) battery voltage
    const int TIMER_RC_CHARGE_PIN = 8; // sleep uProc using RC circuit on this pin
    const int TXD_PIN = 1;
    const int RC_RLY_INTERRUPT_PIN = 3;
 #if PCB_REV_NUMBER >= 3
    const int RC_STATUS_PIN = 17;   // not used, but on REV02 of the PCB
    const int ROCKER_INPUT_PIN = 16;
#elif PCB_REV_NUMBER <= 2   // these pin assignments are reversed on REV02 vs REV03
    const int RC_STATUS_PIN = 16;
    const int ROCKER_INPUT_PIN = 17;
#endif
   const int SENSOR_INTERRUPT_INVERT_PIN = 7;

    /* The Pro Mini has only two interrupt pins that have the necessary features
    ** for responding to the three sources in this design, INT0 and INT1:
    ** 1. the RFM69
    ** 2. the R1/C1 sleep timer
    ** 3. the hall effect sensor AL pin that indicates the approach or departure of a magnet
    **
    ** The RFM69 is on INT0
    ** INT1 is shared by the RC circuit and the hall effect through the NOR at U11.
    ** Both signals are also routed to dedicated input pins on the Arduino, D17 and D16, respectively,
    ** so the interrupt handler on the sketch can distinguish the cause.
    */

    // after "listening" to Serial and RFM69 for as long as below, go to very low power sleep until INT0 or INT1
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
    const int RFM69_CHIP_SELECT_PIN = 10;
    const int RFM69_INT_PIN = 2;
    const uint8_t GATEWAY_NODEID = 1;
 
    // Create a library object for our RFM69HCW module:
    RFM69 radio(RFM69_CHIP_SELECT_PIN, RFM69_INT_PIN, true);
#endif

    int16_t FarThreshold = FAR_THRESHOLD(Si7210::getMaxAmplitude());
    int16_t NearThreshold = NEAR_THRESHOLD(Si7210::getMaxAmplitude());
    uint8_t SignedThreshold = 0xff;

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
    void setNearThreshold(int16_t v)
    {
        int addr = static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_NEAR);
        EEPROM.put(addr, v);
        NearThreshold = v;
    }
    void setFarThreshold(int16_t v)
    {
        int addr = static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_FAR);
        EEPROM.put(addr, v);
        FarThreshold = v;
     }
    void setSignedThreshold(uint8_t v)
    {
        int addr = static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_SIGNED);
        EEPROM.put(addr, v);
        SignedThreshold = v;
     }


#if defined(TELEMETER_BATTERY_V)
    void ResetAnalogReference();
#endif
    const unsigned MAX_SLEEP_LOOP_COUNT = 5000;
    const unsigned MONITOR_ROCKER_MSEC = 100; // go at least this long after waking before assuming it was RC was the cause
    const int MAX_WAIT_FOR_TOGGLE_MSEC = 100;
    const long SERIAL_PORT_BAUDS = 38400;
    const int MAX_MAGFIELD_POLL = 10;

    RadioConfiguration radioConfiguration;
    unsigned long TimeOfWakeup;
    unsigned SleepLoopTimerCount = 30; // approx R1 * C1 seconds per Count (= 100seconds)

    TMP175 tmp175(0b1001000); //0b1001000 per TMP175 docs, is I2C address with all addressing pins low.

    Sensor_t magSensor;
    bool prevSentMagnetClose;
    bool enableSerial = true;
    bool radioOK = false;
    auto ModeStart = millis();
    uint32_t ModeInterval;
    bool readMode = false;
}

namespace {
        const char SET_LOOPCOUNT[] = "SetDelayLoopCount";
        const char SET_SERIAL[] = "SetSerial";
        const char SET_THRESHOLD[] = "SetSWOP"; // HEX number range 00 : 7F where 7f is special case of "latch mode" where  
        const char SET_HYSTERESIS[] = "SetSWHYST"; // argument is HEX number range 00 : 3F, 3F is special case for ZERO
        const char SET_TOOTP[] = "SetToOTP";
        const char VER[] = "VER";
        const char DUMP_RFM69[] = "DumpRFM69";
        const char SET_SERIALONLOOP[] = "DisableSerialOnLoop";
        const char SET_NEAR_THRESHOLD[] = "SetNearThreshold";
        const char SET_FAR_THRESHOLD[] = "SetFarThreshold";
        const char SET_SIGNED_THRESHOLD[] = "SetSignedThreshold";
        const char READ_MODE_SECONDS[] = "ReadModeForSeconds";
}

void setup()
{
    // Open a serial port so we can send keystrokes to the module:
    enableSerial = true;
    Serial.begin(SERIAL_PORT_BAUDS);
    delay(100);
    Serial.print("Packet Rain Gauge " REVISION " for PCB V");
    Serial.println(static_cast<int>(PCB_REV_NUMBER));
    #if defined(USE_AH1383)
    Serial.println(F("AH1383 sensor"));
    #elif defined(USE_S7210)
    Serial.println(F("Si7210 sensor"));
    #endif
    Serial.print("Node ");
    Serial.print(radioConfiguration.NodeId(), DEC);
    Serial.print(" on network ");
    Serial.print(radioConfiguration.NetworkId(), DEC);
    Serial.print(" band ");
    Serial.print(radioConfiguration.FrequencyBandId(), DEC);
    Serial.print(" key ");
    radioConfiguration.printEncryptionKey(Serial);
    Serial.println(" ready");

#if defined(FLASH_LED_ON_STARTUP)
        pinMode(LED_BUILTIN, OUTPUT);
        for (int i = 0; i < 10; i++)
        {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        }
        pinMode(LED_BUILTIN, INPUT);
#endif

#if defined(USE_RFM69)
    pinMode(RFM69_CHIP_SELECT_PIN, OUTPUT);
    digitalWrite(RFM69_CHIP_SELECT_PIN, HIGH);
    SPI.begin();

    // Initialize the RFM69HCW:
    auto nodeId = radioConfiguration.NodeId();
    auto networkId = radioConfiguration.NetworkId();
    auto fbId = radioConfiguration.FrequencyBandId();
    radioOK = nodeId != 0xff &&
            networkId != 0xff &&
            fbId != 0xff &&
            radio.initialize(fbId, nodeId, networkId);
    Serial.println(radioOK ? "Radio init OK" : "Radio init failed");
    if (radioOK)
    {
        uint32_t freq;
        if (radioConfiguration.FrequencyKHz(freq))
            radio.setFrequency(1000*freq);
        Serial.print("Freq= "); Serial.print(radio.getFrequency()/1000); Serial.println(" KHz");
    }
 
    if (radioOK)
    {
        radio.setHighPower(); // Always use this for RFM69HCW
    // Turn on encryption if desired:
        if (radioConfiguration.encrypted() && ENCRYPT)
            radio.encrypt(radioConfiguration.EncryptionKey());
    }
#endif

    pinMode(RC_RLY_INTERRUPT_PIN, INPUT);
    pinMode(ROCKER_INPUT_PIN, INPUT);
    pinMode(SENSOR_INTERRUPT_INVERT_PIN, OUTPUT);
    digitalWrite(SENSOR_INTERRUPT_INVERT_PIN, LOW); // Sensor set not inverted

#if defined(TELEMETER_BATTERY_V)
    ResetAnalogReference();
#endif

    TimeOfWakeup = millis(); // start loop timer now

    unsigned eepromLoopCount(0);
    EEPROM.get(static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_SLEEP_LOOP_COUNT), eepromLoopCount);
    if (eepromLoopCount && eepromLoopCount <= MAX_SLEEP_LOOP_COUNT)
    	SleepLoopTimerCount = eepromLoopCount;

    Serial.print("SleepLoopTimerCount = ");
    Serial.println(SleepLoopTimerCount);

    Wire.begin();
    tmp175.setup();
    magSensor.Setup();
 
    if (prevSentMagnetClose = (digitalRead(ROCKER_INPUT_PIN) == LOW))
    {
        magSensor.toggleOutputSense();
        Serial.println("Magnet close on startup.");
        delay(MAX_WAIT_FOR_TOGGLE_MSEC);
    }

    Serial.print(SET_SERIALONLOOP);
    Serial.print(" ");
    Serial.println(getOnloopSerialDisable() ? "ON" : "OFF");

    uint16_t temp;
    EEPROM.get(static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_NEAR), temp);
    if (temp < Si7210::getMaxAmplitude())
        NearThreshold = temp;
    EEPROM.get(static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_FAR), temp);
    if (temp < Si7210::getMaxAmplitude())
        FarThreshold = temp;
    EEPROM.get(static_cast<uint16_t>(EepromAddresses::PACKET_RAINGAUGE_SIGNED), SignedThreshold);
 
#if defined(USE_S7210)
    Serial.print("Theshold Near="); Serial.println(NearThreshold);
    Serial.print("Theshold Far="); Serial.println(FarThreshold);
    Serial.print("Theshold Signed="); Serial.println((int)SignedThreshold);
#endif

    tmp175.startReadTemperature();
    Serial.println("Setup complete");
}

/* Power management:
 * For ListenAfterTransmitMsec we stay awake and listen on the radio and Serial.
 * Then we power down all: temperature sensor, radio and CPU and CPU
 * sleep using SleepTilNextInterrupt. */

namespace {
    unsigned long ListenAfterTransmitMsec = FirstListenAfterTransmitMsec;
    unsigned int sampleCount;

    unsigned SleepTilNextInterrupt();

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
        }else if (strncmp(pCmd, VER, sizeof(VER) - 1) == 0)
        {
            Serial.println("Revision 03");
            return true;
        }else if (q = strstr(pCmd, SET_SERIAL))
        {
            q += sizeof(SET_SERIAL)-1;
            while (isspace(*q)) q += 1;
            SetSerialEnabled(*q == '1');
            return true;
        }else if (q = strstr(pCmd, SET_SERIALONLOOP))
        {
            q += sizeof(SET_SERIALONLOOP) - 1;
            while (isspace(*q)) q += 1;
            setOnloopSerialDisable(*q == '1');
            return true;
        }
#if defined(USE_S7210)
        else if (q = strstr(pCmd, SET_THRESHOLD))
        {
            q += sizeof(SET_THRESHOLD) - 1;
            uint8_t v = strtol(q, 0, 0);
            setRaingaugeSwop(v);
            if (enableSerial)
            {
                Serial.print("Set threshold=");
                Serial.println(magSensor.setSwOp(v));
            }
            return true;
        }else if (q = strstr(pCmd, SET_HYSTERESIS))
        {
            q += sizeof(SET_HYSTERESIS) - 1;
            uint8_t v = strtol(q, 0, 0);
            setRaingaugeSwHyst(v);
            if (enableSerial)
            {
                Serial.print("Set hyteresis=");
                Serial.println(magSensor.setSwHyst(v));
            }
            return true;
        }else if (q = strstr(pCmd, SET_NEAR_THRESHOLD))
        {
            q += sizeof(SET_NEAR_THRESHOLD) - 1;
            int16_t v = strtol(q, 0, 0);
            setNearThreshold(v);
            return true;
        }else if (q = strstr(pCmd, SET_FAR_THRESHOLD))
        {
            q += sizeof(SET_FAR_THRESHOLD) - 1;
            int16_t v = strtol(q, 0, 0);
            setFarThreshold(v);
            return true;
        }else if (q = strstr(pCmd, SET_SIGNED_THRESHOLD))
        {
            q += sizeof(SET_SIGNED_THRESHOLD) - 1;
            uint16_t v = strtol(q, 0, 0);
            setSignedThreshold(v);
            return true;
        }else if (q = strstr(pCmd, SET_TOOTP))
        {
            setRaingaugeSwHyst(0xff);
            setRaingaugeSwop(0xff);
            magSensor.resetToOTP();
            magSensor.wakeup(); // resetToOTP set sleep()
            return true;
        }
#endif
        else if (q = strstr(pCmd, DUMP_RFM69))
        {
            radio.readAllRegs();
            return true;
        }else if (q = strstr(pCmd, READ_MODE_SECONDS))
        {
            q += sizeof(READ_MODE_SECONDS) - 1;
            auto v = static_cast<uint16_t>(strtol(q, 0, 0));
            if (v > 0)
            {
                ModeStart = millis();
                ModeInterval = 1000u * v;
                readMode = true;
            }
            return true;
        }
        return false;
    }

    bool MonitorRockerInput(unsigned long);
}

void loop()
{
    static bool TransmittedSinceSleep = false;
    unsigned long now = millis();

#if defined(USE_RFM69)
    // RECEIVING
    // In this section, we'll check with the RFM69HCW to see
    // if it has received any packets:

    if (radioOK && radio.receiveDone()) // Got one!
    {
        // Print out the information:
        TimeOfWakeup = now; // extend sleep timer

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

        // RFM69 ensures trailing zero byte, unless buffer is full...so
        radio.DATA[sizeof(radio.DATA) - 1] = 0; // ...if buffer is full, ignore last byte
        if (processCommand((const char*)&radio.DATA[0]))
        {
            if (enableSerial)
                Serial.println("Received command accepted");
        }
        if (radio.ACKRequested())
        {
            radio.sendACK();
            if (enableSerial)
                Serial.println("ACK sent");
        }
    }
#endif
	
    // Set up a "buffer" for characters that we'll send:
    static char sendbuffer[62];
    static int sendlength = 0;
    // In this section, we'll gather serial characters and
    // send them to the other node if we (1) get a carriage return,
    // or (2) the buffer is full (61 characters).

    // If there is any serial input, add it to the buffer:

    while (enableSerial && Serial.available() > 0)
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
   
    bool rainActivated = MonitorRockerInput(now);
    
    Si7210::MagField_t magField = 0;
    if (magSensor.loop(now, rainActivated, magField))
        return; // not ready. loop() again

    if (rainActivated ||
        (!TransmittedSinceSleep && TimerCompleted(now , TimeOfWakeup, MONITOR_ROCKER_MSEC)))
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
        if (enableSerial)
            Serial.println(buf);
#if defined(USE_RFM69) 
        if (radioOK)
            radio.sendWithRetry(GATEWAY_NODEID, buf, strlen(buf));
#endif
    }

    if (TimerCompleted(now, TimeOfWakeup, ListenAfterTransmitMsec))
    {	// go to sleep. with R1/C1 as specified will be about 100 seconds
        TransmittedSinceSleep = false;
        if (getOnloopSerialDisable())
            SetSerialEnabled(false);
        SleepTilNextInterrupt();
        TimeOfWakeup = millis();
        ListenAfterTransmitMsec = NormalListenAfterTransmit;
    }
}

void sleepPinInterrupt()	// requires 10uF and 10M between two pins
{
    // run at interrupt level. disables further interrupt from this source
    detachInterrupt(digitalPinToInterrupt(RC_RLY_INTERRUPT_PIN));
}

namespace {
    unsigned SleepTilNextInterrupt()
    {
        if (enableSerial)
        {
            Serial.print("sleep for count=");
            Serial.println(SleepLoopTimerCount);
            Serial.flush();// wait for finish and turn off pins before sleep
            Serial.end();
        }
        // regardless of whether the Serial is going to be re-enabled, hold TX steady
        // so if a port is attached, the PC doesn't get (as much) garbage.
        pinMode(0, INPUT); // Arduino libraries have a symbolic definition for Serial pins?
        digitalWrite(TXD_PIN, HIGH);
        pinMode(TXD_PIN, OUTPUT); // TXD hold steady

#if defined(USE_RFM69)
        if (radioOK)
            radio.sleep();
#endif

#if defined(TELEMETER_BATTERY_V)
        auto saveADCSRA = ADCSRA;
        ADCSRA = 0; // Turn off ADC
#endif

        magSensor.sleep(false);

        power_all_disable(); // turn off everything

        unsigned count = 0;

        // this uses R1 and C1 to ground
        while (count < SleepLoopTimerCount)
        {
            #if EXPERIMENTAL_AH1383_ON_PCBV2==0 // this is the "normal" code. Test code is below
            if (magSensor.isInterrupting())
                break;
            power_timer0_enable(); // delay() requires this
            pinMode(TIMER_RC_CHARGE_PIN, OUTPUT);
            digitalWrite(TIMER_RC_CHARGE_PIN, HIGH);
            delay(10); // Charge the Cap (1uF to 10uF)
            pinMode(TIMER_RC_CHARGE_PIN, INPUT);
            cli();
            power_timer0_disable(); // timer0 powered down again
            attachInterrupt(digitalPinToInterrupt(RC_RLY_INTERRUPT_PIN), sleepPinInterrupt, LOW);         
            
            #else  /*Testing an AH1383 on the older PCB
            ** The RC based transmit events only happen with the bucket magnet AWAY.
            ** While the magnet is close, we sleep waiting for the interrupt on HIGH level, */
            bool hallEffectActive = digitalRead(ROCKER_INPUT_PIN) == LOW;
            power_timer0_enable(); // delay() requires this
            pinMode(TIMER_RC_CHARGE_PIN, OUTPUT);
            digitalWrite(TIMER_RC_CHARGE_PIN, HIGH);
            delay(10); // Charge the 1uF
            if (!hallEffectActive) // Disable the RC timer while hall effect active
                pinMode(TIMER_RC_CHARGE_PIN, INPUT);
            cli();
            power_timer0_disable(); // timer0 powered down again
            attachInterrupt(digitalPinToInterrupt(RC_RLY_INTERRUPT_PIN), sleepPinInterrupt, hallEffectActive? HIGH: LOW);
            #endif

            set_sleep_mode(SLEEP_MODE_PWR_DOWN);
            sleep_enable();
            sleep_bod_disable();
            sei();
            sleep_cpu(); 
            sleep_disable();
            sei();
            count += 1;
        }

        power_all_enable();

#if defined(TELEMETER_BATTERY_V)
        ADCSRA = saveADCSRA;
        ResetAnalogReference();
#endif

        if (enableSerial)
        {
            Serial.begin(SERIAL_PORT_BAUDS);
            Serial.println(F("******waked up*******"));
        }

        tmp175.startReadTemperature();
        magSensor.wakeup();
        magSensor.one(); // Output pin check won't be updated in loop() without this
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

// Pcb3Si7210 methods********************************************************************
    void Pcb3Si7210::Setup() 
    {
        auto OTPthreshold = si7210.setup();
        Serial.print("si7210 OTP threshold=");
        Serial.println(OTPthreshold);
    
        auto swop = getRaingaugeSwop();
        if (swop != 0xffu)
            SetupSwOp(swop);
    
        auto hyst = getRaingaugeSwHyst();
        SetupSwHyst(hyst);
        FinishSetup();
    }
    bool Pcb3Si7210::isInterrupting()
    {
        return digitalRead(ROCKER_INPUT_PIN) == LOW;
    }
    void Pcb3Si7210::FinishSetup()
    {
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
    }

    void Pcb3Si7210::SetupSwOp(uint8_t swop)
    {
        uint16_t threshold = si7210.setSwOp(swop);
        Serial.print(F("SWOP EEPROM raw =0x")); 
        Serial.print((int)swop, HEX);
        Serial.print(" threshold=");
        Serial.println(threshold);
    }

    void Pcb3Si7210::SetupSwHyst(uint8_t hyst)
    {
        uint16_t hysteresis;
        if (hyst != 0xffu)
        {
            hysteresis = si7210.setSwHyst(hyst);
            Serial.print(F("SWHYST EEPROM raw=0x"));
            Serial.print((int) hyst, HEX);
            Serial.print(" hysteresis = ");
        }
        else
        {
            Serial.print("OTP hysteresis = ");
            hysteresis = si7210.hysteresis(si7210.getSwHyst());
        }
        Serial.println(hysteresis);
    }

    Si7210::MagField_t Pcb3Si7210::PollOutput()
    {
        si7210.one();
        delayMicroseconds(Si7210::CONVERSION_TIME_MICROSECONDS);
        Si7210::MagField_t magField;  
        for (int i = 0;;)
        {
            magField = si7210.readMagField();
            if (magField != Si7210::INVALID_FIELD_READING)
                break;
            if (++i >= MAX_MAGFIELD_POLL)
                break; // do loop() again
        }
        return magField;
   }

void Pcb3Si7210::DoReadMode(unsigned long now, Si7210::MagField_t magField)
{
    if (TimerCompleted(now, ModeStart, ModeInterval))
        readMode = false;
    else
    {
        if (enableSerial)
        {
            TimeOfWakeup = now;
            auto v = digitalRead(RC_RLY_INTERRUPT_PIN);
            Serial.print(F("Int="));
            Serial.print(v);
            Serial.print(F(" Rocker="));
            v = digitalRead(ROCKER_INPUT_PIN);
            Serial.print(v);
            Serial.print(F(" Magfield: "));
            Serial.println(magField);
        }
    }
}

bool Pcb3Si7210::CheckThresholds(unsigned long now, Si7210::MagField_t magField)
{   /* Using the si7210 manufacturer defaults for threshold and hysteresis 
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
    ** from one extreme to the other.  */

    bool MagIsClose(false);
    bool MagIsFar(false);
    switch (SignedThreshold)
    {
        case 0: // positive
            MagIsClose = magField > NearThreshold;
            MagIsFar = magField < FarThreshold;
            break;
        case 1: // negative
            MagIsClose = magField < NearThreshold;
            MagIsFar = magField > FarThreshold;
            break;
        default:
            Si7210::MagField_t amplitude = magField;
            if (amplitude < 0)
                amplitude = -amplitude;
            MagIsClose = amplitude > NearThreshold;
            MagIsFar = amplitude < FarThreshold;
        break;
    }

    // compute rainActivated using the magnetic field amplitude rather than the si7210 output pin 
    bool rainActivated = !(!MagIsClose && !MagIsFar); // only report farther than FAR_THRESHOLD or nearer than NEAR_THRESHOLD
    static const int MAX_MAG_SAVED_BETWEEN = 1 << 2;  // Must be a power of 2 for opimized modular arithmetic
    static uint8_t newestMagIndex = 0;
    static uint8_t oldestMagIndex = 0; // no entries when newest/oldest match
    //#define DBG_DIR
    #if defined(DBG_DIR)
    auto prevnew = newestMagIndex;
    auto prevold = oldestMagIndex;
    #endif
    if (!rainActivated)
    {   // extend the wakeup timer if we observe a rate of change...
        static const Si7210::MagField_t MinChangeToExtendTimer = 400; // ... by this or more
        static const Si7210::MagField_t MinChangetoSave = 20; // keep list of mags different by this much
        static Si7210::MagField_t savedMagnitudesBetween[MAX_MAG_SAVED_BETWEEN];
        auto i = oldestMagIndex;
        for (; i != newestMagIndex; i += 1, i &= MAX_MAG_SAVED_BETWEEN-1)
        {
            auto diff = magField - savedMagnitudesBetween[i];
            if (diff < 0)
                diff = -diff;
            if (diff <= MinChangetoSave)
                goto not_rainActivated; // discard small diff
            else if (diff >= MinChangeToExtendTimer)
            {
                // extend sleep timer while magfield is "in between"
                newestMagIndex = oldestMagIndex = 0;
                TimeOfWakeup = now; 
#if defined(DBG_DIR)
                Serial.println("****************extending TimeOfWakeup*********");
#endif
                goto not_rainActivated;
            }
        } 
        // keep this one, discarding oldest, if necessary
        savedMagnitudesBetween[newestMagIndex] = magField;        
        newestMagIndex += 1; newestMagIndex &= MAX_MAG_SAVED_BETWEEN-1; // modular add
        if (newestMagIndex == oldestMagIndex)
        {
            oldestMagIndex += 1; oldestMagIndex &= MAX_MAG_SAVED_BETWEEN-1; // modular add
        }
     }
    else
    {
        newestMagIndex = oldestMagIndex = 0;
        rainActivated = prevSentMagnetClose ^ MagIsClose; // only report when its changed
    }
not_rainActivated:
#if defined(DBG_DIR)
    if (prevnew != newestMagIndex || prevold != oldestMagIndex)
    {
    Serial.print("oldestMagIndex="); Serial.print((int)oldestMagIndex);
    Serial.print("  newestMagIndex="); Serial.print((int)newestMagIndex);
    Serial.print("  magField="); Serial.println(magField);
    }
#endif

    if (rainActivated)
        prevSentMagnetClose = MagIsClose;
    return rainActivated;
}

bool Pcb3Si7210::loop(unsigned long now, bool &rainActivated, Si7210::MagField_t&magField)
{
    magField = PollOutput();
    if (magField == Si7210::INVALID_FIELD_READING)
        return true; // do loop() again

    if (readMode)
        DoReadMode(now, magField);

    rainActivated = CheckThresholds(now, magField);
    return false;
}

// Ah1383 methods ***********************************************************
int16_t Ah1383::toggleOutputSense()
{
#if EXPERIMENTAL_AH1383_ON_PCBV2==0
    auto v = digitalRead(SENSOR_INTERRUPT_INVERT_PIN);
    digitalWrite(SENSOR_INTERRUPT_INVERT_PIN, v == LOW);
    return v;
#else
    return 0;
#endif
}

void Ah1383::DoReadMode(unsigned long now, Si7210::MagField_t magField)
{
    if (TimerCompleted(now, ModeStart, ModeInterval))
    {
        if (readMode)
            Serial.println(F("Canceling read mode"));
        readMode = false;
    }
    else
    {
        TimeOfWakeup = now;
        bool v = digitalRead(ROCKER_INPUT_PIN) != LOW;
        bool w = digitalRead(SENSOR_INTERRUPT_INVERT_PIN) != LOW;
        Serial.print(F("Int="));
        Serial.print(v ? 1 : 0);
        Serial.print(F(" Rocker="));
        Serial.print((v^w) ? 1 : 0);
        Serial.print(F(" Magfield: "));
        Serial.println(magField);
    }
}

bool Ah1383::isInterrupting()
{
    return (digitalRead(ROCKER_INPUT_PIN) == HIGH) ^ (digitalRead(SENSOR_INTERRUPT_INVERT_PIN) == HIGH);
}

bool Ah1383::loop(unsigned long now, bool &rain, Si7210::MagField_t&magField)
{
    bool v = digitalRead(ROCKER_INPUT_PIN) != LOW;
    if (v)
        magField = 0;
    else
        magField = 1; // this part triggers on the SOUTH pole 
   if (readMode)
        DoReadMode(now, magField);    
    return false;
}

namespace {
#if EXPERIMENTAL_AH1383_ON_PCBV2==0
    bool MonitorRockerInput(unsigned long now)
    {   // if magnetic sensor interrupt is active, switch it to its other sense
        if (magSensor.isInterrupting())
        { /* I only have one job under this funnel, and I'm going to do it.*/
            auto os = magSensor.toggleOutputSense();
            for (int j = 0; j < MAX_WAIT_FOR_TOGGLE_MSEC; j++)
            {
                delay(1); // give ROCKER_INPUT_PIN time to respond
                if (!magSensor.isInterrupting())
                {
                    auto prev = prevSentMagnetClose;
                    prevSentMagnetClose = os == HIGH;
                    return prev ^ prevSentMagnetClose; // normally this happens with j == 0
                }
            }
            // With PCB3 and AH1383, getting here means there is a hardware problem
            Serial.println(F("Oops: changing SENSOR_INTERRUPT_INVERT_PIN output did not change ROCKER_PIN_INPUT"));
            TimeOfWakeup = now; // extend sleep timer
        }  
        return false;
    }
#else // Experimental only
    bool MonitorRockerInput(unsigned long now)
    {   // The AH1383 on the old board. 
        bool AH1383Active = digitalRead(ROCKER_INPUT_PIN) == LOW;
        bool ret = AH1383Active != prevSentMagnetClose;
        prevSentMagnetClose = AH1383Active;
        return ret;
    }
#endif
}
