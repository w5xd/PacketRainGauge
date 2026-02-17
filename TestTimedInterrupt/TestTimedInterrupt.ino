/* Test Timed Interrupt
*/


#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>


#define USE_SERIAL
#define TELEMETER_BATTERY_V
#define REVISION "REV01"
#define PCB_REV_NUMBER 3 // Sketch supports versions 2 or 3 only


namespace {
    
    const int BATTERY_PIN = A0; // digitize (fraction of) battery voltage
    const int TIMER_RC_CHARGE_PIN = 8; // sleep uProc using RC circuit on this pin
    const int RC_RLY_INTERRUPT_PIN = 3;
 #if PCB_REV_NUMBER >= 3
    const int RC_STATUS_PIN = 17;   // not used, but on REV02 of the PCB
    const int ROCKER_INPUT_PIN = 16;
#elif PCB_REV_NUMBER <= 2   // these pin assignments are reversed on REV02 vs REV03
    const int RC_STATUS_PIN = 16;
    const int ROCKER_INPUT_PIN = 17;
#endif
   const int SENSOR_INTERRUPT_INVERT_PIN = 7;

    bool enableSerial = true;
   

#if defined(TELEMETER_BATTERY_V)
    void ResetAnalogReference();
#endif
    const unsigned MAX_SLEEP_LOOP_COUNT = 5000;
    unsigned long TimeOfWakeup;
    unsigned SleepLoopTimerCount = 1; // approx R1 * C1 seconds per Count (= 100seconds)

     const long SERIAL_PORT_BAUDS = 38400;
 
}

void printPins()
{
    Serial.println();
    Serial.print("Interrupt 3 is ");
    Serial.println(digitalRead(RC_RLY_INTERRUPT_PIN) == LOW ? "active" : "inactive");
    Serial.print("Hall effect interrupt is ");
    Serial.println(digitalRead(ROCKER_INPUT_PIN) == LOW ? "active" : "inactive");
    Serial.print("RC_STATUS_PIN is ");
    Serial.println(digitalRead(RC_STATUS_PIN) == HIGH ? "active" : "inactive");
}

void setup()
{
#if defined(USE_SERIAL)
    // Open a serial port so we can send keystrokes to the module:
    enableSerial = true;
    Serial.begin(SERIAL_PORT_BAUDS);
    delay(100);
    Serial.print("Test Interrupt " REVISION  " PCB:");
    Serial.println(static_cast<int>(PCB_REV_NUMBER));
    Serial.println(" ready");
#endif

    pinMode(RC_RLY_INTERRUPT_PIN, INPUT);
    pinMode(ROCKER_INPUT_PIN, INPUT_PULLUP);
    pinMode(SENSOR_INTERRUPT_INVERT_PIN, OUTPUT);
    digitalWrite(SENSOR_INTERRUPT_INVERT_PIN, digitalRead(ROCKER_INPUT_PIN)); 

#if defined(TELEMETER_BATTERY_V)
    ResetAnalogReference();
#endif

    TimeOfWakeup = millis(); // start loop timer now

#if defined(USE_SERIAL)
    Serial.print("SleepLoopTimerCount = ");
    Serial.println(SleepLoopTimerCount);
#endif

    printPins();
    Serial.println("Setup complete");
}

namespace {
    unsigned SleepTilNextInterrupt();

    const unsigned long FirstListenAfterTransmitMsec = 10000;// at system reset, listen Serial/RF for this long
    const unsigned long NormalListenAfterTransmit = 300;// after TX, go to RX for this long
    unsigned long ListenAfterTransmitMsec = FirstListenAfterTransmitMsec;
    unsigned int sampleCount;

    
    bool processCommand(const char *pCmd)
    {
         return false;
    }
}

void loop()
{
    static bool TransmittedSinceSleep = false;
    unsigned long now = millis();
	
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
                if (enableSerial)
                {
                    Serial.print(sendbuffer);
                    Serial.println(" command accepted for rain gauge");
                }
            }
            
            if (enableSerial)
                Serial.println("ready");
            sendlength = 0; // reset the packet
        }
    }
#endif

    if (now - TimeOfWakeup > ListenAfterTransmitMsec)
    {	// go to sleep. with R1/C1 as specified will be about 100 seconds
        TransmittedSinceSleep = false;
        pinMode(TIMER_RC_CHARGE_PIN, OUTPUT);
        digitalWrite(TIMER_RC_CHARGE_PIN, HIGH);
        printPins();
        digitalWrite(SENSOR_INTERRUPT_INVERT_PIN, digitalRead(ROCKER_INPUT_PIN)); 
        SleepTilNextInterrupt();
        printPins();
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
        if (enableSerial)
        {
            Serial.print("sleep for count=");
            Serial.println(SleepLoopTimerCount);
            Serial.flush();// wait for finish and turn off pins before sleep
            Serial.end();
        }
        pinMode(0, INPUT); // Arduino libraries have a symbolic definition for Serial pins?
#endif


#if defined(TELEMETER_BATTERY_V)
        analogReference(EXTERNAL); // This sequence drops idle current by 30uA
        analogRead(BATTERY_PIN); // doesn't shut down the band gap until we USE ADC
#endif


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
