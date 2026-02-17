#include <Wire.h>
#include <RG-Tmp175.h>

#define PCB_REV_NUMBER 3 // Sketch supports versions 2 or 3 only

namespace {

    const int RC_RLY_INTERRUPT_PIN = 3;
#if PCB_REV_NUMBER >= 3
    const int RC_STATUS_PIN = 17;   // not used, but on REV02 of the PCB
    const int ROCKER_INPUT_PIN = 16;
#elif PCB_REV_NUMBER <= 2   // these pin assignments are reversed on REV02 vs REV03
    const int RC_STATUS_PIN = 16;
    const int ROCKER_INPUT_PIN = 17;
#endif
    const int SENSOR_INTERRUPT_INVERT_PIN = 7;
    const int TIMER_RC_CHARGE_PIN = 8; // sleep uProc using RC circuit on this pin
    const long SERIAL_PORT_BAUDS = 38400;

    void printPins()
    {
        Serial.println();
        Serial.print("Interrupt 3 is ");
        Serial.println(digitalRead(RC_RLY_INTERRUPT_PIN) == LOW ? "active" : "inactive");
        Serial.print("Hall effect interrupt) is ");
        Serial.println(digitalRead(ROCKER_INPUT_PIN) == LOW ? "active" : "inactive");
        Serial.print("RC_STATUS_PIN is ");
        Serial.println(digitalRead(RC_STATUS_PIN) == HIGH ? "active" : "inactive");
    }    
 
    TMP175 tmp175(0b1001000); //0b1001000 per TMP175 docs, is I2C address with all addressing pins low.

    bool processCommand(const char* pCmd)
    {
        static const char DUMP[] = "D";
        static const char CHARGE[] = "R";
        static const char WATCH[] = "C";
        static const char TEMP[] = "T";
        static const char INPUTS[] = "P";
        static const char START[] = "S";
        if (strncmp(pCmd, DUMP, sizeof(DUMP) - 1) == 0)
        {
            tmp175.dump();
            return true;
        }
        else if (strncmp(pCmd, CHARGE, sizeof(CHARGE) - 1) == 0)
        {
            pinMode(TIMER_RC_CHARGE_PIN, OUTPUT);
            digitalWrite(TIMER_RC_CHARGE_PIN, HIGH);
            Serial.println("Charge");
            return true;
        }
        else if (strncmp(pCmd, WATCH, sizeof(WATCH) - 1) == 0)
        {
            pinMode(TIMER_RC_CHARGE_PIN, INPUT);
            Serial.println("RC discharge");
            return true;
        }
        else if (strncmp(pCmd, TEMP, sizeof(TEMP) - 1) == 0)
        {
            auto mag = tmp175.readTempCx16();
            Serial.print("Raw = ");
            Serial.println(mag);
            bool isNeg = false;
            if (mag < 0)
            {
                isNeg = true;
                mag = -mag;
            }
            auto whole = mag >> 4;
            auto frac = mag & 0xF;
            frac *= 100;
            frac >>= 4;
            Serial.print("T=");
            if (isNeg)
                Serial.print('-');
            else
                Serial.print(' ');
            Serial.print(whole);
            Serial.print('.');
            auto tenths = frac % 10;
            auto hundredths = frac / 10;
            Serial.print((char)('0' + tenths));
            Serial.println((char)('0' + hundredths));
            return true;
        }
        else if (strncmp(pCmd, INPUTS, sizeof(INPUTS)-1) == 0)
        {
            printPins();
            return true;
        }
        else if (strncmp(pCmd, START, sizeof(START)-1) == 0)
        {
            tmp175.startReadTemperature();
            return true;
        }
        return false;
    }
}

void setup()
{
    pinMode(ROCKER_INPUT_PIN, INPUT_PULLUP);
    pinMode(RC_RLY_INTERRUPT_PIN, INPUT);
    pinMode(TIMER_RC_CHARGE_PIN, OUTPUT);
    digitalWrite(TIMER_RC_CHARGE_PIN, HIGH);
    pinMode(SENSOR_INTERRUPT_INVERT_PIN, OUTPUT);
    digitalWrite(SENSOR_INTERRUPT_INVERT_PIN, digitalRead(ROCKER_INPUT_PIN)); 

    Serial.begin(SERIAL_PORT_BAUDS);
    Serial.println("TMP175 test");
    printPins();
    Wire.begin();
    tmp175.setup();
    Serial.println("Setup complete");
    pinMode(TIMER_RC_CHARGE_PIN, INPUT);
}

void loop()
{
    while (Serial.available() > 0)
    {
        static char sendbuffer[62];
        static int sendlength = 0;
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
                Serial.println(" command accepted for test");
            }
            Serial.println("ready");
            sendlength = 0; // reset the packet
        }
    }

    {
        static int prevsi7210 = -1;
        static int previntIn = -1;
        auto si7210Input = digitalRead(ROCKER_INPUT_PIN);
        auto intIn = digitalRead(RC_RLY_INTERRUPT_PIN);
        if ((intIn != previntIn) || (prevsi7210 != si7210Input))
        {
            printPins();
        }
        prevsi7210 = si7210Input;
        previntIn = intIn;

    }
}