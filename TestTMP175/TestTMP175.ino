#include <Wire.h>
#include <RG-Tmp175.h>

namespace {
    const int ROCKER_INPUT_PIN = 17;
    const int INTERRUPT_INPUT_PIN = 3;
    const int RC_INOUT_PIN = 8;

    TMP175 tmp175(0b1001000); //0b1001000 per TMP175 docs, is I2C address with all addressing pins low.

    bool processCommand(const char* pCmd)
    {
        static const char DUMP[] = "D";
        static const char CHARGE[] = "R";
        static const char WATCH[] = "C";
        static const char TEMP[] = "T";
        static const char INPUTS[] = "P";
        if (strncmp(pCmd, DUMP, sizeof(DUMP) - 1) == 0)
        {
            tmp175.dump();
            return true;
        }
        else if (strncmp(pCmd, CHARGE, sizeof(CHARGE) - 1) == 0)
        {
            pinMode(RC_INOUT_PIN, OUTPUT);
            digitalWrite(RC_INOUT_PIN, HIGH);
            Serial.println("Charge");
            return true;
        }
        else if (strncmp(pCmd, WATCH, sizeof(WATCH) - 1) == 0)
        {
            pinMode(RC_INOUT_PIN, INPUT);
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
            Serial.print("D3=");
            Serial.print(digitalRead(3));
            Serial.print(" D17=");
            Serial.println(digitalRead(17));
        }
        return false;
    }
}

void setup()
{
    pinMode(ROCKER_INPUT_PIN, INPUT);
    pinMode(INTERRUPT_INPUT_PIN, INPUT);
    Serial.begin(9600);
    Serial.println("TMP175 test");
    tmp175.setup();
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
        auto intIn = digitalRead(INTERRUPT_INPUT_PIN);
        if ((intIn != previntIn) || (prevsi7210 != si7210Input))
        {
            Serial.print("D3="); 
            Serial.print((int)intIn);
            Serial.print(" D17=");
            Serial.println((int)si7210Input);
        }
        prevsi7210 = si7210Input;
        previntIn = intIn;

    }
}