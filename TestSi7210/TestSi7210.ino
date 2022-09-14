#include <Wire.h>
#include <RG-Si7210.h>

namespace {
    const int ROCKER_INPUT_PIN = 17;
    const int INTERRUPT_INPUT_PIN = 3;
    const int RC_INOUT_PIN = 8;

    Si7210 si7210(0x33);

    bool processCommand(const char* pCmd)
    {
        static const char TOGGLE[] = "T";
        static const char DUMP[] = "D";
        static const char ONE[] = "1";
        static const char CHARGE[] = "R";
        static const char WATCH[] = "C";
        static const char WAKE[] = "W";
        static const char SLEEP[] = "S";
        static const char MAG[] = "M";
        static const char OTP[] = "O";
        static const char INPUTS[] = "P";
        if (strncmp(pCmd, TOGGLE, sizeof(TOGGLE) - 1) == 0)
        {
            auto ti = si7210.toggleOutputSense();
            Serial.print("toggleOutputSense returned: ");
            Serial.println(ti, HEX);
            return true;
        }
        else if (strncmp(pCmd, DUMP, sizeof(DUMP) - 1) == 0)
        {
            si7210.dump();
            return true;
        }
        else if (strncmp(pCmd, ONE, sizeof(ONE) - 1) == 0)
        {
            si7210.one();
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
        else if (strncmp(pCmd, SLEEP, sizeof(SLEEP) - 1) == 0)
        {
            auto v = si7210.sleep();
            Serial.print("Sleep reg=0x");
            Serial.println((int)v, HEX);
            return true;
        }
        else if (strncmp(pCmd, OTP, sizeof(OTP) - 1) == 0)
        {
            si7210.resetToOTP();
            return true;
        }
        else if (strncmp(pCmd, WAKE, sizeof(WAKE) - 1) == 0)
        {
            si7210.wakeup();
            return true;
        }
        else if (strncmp(pCmd, MAG, sizeof(MAG) - 1) == 0)
        {
            auto mag = si7210.readMagField();
            Serial.print("SI7210 results: field=");
            Serial.print(mag);
            Serial.print(" interrupt=");
            Serial.println(digitalRead(ROCKER_INPUT_PIN));
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
    auto swop = si7210.setup();
    Serial.print("Si7210 test. swop=");
    Serial.println(swop);
}

void loop()
{
    static const int POLL_INTERVAL_MSEC = 300;
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

    auto now = millis();
    static unsigned long LastPolled;
    if (now - LastPolled > POLL_INTERVAL_MSEC)
    {
        LastPolled = now;
        if (!si7210.isSleeping())
            si7210.one();
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

        if (si7210.isSleeping() && si7210Input == LOW)
        {
            si7210.wakeup();
            si7210.one();
            processCommand("M");
            si7210.toggleOutputSense();
            si7210.sleep();
        }
    }
}