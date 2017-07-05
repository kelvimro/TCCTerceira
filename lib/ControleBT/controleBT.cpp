#include <Arduino.h>
const byte PIN_ANALOG_X = 0;
const byte PIN_ANALOG_Y = 1;
double cmdX;
double cmdY;
double sendA;
double sendB;
int looptime = 200;
int oldtime = 0;


void setup() {
    Serial.begin(9600);
    oldtime = millis();
}


void loop() {

    if ((millis() - oldtime) >= looptime) {

        oldtime = millis();

        cmdY = analogRead(PIN_ANALOG_Y);
        cmdX = analogRead(PIN_ANALOG_X);

        if (cmdX >= 0 && cmdX <= 501) cmdX = map(cmdX, 0, 501, -100, 0);
        else cmdX = map(cmdX, 502, 1023, 0, 100);

        if (cmdY >= 0 && cmdY <= 509) cmdY = map(cmdY, 0, 509, -100, 0);
        else cmdY = map(cmdY, 510, 1023, 0, 100);

        sendA = sendB = cmdY;

        if (cmdX < -1) {
            sendA -= (cmdY * (cmdX / -100));
        } else if (cmdX > 1) {
            sendB -= (cmdY * (cmdX / 100));
        }

        int a, b;
        a = sendA;
        b = sendB;

        Serial.print(" x: ");
        Serial.print(a);
        Serial.print(" y: ");
        Serial.println(b);
    }
}



