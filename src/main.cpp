#include <Arduino.h>
// Modu Cinza GY 4 pinos
#include <SoftwareSerial.h>
/*
Spiderbot bluetooth remote control using
Arduino Bluetooth Controller App

It has two modes:

  RC mode (default when turned on)
  line-following - manual control is disabled except for the select (toggle) button

The"select" button toggles these two modes
*/


//Pins for HC-06 (FC114 model) RX/TX
//  Arduino D12(TX) to BlueTooth RX
//  Arduino D2(RX) to BlueTooth TX
SoftwareSerial BTSerial(0, 1); // RX | TX

//For NANO
//PWM: 3, 5, 6, 9, 10, and 11. Provide 8-bit PWM output with the analogWrite() function.

const int lSensor = A1; //left IR line tracking sensor input
const int rSensor = A2; //right IR line tracking sensor input

//Encoders parameters
const int Lencoder1 = 2;
const int Rencoder1 = 3;
const int Lencoder2 = 7;
const int Rencoder2 = 8;
//Altere o numero abaixo de acordo com o seu disco encoder
//34 Total 12 brancos 12 pretos --- Preto= '1' Branco = '0'
unsigned int pulsos_por_volta = 12;
int rpm;
volatile byte pulsos;
unsigned long oldtime;

//drive motor/shield parameters
const int LmotorPWM  = 5;   //speed control using analogWrite() function. Value between 0 - 255
const int LmotorPOS  = 9;   //HIGH should go forward
const int LmotorNEG  = 10;  //LOW should go forward

const int RmotorPWM  = 6;   //speed control using analogWrite() function. Value between 0 - 255
const int RmotorPOS  = 11;  //HIGH should go forward
const int RmotorNEG  = 12;  //LOW should go forward
const int motorSTBY  = 13;  //STBY pin on TB6612FNG. Must be HIGH to enable motor

//calibrate the following values to make it go straight
int LmotorSpeed = 250; //slow down a bit. left more appears to be more powerful than rightt
int RmotorSpeed = 100; //max speed

//for Serial.read
char command = ' ';

//op mode: could also use boolean in this case
int opMode = 0; // operation mode: 0 = RC (default), 1 = line following

//set this to true when debugging and watch the serial monitor for status msg
boolean DEBUG = false;


// Bateria config
int bateria = A7;
const int ledPin = 4;
int outbat = 0;
int bat = 0;
int estadoLed = HIGH; // estadoLed

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0; // will store last time LED was updated

// constants won't change :
const long intervalo = 1000; // interval at which to blink (milliseconds)
// Bateria fim

/****************************************************************************/
void contador(){
    pulsos++;
}
/****************************************************************************/
void testeencoder(){
    Serial.println(digitalRead(Lencoder1));
}
/****************************************************************************/
void setup() {

    //Set bateria
    Serial.begin(9600);
    pinMode(bateria, INPUT);

    //set the pinmode of encoder
    pinMode (lSensor,INPUT);
    pinMode (rSensor,INPUT);

    //Setup de RPM
    //Interrupcao "0" - pino digital 2 (Lencoder1)
    //Aciona o contador a cada pulso
    //attachInterrupt(0, contador, FALLING); //Muda quando 1 vira 0
    attachInterrupt(digitalPinToInterrupt(Lencoder1), contador, RISING);
    //Contador de pulsos
    pulsos = 0;
    //ultima RPM calculada
    rpm = 0;
    // Não sei ainda
    oldtime = 0;

    //motor output
    pinMode(LmotorPWM, OUTPUT);
    pinMode(LmotorPOS, OUTPUT);
    pinMode(LmotorNEG, OUTPUT);
    pinMode(RmotorPWM, OUTPUT);
    pinMode(RmotorPOS, OUTPUT);
    pinMode(RmotorNEG, OUTPUT);
    pinMode(motorSTBY, OUTPUT);

    if (DEBUG) {
        Serial.begin(9600);
        Serial.println("Remote Control Spider Bot using Arduino Bluetooth Contoller App");
        Serial.println("Arduino with HC-0x FC-114 is ready");
    }

    // FC-114 default baud rate is 9600
    BTSerial.begin(9600);


    if (DEBUG) {
        Serial.println("BTserial started at 9600");
    }

}
/****************************************************************************/

void goStraight() {
    analogWrite (LmotorPWM, LmotorSpeed);
    digitalWrite (LmotorPOS, HIGH);
    digitalWrite (LmotorNEG, LOW);

    analogWrite (RmotorPWM, RmotorSpeed);
    digitalWrite (RmotorPOS, HIGH);
    digitalWrite (RmotorNEG, LOW);

    digitalWrite (motorSTBY, HIGH);
}
/****************************************************************************/

void goReverse() {
    analogWrite (LmotorPWM, LmotorSpeed);
    digitalWrite (LmotorPOS, LOW);
    digitalWrite (LmotorNEG, HIGH);

    analogWrite (RmotorPWM, RmotorSpeed);
    digitalWrite (RmotorPOS, LOW);
    digitalWrite (RmotorNEG, HIGH);

    digitalWrite (motorSTBY, HIGH);
}
/****************************************************************************/

void goLeft() {
    analogWrite (LmotorPWM, LmotorSpeed);
    digitalWrite (LmotorPOS, LOW);
    digitalWrite (LmotorNEG, HIGH);

    analogWrite (RmotorPWM, RmotorSpeed);
    digitalWrite (RmotorPOS, HIGH);
    digitalWrite (RmotorNEG, LOW);

    digitalWrite (motorSTBY, HIGH);
}
/****************************************************************************/

void goRight() {
    analogWrite (LmotorPWM, LmotorSpeed);
    digitalWrite (LmotorPOS, HIGH);
    digitalWrite (LmotorNEG, LOW);

    analogWrite (RmotorPWM, RmotorSpeed);
    digitalWrite (RmotorPOS, LOW);
    digitalWrite (RmotorNEG, HIGH);

    digitalWrite (motorSTBY, HIGH);
}
/****************************************************************************/

void stop() {
    analogWrite (LmotorPWM, 0);
    digitalWrite (LmotorPOS, LOW);
    digitalWrite (LmotorNEG, LOW);

    analogWrite (RmotorPWM, 0);
    digitalWrite (RmotorPOS, LOW);
    digitalWrite (RmotorNEG, LOW);

    digitalWrite (motorSTBY, LOW);
}
/****************************************************************************/

//if RC mode, run this routine
void RC_mode() {

    if (BTSerial.available()) {

        command = BTSerial.read();
        //Serial.print(command);

        //program the following buttons into Arduino Bluetooth Controller App
        switch(command) {
            case 'u':
                goStraight();
                if (DEBUG) Serial.println("go straight");
                digitalWrite(ledPin, HIGH);
                break;
            case 'd':
                goReverse();
                if (DEBUG) Serial.println("go reverse");
                break;
            case 'l':
                goLeft();
                if (DEBUG) Serial.println("go left");
                break;
            case 'r':
                goRight();
                if (DEBUG) Serial.println("go right");
                break;
            case 's':
                stop();
                if (DEBUG) Serial.println("stop");
                digitalWrite(ledPin, LOW);
                break;
            case 't':
                stop();
                if (DEBUG) Serial.println("Toggle to Line Following Mode");
                opMode = 1;   //toggle to line following mode
                break;
        }
    }

}
/****************************************************************************/

//if line following mode, run this routine
void LineFollowing_mode() {

    int lSensStat = analogRead (lSensor);  //0 = signal detected (white surface): 1 = black line
    int rSensStat = analogRead (rSensor);  //0 = signal detected (white surface): 1 = black line

    if (DEBUG) if (!lSensStat) { Serial.print ("Left on line   ");} else { Serial.print ("Left no line   ");};
    if (DEBUG) if (!rSensStat) { Serial.println ("Right on line");} else { Serial.println ("Right no line");};

    if (lSensStat && rSensStat) { goStraight();};  //both sensors detected white floor. go straight
    if (lSensStat && !rSensStat) { goRight();};    //detected black surface on right (off), so turn right
    if (!lSensStat && rSensStat) { goLeft();};     //detected black surface on left (off), so turn left
    if (!lSensStat && !rSensStat) { stop();};      //detected black surface on both sensor, so stop!

    //delay(20);


    //check for toggle button
    if (BTSerial.available()) {

        command = BTSerial.read();
        //Serial.print(command);

        if (command == 't') {
            stop();
            opMode = 0;   //toggle to RC mode
        }
    }
}
/****************************************************************************/

//main program
void loop() {
    //Inicio Encoder


/****************************************************************************/
    // Bateria inicio
    unsigned long currentMillis = millis();
    bat = analogRead(bateria);
    outbat = map(bat, 0, 1023, 0, 255);
    if(DEBUG) {
        Serial.print("sensor = ");
        Serial.print(bat);
        Serial.print("\t output = ");
        Serial.println(outbat);
        delay(10);
    }
    analogWrite (ledPin, outbat);
    if (outbat < 140){
        if (currentMillis - previousMillis >= intervalo) {
            // save the last time you blinked the LED
            previousMillis = currentMillis;

            // if the LED is off turn it on and vice-versa:
            if (estadoLed == HIGH) {
                estadoLed = LOW;
            } else {
                estadoLed = HIGH;
            }
        }
        // set the LED with the ledState of the variable:
        digitalWrite(ledPin, estadoLed);
    }
    else
        digitalWrite(ledPin, LOW);
    //Bateria fim

/****************************************************************************/
    //Atualiza contador a cada segundo
    if (millis() - oldtime >= 1000)
    {
        //Desabilita interrupcao durante o calculo
        detachInterrupt(digitalPinToInterrupt(Lencoder1));
        rpm = (60 * 1000 / pulsos_por_volta ) / (millis() - oldtime) * pulsos;
        oldtime = millis();
        pulsos = 0;

        //Mostra o valor de RPM no serial monitor
        Serial.print("RPM = ");
        Serial.println(rpm, DEC);
        //Habilita interrupcao
        attachInterrupt(digitalPinToInterrupt(Lencoder1), contador, RISING);
    }
/****************************************************************************/
    //default mode is RC mode
    if (opMode == 0) {
        RC_mode();
        if (DEBUG) Serial.println("RC mode");
    } else if (opMode == 1) {
        LineFollowing_mode();
        if (DEBUG) Serial.println("Line Following mode");
    }
}
/****************************************************************************/