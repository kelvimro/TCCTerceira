#include <Arduino.h>
#include <SoftwareSerial.h>
#include "../lib/Arduino-PID-Library-master/PID_v1.h"


// Test MD03a / Pololu motor with encoder
// speed control (PI), V & I display
// Credits:
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923



/*****************************************************************************************/
//Motor Esquerdo = A
const int PWMA =5;                      // PWM motor pin
const int InA1 = 9;                      // INA motor pin
const int InA2 = 10;                     // INB motor pin

//Motor Direito = B
const int PWMB = 6;                       // PWM motor pin
const int InB1 = 11;                     // INA motor pin
const int InB2 = 12;                      // INB motor pin

const int motorSTBY = 13;  //STBY pin on TB6612FNG. Must be HIGH to enable motor

/*****************************************************************************************/

const int encodPinA1 = 2;                       // encoder A pin
const int encodPinB1 = 3;                       // encoder B pin
const int pulsos = 17;

/*****************************************************************************************/

#define Vpin            A7                      // battery monitoring analog pin
#define Apin            1                       // motor current monitoring analog pin

/*****************************************************************************************/

#define CURRENT_LIMIT   1000                    // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        100                     // PID loop time
#define NUMREADINGS     10                      // samples for Amp average

/*****************************************************************************************/

int readings[NUMREADINGS];
unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing
unsigned long oldtime = 0;
//PWM
double PWM_valA = 0;                               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
double PWM_valB = 0;                               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
//Bateria
int voltage = 0;                                // in mV
int current = 0;                                // in mA

volatile byte countA = 0;                        // rev counter
volatile double countB = 0;                        // rev counter


/*****************************************************************************************/

//Velocidade solicitada
double speed_reqA = 300;                           // speed (Set Point)
double speed_reqB = 300;                           // speed (Set Point)

//Velocidade atual
double speed_actA = 0;                             // speed (actual value)
double speed_actB = 0;                             // speed (actual value)

/*****************************************************************************************/

//PID config
float Kp = .4;                                // PID proportional control Gain
float Ki = 0.02;
float Kd = 0.5;                                // PID Derivitave control gain

//Specify the links and initial tuning parameters
//PID MPID(&input, &output, &setpoint, Kp, Ki, Kd, OPITON);
PID myPIDA(&speed_actA, &PWM_valA, &speed_actB, Kp, Ki, Kd, DIRECT);
PID myPIDB(&speed_actB, &PWM_valB, &speed_reqA, Kp, Ki, Kd, DIRECT);
PID myPIDX(&speed_actA, &PWM_valA, &speed_reqA, Kp, Ki, Kd, DIRECT);

/*****************************************************************************************/
/*****************************************************************************************/
int treta = 0;
/*****************************************************************************************/

void rencoderA() {                               // pulse and direction, direct port reading to save cycles
    countA++;    // if(digitalRead(encodPinB1)==HIGH)   count ++;
}
/*****************************************************************************************/
void rencoderB() {
    countB++;
}
/*****************************************************************************************/

void setup() {
    // Current external ref is 3.3V
    analogReference(EXTERNAL);

    Serial.begin(115200);

    myPIDA.SetMode(AUTOMATIC);
    myPIDB.SetMode(AUTOMATIC);

    pinMode(InA1, OUTPUT);
    pinMode(InA2, OUTPUT);
    pinMode(InB1, OUTPUT);
    pinMode(InB2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(encodPinA1, INPUT);
    pinMode(encodPinB1, INPUT);
    //digitalWrite(encodPinA1, HIGH);                      // turn on pullup resistor
    //digitalWrite(encodPinB1, HIGH);

    //Interrupt para contagem dos encoders
    attachInterrupt(digitalPinToInterrupt(encodPinA1), rencoderA, FALLING);
    attachInterrupt(digitalPinToInterrupt(encodPinB1), rencoderB, FALLING);

    for (int i = 0; i < NUMREADINGS; i++) readings[i] = 0;  // initialize readings to 0

    digitalWrite(PWMA, PWM_valA);
    digitalWrite(InA1, HIGH);
    digitalWrite(InA2, LOW);
    digitalWrite(PWMB, PWM_valB);
    digitalWrite(InB1, HIGH);
    digitalWrite(InB2, LOW);
}
/*****************************************************************************************/

void printMotorInfo() {                                                      // display data
    if ((millis() - lastMilliPrint) >= 500) {
        lastMilliPrint = millis();
        //Motor A informações
        Serial.print("A! SP:");        Serial.print(speed_reqA);
        Serial.print("  RPM:");        Serial.print(speed_actA);
        Serial.print("  PWM:");        Serial.print(PWM_valA);
        Serial.print("  V:");          Serial.print(float(voltage) / 1000, 1);
        Serial.print("  mA:");         Serial.println(current);
        //Motor B informações
        Serial.print("B! SP:");        Serial.print(speed_reqB);
        Serial.print("  RPM:");        Serial.print(speed_actB);
        Serial.print("  PWM:");        Serial.print(PWM_valB);
        Serial.print("  V:");          Serial.print(float(voltage) / 1000, 1);
        Serial.print("  mA:");         Serial.println(current);

        if (current > CURRENT_LIMIT) Serial.println("*** CURRENT_LIMIT ***");
        if (voltage > 1000 && voltage < LOW_BAT) Serial.println("*** LOW_BAT ***");
    }
}
/*****************************************************************************************/

int getParam() {
    char param, cmd;
    if (!Serial.available()) return 0;
    delay(10);
    param = Serial.read();                              // get parameter byte
    if (!Serial.available()) return 0;
    cmd = Serial.read();                                // get command byte
    Serial.flush();
    switch (param) {
        case 'v':                                         // adjust speed
            if (cmd == '+') {
                speed_reqA += 20;
                if (speed_reqA > 400) speed_reqA = 400;
            }
            if (cmd == '-') {
                speed_reqA -= 20;
                if (speed_reqA < 0) speed_reqA = 0;
            }
            break;
        case 's':                                        // adjust direction
            if (cmd == '+') {
                digitalWrite(InA1, LOW);
                digitalWrite(InB1, HIGH);
            }
            if (cmd == '-') {
                digitalWrite(InA1, HIGH);
                digitalWrite(InB1, LOW);
            }
            break;
        case 'o':                                        // user should type "oo"
            digitalWrite(InA1, LOW);
            digitalWrite(InB1, LOW);
            speed_reqA = 0;
            break;
        default:
            Serial.println("???");
    }
}
/*****************************************************************************************/

int digital_smooth(int value, int *data_array) {    // remove signal noise
    static int ndx = 0;
    static int count = 0;
    static int total = 0;
    total -= data_array[ndx];
    data_array[ndx] = value;
    total += data_array[ndx];
    ndx = (ndx + 1) % NUMREADINGS;
    if (count < NUMREADINGS) count++;
    return total / count;
}
/*****************************************************************************************/

void getMotorData() {                                      // calculate speed, volts and Amps
   // static long countAntA = 0;                                      // last count
   // static long countAntB = 0;                                      // last count

/*****************************************/
    //Desabilita interrupcao durante o calculo
    detachInterrupt(digitalPinToInterrupt(encodPinA1));
    detachInterrupt(digitalPinToInterrupt(encodPinB1));

    //Desabilita interrupcao durante o calculo
    speed_actA = ((60 * 1000 / pulsos) / (millis() - oldtime) * countA)/10000;
    speed_actB = ((60 * 1000 / pulsos) / (millis() - oldtime) * countB)/10000;

    oldtime = millis();
    countA = 0;
    countB = 0;

    //Habilita interrupcao
    attachInterrupt(digitalPinToInterrupt(encodPinA1), rencoderA, FALLING);
    attachInterrupt(digitalPinToInterrupt(encodPinB1), rencoderB, FALLING);
/*****************************************/
    // Atualiza velocidade atual
    //speed_actA = ((countA - countAntA) * (60 * (1000 / LOOPTIME)))/(pulsos);
    //speed_actB = ((countB - countAntB) * (60 * (1000 / LOOPTIME)))/(pulsos);
    //countAntA = countA;
    //countAntB = countB;

    // battery voltage: mV=ADC*3300/1024, voltage divider 10K+2K
    voltage = int(analogRead(Vpin) * 3.22 * 12.2 / 2.2);
    // motor current - output: 130mV per Amp
    current = int(analogRead(Apin) * 3.22 * .77 * (1000.0 / 132.0));
    // remove signal noise
    current = digital_smooth(current, readings);
}
/*****************************************************************************************/

int updatePid(int command, int targetValue, int currentValue) {  // compute PWM value
    float pidTerm = 0;                                             // PID correction
    int error = 0;
    static int last_error = 0;

    error = abs(targetValue) - abs(currentValue);

    pidTerm = (Kp * error) + (Kd * (error - last_error));

    last_error = error;

    return constrain(command + int(pidTerm), 0, 255);
}
/*****************************************************************************************/

void loop() {
    delay(10);

    //getParam(); // check keyboard
    /*if (Serial.available()){
        int mimimi;
        int teste;
        mimimi = Serial.read();
        Serial.flush();
        teste = map(mimimi, 0, 9, 0, 255);
        PWM_valA = teste;
        PWM_valB = teste;
        digitalWrite(PWMA, PWM_valA);
        digitalWrite(PWMB, PWM_valB);
        lastMilli = millis();
    }*/

    while (treta < 3){
        delay(1500);
        digitalWrite(PWMA, 30);
        digitalWrite(InA1, HIGH);
        digitalWrite(InA2, LOW);

        digitalWrite(PWMB, 30);
        digitalWrite(InB1, HIGH);
        digitalWrite(InB2, LOW);

        digitalWrite(motorSTBY, HIGH);

        Serial.println("Setup OK");
        treta++;
        countA = 0;
        countB = 0;
        lastMilli = millis();
    }

    if ((millis() - lastMilli) >= LOOPTIME) {                       // enter tmed loop
        lastMilli = millis();

        // display data
        printMotorInfo();
        // calculate speed, volts and Amps
        getMotorData();

//        PWM_valA = updatePid(PWM_valA, speed_reqA, speed_actA);     // compute PWM value
//        analogWrite(PWMA, PWM_valA);                                // send PWM to motor
//
//        PWM_valB = updatePid(PWM_valB, speed_reqB, speed_actB);      // compute PWM value
//        analogWrite(PWMB, PWM_valB);                                 // send PWM to motor

        //myPID.SetTunings(Kp,Ki,Kd);
        if(speed_actA > speed_actB) {
            myPIDA.Compute();
            PWM_valA = constrain(PWM_valA, 0, 255);
            analogWrite(PWMA, PWM_valA);
            digitalWrite(InA1, HIGH);
            digitalWrite(InA2, LOW);
            digitalWrite(motorSTBY, HIGH);
        } else if (speed_actB > speed_actA) {
            myPIDB.Compute();
            PWM_valB = constrain(PWM_valB, 0, 255);
            analogWrite(PWMB, PWM_valB);
            digitalWrite(InB1, HIGH);
            digitalWrite(InB2, LOW);
            digitalWrite(motorSTBY, HIGH);
        }
    }
}
/*****************************************************************************************/

