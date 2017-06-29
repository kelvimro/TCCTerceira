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
const int PWMA = 5;                      // PWM motor pin
const int InA1 = 9;                     // INA1 motor pin - Para frente
const int InA2 = 10;                    // INA2 motor pin - Para trás

//Motor Direito = B
const int PWMB = 6;                       // PWM motor pin
const int InB1 = 11;                      // INB1 motor pin - Para frente
const int InB2 = 12;                      // INB2 motor pin - Para trás

const int motorSTBY = 13;  //STBY pin on TB6612FNG. Must be HIGH to enable motor

/*****************************************************************************************/

const int encodPinA1 = 2;                       // encoder A pin
const int encodPinB1 = 3;                       // encoder B pin
const int pulsos = 17;

/*****************************************************************************************/

#define Vpin A7                      // battery monitoring analog pin
const int Apin = 1;                       // motor current monitoring analog pin

/*****************************************************************************************/

// high current warning
const int CURRENT_LIMIT = 1000;
// low bat warning
const int LOW_BAT = 10000;
// PID loop time
const int LOOPTIME = 100;
// samples for Amp average
const int NUMREADINGS = 10;

/*****************************************************************************************/

int readings[NUMREADINGS];
// loop timing
unsigned long lastMilliPrint = 0;
// loop timing
unsigned long lastMilli = 0;
// speed timing
unsigned long oldtime = 0;
//PWM
double PWM_valA = 0;                               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
double PWM_valB = 0;                               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
//Bateria
int voltage = 0;                                // in mV
int current = 0;                                // in mA

double volatile countA = 0;                        // rev counter
double volatile countB = 0;                        // rev counter


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

//Varaveis myPID do tipo PID
//Specify the links and initial tuning parameters
//PID MPID(&input, &output, &setpoint, Kp, Ki, Kd, OPITON);
PID myPIDA(&speed_actA, &PWM_valA, &speed_actB, Kp, Ki, Kd, REVERSE);
PID myPIDB(&speed_actB, &PWM_valB, &speed_actA, Kp, Ki, Kd, REVERSE);

PID myPIDX(&speed_actA, &PWM_valA, &speed_reqA, Kp, Ki, Kd, DIRECT);

/*****************************************************************************************/
/*****************************************************************************************/
int treta = 0;
int treta2 = 0;

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

    Serial.begin(9600);
    // PID setado para automatico
    myPIDA.SetMode(AUTOMATIC);
    myPIDB.SetMode(AUTOMATIC);

    pinMode(InA1, OUTPUT);
    pinMode(InA2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(InB1, OUTPUT);
    pinMode(InB2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(encodPinA1, INPUT);
    pinMode(encodPinB1, INPUT);

    //turn on pullup resistor
    //digitalWrite(encodPinA1, HIGH);
    //digitalWrite(encodPinB1, HIGH);

    //Interrupt para contagem dos encoders - interrupt evita perda de contagens por outros processos
    attachInterrupt(digitalPinToInterrupt(encodPinA1), rencoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encodPinB1), rencoderB, CHANGE);

    // initialize readings to 0
    for (int i = 0; i < NUMREADINGS; i++) readings[i] = 0;

    //Start motor com configurações inicais "á frente"
    analogWrite(PWMA, PWM_valA);
    digitalWrite(InA1, HIGH);
    digitalWrite(InA2, LOW);
    analogWrite(PWMB, PWM_valB);
    digitalWrite(InB1, HIGH);
    digitalWrite(InB2, LOW);
}
/*****************************************************************************************/

// display data
void printMotorInfo() {
    //Controle de tempo (evitar excesso de prints)
    if ((millis() - lastMilliPrint) >= 500) {
        lastMilliPrint = millis();
        //Motor A informações
        Serial.print("A! SP:");
        Serial.print(speed_reqA);
        Serial.print("  RPM:");
        Serial.print(speed_actA);
        Serial.print("  PWM:");
        Serial.print(PWM_valA);
        Serial.print("  V:");
        Serial.print(float(voltage) / 1000, 1);
        Serial.print("  mA:");
        Serial.println(current);
        //Motor B informações
        Serial.print("B! SP:");
        Serial.print(speed_reqB);
        Serial.print("  RPM:");
        Serial.print(speed_actB);
        Serial.print("  PWM:");
        Serial.print(PWM_valB);
        Serial.print("  V:");
        Serial.print(float(voltage) / 1000, 1);
        Serial.print("  mA:");
        Serial.println(current);

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
// remove signal noise
int digital_smooth(int value, int *data_array) {
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

    //speed_actA = (60 * 1000 / pulsos) / (millis() - oldtime) * countA;
    //speed_actB = (60 * 1000 / pulsos) / (millis() - oldtime) * countB;

    speed_actA = countA;
    speed_actB = countB;
    oldtime = millis();
    countA = 0;
    countB = 0;

    //Habilita interrupcao
    attachInterrupt(digitalPinToInterrupt(encodPinA1), rencoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encodPinB1), rencoderB, CHANGE);
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
    if (Serial.available()) {
        delay(10);
        int mimimi;
        int teste;
        mimimi = Serial.parseInt();
        Serial.print("Res: ");
        Serial.println(mimimi);
        teste = map(mimimi, 0, 9, 0, 255);
        Serial.print("Map: ");
        Serial.println(teste);
        PWM_valA = teste;
        PWM_valB = teste;
        analogWrite(PWMA, PWM_valA);
        analogWrite(PWMB, PWM_valB);
        treta2 = 0;
        countA = 0;
        countB = 0;
        delay(100);
        getMotorData();
        lastMilli = millis();
    }

    while (treta < 3) {
        PWM_valA = 35; PWM_valB = 35;
        analogWrite(PWMA, PWM_valA);
        digitalWrite(InA1, HIGH);
        digitalWrite(InA2, LOW);
        analogWrite(PWMB, PWM_valB);
        digitalWrite(InB1, LOW);
        digitalWrite(InB2, HIGH);
        digitalWrite(motorSTBY, HIGH);
        Serial.println("Setup OK");
        delay(1500);
        treta2 = 1;
        countA = 0;
        countB = 0;
        lastMilli = millis();
        treta++;
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
        if (treta2 == 0) {
            delay(100);
            getMotorData();
            if(countA > countB) treta2 = 1;
            if(countB > countA) treta2 = 2;
            while (speed_actA >= (speed_actB * 1.05) && treta2 == 1) {
                myPIDA.Compute();
                PWM_valA = constrain(PWM_valA, 0, 255);
                Serial.print("PID A ");
                Serial.println(PWM_valA);
                Serial.print("CNT A ");
                Serial.println(countA);
                Serial.print("CNT B ");
                Serial.println(countB);
                analogWrite(PWMA, PWM_valA);
                digitalWrite(InA1, HIGH);
                digitalWrite(InA2, LOW);
                digitalWrite(motorSTBY, HIGH);
                getMotorData();
                printMotorInfo();
                delay(100);
            }
            while (speed_actB >= (speed_actA * 1.05) && treta2 == 2) {
                myPIDB.Compute();
                PWM_valB = constrain(PWM_valB, 0, 255);
                Serial.print("PID B ");
                Serial.println(PWM_valB);
                Serial.print("CNT A ");
                Serial.println(countA);
                Serial.print("CNT B ");
                Serial.println(countB);
                analogWrite(PWMB, PWM_valB);
                digitalWrite(InB1, LOW);
                digitalWrite(InB2, HIGH);
                digitalWrite(motorSTBY, HIGH);
                getMotorData();
                printMotorInfo();
                delay(100);
            }

        }
    }
}
/*****************************************************************************************/

