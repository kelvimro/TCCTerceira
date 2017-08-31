#include <Arduino.h>
#include <SoftwareSerial.h>
#include "../lib/Arduino-PID-Library-master/PID_v1.h"
#include "../lib/PID_AutoTune_v0/PID_AutoTune_v0.h"
#include "../lib/QueueArray/QueueArray.h"


// Test MD03a / Pololu motor with encoder
// speed control (PI), V & I display
// Credits:
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923



/****************************************************************************************************/
/****************************************************************************************************/

//Motor Esquerdo = A
const int PWMA = 5;                      // PWM motor pin
const int InA1 = 9;                     // INA1 motor pin - Para frente
const int InA2 = 10;                    // INA2 motor pin - Para trás

//Motor Direito = B
const int PWMB = 6;                       // PWM motor pin
const int InB1 = 11;                      // INB1 motor pin - Para frente
const int InB2 = 12;                      // INB2 motor pin - Para trás

const int motorSTBY = 13;  //STBY pin on TB6612FNG. Must be HIGH to enable motor

/****************************************************************************************************/
/****************************************************************************************************/


const int encodPinA1 = 2;                       // encoder A pin
const int encodPinB1 = 3;                       // encoder B pin
const int pulsos = 17;

/****************************************************************************************************/
/****************************************************************************************************/


#define Vpin A7                      // battery monitoring analog pin
const int Apin = 1;                       // motor current monitoring analog pin

/****************************************************************************************************/
/****************************************************************************************************/

// Lenta
const int LOW_SPEED = 25;
// Calibrate millis
const int CALIBTIME = 300;
// high current warning
const int CURRENT_LIMIT = 1000;
// low bat warning
const int LOW_BAT = 10000;
// PID loop time
const int LOOPTIME = 180;
// samples for Amp average
const int NUMREADINGS = 10;


/****************************************************************************************************/
/****************************************************************************************************/


int readings[NUMREADINGS];
// loop timing
unsigned long lastMilliPrint = 0;
// loop timing
unsigned long loopMilli = 0;
// speed timing
unsigned long speedMilli = 0;
//PWM
double PWM_valA = 0;                               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
double PWM_valB = 0;                               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
//Bateria
int voltage = 0;                                // in mV
int current = 0;                                // in mA

double volatile countA = 0;                        // rev counter
double volatile countB = 0;                        // rev counter


/****************************************************************************************************/
/****************************************************************************************************/


//Velocidade solicitada
double speed_reqA = 0;                           // speed (Set Point)
double speed_reqB = 0;                           // speed (Set Point)

//Velocidade atual
double speed_actA = 0;                             // speed (actual value)
double speed_actB = 0;                             // speed (actual value)

// Filas para calibragem por media movel
QueueArray<int> calibA;
QueueArray<int> calibB;
QueueArray<long> mediaA;
QueueArray<long> mediaB;

// Peso dos motores
double pesoA = 1;
double pesoB = 1;

// Numero de amostras para a media movel
int NUM_AMOSTRA = 9;

// Flags
boolean calibrating = false;
char dirA = '+';                // Flag de direção motor A
char dirB = '+';                // Flag de direção motor B

//Comandos
int pidCmd = 0;
double cmdA = 0;                // Comando rescebido pelo controlador
double cmdB = 0;                // Comando rescebido pelo controlador
double oldcmdA = 0;             // Comando anterior rescebido pelo controlador
double oldcmdB = 0;             // Comando anterior rescebido pelo controlador
int atualizado = 0;             // flag atualização PID


/****************************************************************************************************/
/****************************************************************************************************/


/*/PID config
float Kp = 0.7;                                // PID proportional control Gain
float Ki = 0.3;
float Kd = 0.2;                                // PID Derivitave control gain

// Variaveis do Auto Tune
byte atAON;
byte atBON;

//Varaveis myPID do tipo PID
//Specify the links and initial tuning parameters
//PID MPID(&input,      &output,   &setpoint,   Kp, Ki, Kd, OPITON);
PID myPIDA(&speed_actA, &PWM_valA, &speed_reqA, Kp, Ki, Kd, P_ON_M, DIRECT);
PID myPIDB(&speed_actB, &PWM_valB, &speed_reqB, Kp, Ki, Kd, P_ON_M, DIRECT);

// AutoTune Kp Ki kD
//PID_ATune aTune(&input, &output);
PID_ATune atPIDA(&speed_actA, &PWM_valA);
PID_ATune atPIDB(&speed_actB, &PWM_valB);


PID myPIDX(&speed_actA, &PWM_valA, &speed_reqA, Kp, Ki, Kd, DIRECT);


/****************************************************************************************************/
/****************************************************************************************************/


#define START_CMD_CHAR '*'
#define DIV_CMD_CHAR '|'
#define END_CMD_CHAR '#'
#define CALIB_CMD_CHAR '@'

/****************************************************************************************************/
/****************************************************************************************************/

int treta = 0;
int treta2 = 0;
char get_char = ' ';


/****************************************************************************************************/
/****************************************************************************************************/


void rencoderA() {                               // pulse and direction, direct port reading to save cycles
    countA++;    // if(digitalRead(encodPinB1)==HIGH)   count ++;
}


/****************************************************************************************************/
/****************************************************************************************************/

void rencoderB() {
    countB++;
}


/****************************************************************************************************/
/****************************************************************************************************/


void setup() {
    // Current external ref is 3.3V
    analogReference(EXTERNAL);

    Serial.begin(38400);
    /*/ PID setado para automatico
    myPIDA.SetMode(AUTOMATIC);
    myPIDB.SetMode(AUTOMATIC); */

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
    digitalWrite(motorSTBY, HIGH);
}

/****************************************************************************************************/
/****************************************************************************************************/


// display data
void printMotorInfo() {
    //Controle de tempo (evitar excesso de prints)
    if ((millis() - lastMilliPrint) >= 100) {
        lastMilliPrint = millis();

        Serial.print(int(speed_actA));
        Serial.print(DIV_CMD_CHAR);
        Serial.println(int(speed_actB));

        /*/Motor A informações
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
        */
    }
}

/*****************************************************************************************/
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

/****************************************************************************************************/
/****************************************************************************************************/

void getMotorData() {                                      // calculate speed, volts and Amps
    // static long countAntA = 0;                                      // last count
    // static long countAntB = 0;                                      // last count

/*****************************************/
    //Desabilita interrupcao durante o calculo
    detachInterrupt(digitalPinToInterrupt(encodPinA1));
    detachInterrupt(digitalPinToInterrupt(encodPinB1));

    //speed_actA = (60 * 1000 / pulsos) / (millis() - oldtime) * countA;
    //speed_actB = (60 * 1000 / pulsos) / (millis() - oldtime) * countB;

    // Atualiza contagens do encoder
    speed_actA = countA;
    speed_actB = countB;
    speedMilli = millis();
    //Zera contagem
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

/****************************************************************************************************/
/****************************************************************************************************/

int updatePid(int command, int targetValue, int currentValue) {  // compute PWM value
    float pidTerm = 0;                                             // PID correction
    int error = 0;
    static int last_error = 0;

    error = abs(targetValue) - abs(currentValue);

    // pidTerm = (Kp * error) + (Kd * (error - last_error));

    last_error = error;

    return constrain(command + int(pidTerm), 0, 255);
}

/****************************************************************************************************/
/****************************************************************************************************/

void motorupdate() {
    // + + Ambos para frente
    if (PWM_valA > 0 && PWM_valB > 0) {
        digitalWrite(InA1, HIGH);
        digitalWrite(InA2, LOW);
        digitalWrite(InB1, HIGH);
        digitalWrite(InB2, LOW);
        digitalWrite(motorSTBY, HIGH);
        dirA = dirB = '+';
    }
        // + - Curva a direita (B)
    else if (PWM_valA > 0 && PWM_valB < 0) {
        PWM_valB *= PWM_valB * -1;
        digitalWrite(InA1, HIGH);
        digitalWrite(InA2, LOW);
        digitalWrite(InB1, LOW);
        digitalWrite(InB2, HIGH);
        digitalWrite(motorSTBY, HIGH);
        dirA = '+';
        dirB = '-';
    }
        // - + Curva a esquerda(A)
    else if (PWM_valA < 0 && PWM_valB > 0) {
        PWM_valA *= -1;
        digitalWrite(InA1, LOW);
        digitalWrite(InA2, HIGH);
        digitalWrite(InB1, HIGH);
        digitalWrite(InB2, LOW);
        digitalWrite(motorSTBY, HIGH);
        dirA = '-';
        dirB = '+';
    }
        // - - Ambos para tras
    else if (PWM_valA < 0 && PWM_valB < 0) {
        PWM_valA *= -1;
        PWM_valB *= -1;
        digitalWrite(InA1, LOW);
        digitalWrite(InA2, HIGH);
        digitalWrite(InB1, LOW);
        digitalWrite(InB2, HIGH);
        digitalWrite(motorSTBY, HIGH);
        dirA = dirB = '-';
    }
        // 0 0 Motores parados
    else if (PWM_valA == 0 && PWM_valB == 0) {
        digitalWrite(InA1, LOW);
        digitalWrite(InA2, LOW);
        digitalWrite(InB1, LOW);
        digitalWrite(InB2, LOW);
        digitalWrite(motorSTBY, HIGH);
    }
    PWM_valA *= pesoA;
    PWM_valB *= pesoB;
    PWM_valA = int(PWM_valA);
    PWM_valB = int(PWM_valB);
    analogWrite(PWMA, PWM_valA);
    analogWrite(PWMB, PWM_valB);
    getMotorData();
}

/****************************************************************************************************/
/****************************************************************************************************/

void motorRefresh() {
    PWM_valA *= pesoA;
    PWM_valB *= pesoB;
    PWM_valA = int(PWM_valA);
    PWM_valB = int(PWM_valB);
    analogWrite(PWMA, PWM_valA);
    analogWrite(PWMB, PWM_valB);
}

/****************************************************************************************************/
/****************************************************************************************************/
/*
void pidAUpdate() {
    if (atAON != 0) {
        Kp = atPIDA.GetKp();
        Ki = atPIDA.GetKi();
        Kd = atPIDA.GetKd();
        myPIDA.SetTunings(Kp, Ki, Kd);
        Serial.println("=== AUTO TUNE A ===");
    }
    myPIDA.Compute();
    PWM_valA = constrain(PWM_valA, 0, 255);
}

/****************************************************************************************************/
/****************************************************************************************************/
/*
void pidBUpdate() {
    if (atBON != 0) {
        Kp = atPIDB.GetKp();
        Ki = atPIDB.GetKi();
        Kd = atPIDB.GetKd();
        myPIDB.SetTunings(Kp, Ki, Kd);
        Serial.println("=== AUTO TUNE A ===");
    }
    myPIDB.Compute();
    PWM_valB = constrain(PWM_valB, 0, 255);
}

/****************************************************************************************************/

/****************************************************************************************************/
/****************************************************************************************************/
void detach() {
    detachInterrupt(digitalPinToInterrupt(encodPinB1));
    detachInterrupt(digitalPinToInterrupt(encodPinA1));
}

void attach() {
//Habilita interrupcao
    attachInterrupt(digitalPinToInterrupt(encodPinB1), rencoderB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encodPinA1), rencoderA, CHANGE);
}

/****************************************************************************************************/
/****************************************************************************************************/

boolean processMedia() {
    int _soma = 0;
    int _mark = 0;
    while (!calibA.isEmpty()) {
        _soma += calibA.pop();
        _mark++;
    }
    mediaA.push(_soma / _mark);

    _soma = _mark = 0;

    while (!calibB.isEmpty()) {
        _soma += calibB.pop();
        _mark++;
    }
    mediaB.push(_soma / _mark);
    return true;
}

/****************************************************************************************************/
/****************************************************************************************************/


boolean getAmostras(int _PWM) {
    // Set PWM
    PWM_valB = PWM_valA = _PWM;
    motorRefresh();

    // Media de NUM_AMOSTRAS com PWM _PWM
    double calibMillis = millis();
    detach();
    countB = countA = 0;
    attach();

    while (PWM_valA == _PWM) {
        for (int i = 0; i < NUM_AMOSTRA; ++i) {
            while ((millis() - calibMillis) <= CALIBTIME) {
            }
            // Desarma interrupts
            detach();

            calibA.push(countA);
            calibB.push(countB);
            // Log
            Serial.print(countA);
            Serial.print("<- A | B ->");
            Serial.println(countB);
            // Zera contadores de marcos do encoder
            countB = countA = 0;
            // Arma interrupsts
            attach();

            calibMillis = millis();
        }
        if (processMedia()) {
            Serial.print("Calibrado PWM=");
            Serial.println(_PWM);

            Serial.print(mediaA.front());
            Serial.print(" <- Méd.A | Méd.B -> ");
            Serial.println(mediaB.front());
        }
        PWM_valA = PWM_valB = 1;
        motorRefresh();
    }
}

/****************************************************************************************************/
/****************************************************************************************************/



/****************************************************************************************************/
/******************************* Calibrando médias de velocidade ************************************/
/****************************************************************************************************/

boolean setMarc() {
    PWM_valA = PWM_valB = 50;
    motorupdate();
    PWM_valA = PWM_valB = LOW_SPEED;
    motorRefresh();
    // Garantir ambos encoder em falso
    // True True
    while (digitalRead(encodPinA1) && digitalRead(encodPinB1)) {
        PWM_valA = PWM_valB = LOW_SPEED;
        motorRefresh();
    }
    PWM_valA = PWM_valB = 1;
    motorRefresh();
    // Log
    Serial.print(digitalRead(encodPinA1));
    Serial.print("<- encodA | encodB ->");
    Serial.println(digitalRead(encodPinB1));

    // True False
    while (digitalRead(encodPinA1)) {
        PWM_valA = LOW_SPEED;
        motorRefresh();
    }
    PWM_valA = PWM_valB = 1;
    motorRefresh();
    // Log
    Serial.print(digitalRead(encodPinA1));
    Serial.print("<- encodA | encodB ->");
    Serial.println(digitalRead(encodPinB1));

    // Flase True
    while (digitalRead(encodPinB1)) {
        PWM_valB = LOW_SPEED;
        motorRefresh();
    }
    PWM_valA = PWM_valB = 1;
    motorRefresh();
    // Log
    Serial.print(digitalRead(encodPinA1));
    Serial.print("<- encodA | encodB ->");
    Serial.println(digitalRead(encodPinB1));

    // Rodar até tornar verdadeiro *******************************************************
    while (!digitalRead(encodPinA1)) {
        PWM_valA = LOW_SPEED;
        motorRefresh();
    }
    PWM_valA = PWM_valB = 1;
    motorRefresh();
    // Log
    Serial.print(digitalRead(encodPinA1));
    Serial.print("<- encodA | encodB ->");
    Serial.println(digitalRead(encodPinB1));

    while (!digitalRead(encodPinB1)) {
        PWM_valB = LOW_SPEED;
        motorRefresh();
    }
    PWM_valA = PWM_valB = 0;
    motorRefresh();
    // Log
    Serial.println("Alinhados ===");
    Serial.print(digitalRead(encodPinA1));
    Serial.print("<- encodA | encodB ->");
    Serial.println(digitalRead(encodPinB1));
    return true;
}



/****************************************************************************************************/
/****************************************************************************************************/

// Processa os pesos com o vetor de medias
boolean finalMedia() {
    int _soma = 0;
    int _mark = 0;

    while (!mediaA.isEmpty()) {
        _soma += mediaA.pop();
        _mark++;
    }
    pesoA = _soma / _mark;

    _soma = _mark = 0;
    while (!mediaB.isEmpty()) {
        _soma += mediaB.pop();
        _mark++;
    }
    pesoB = _soma / _mark;

    if (pesoA > pesoB) {
        pesoA = pesoB / pesoA;
        pesoB = 1;
    } else if (pesoA < pesoB) {
        pesoB = pesoA / pesoB;
        pesoA = 1;
    } else if (pesoA == pesoB) {
        pesoA = pesoB = 1;
    } else {
        // Log de erro em caso de cmd não localizado
        Serial.println("#### PESOS não mapeados #####");
        Serial.print(pesoA);
        Serial.print(" <- pesoA pesoB -> ");
        Serial.println(pesoB);
    }
    return true;
}

/****************************************************************************************************/
/****************************************************************************************************/

boolean calibrate() {
    calibrating = true;
    Serial.print(setMarc());

    PWM_valA = PWM_valB = 1;
    motorRefresh();

    Serial.println(getAmostras(255));

    Serial.println(getAmostras(200));

    Serial.println(getAmostras(150));

    Serial.println(getAmostras(100));

    if (finalMedia()) {
        Serial.println("~ Calibragem finalizada. ~");
        Serial.print(pesoA);
        Serial.print(" <- pesoA pesoB -> ");
        Serial.println(pesoB);
    }
    calibrating = false;
}


/****************************************************************************************************/
/****************************************************************************************************/






void loop() {
    // Esvazia comandos recebidos
    //getParam(); // check keyboard

    // Espera por comando
    if (Serial.available()) {
        // Parce do primero char do comando
        get_char = Serial.read();
        if (get_char == CALIB_CMD_CHAR) calibrate();
        // Se diferente de comando inicial retorna loop
        if (get_char != START_CMD_CHAR) return;
        //Salva comando anteriores para comparação
        oldcmdA = cmdA;
        oldcmdB = cmdB;
        //Parceia os proximos 2 inteiros descatando o '|'
        cmdA = Serial.parseInt();
        cmdB = Serial.parseInt();

        //Serial.print("cmdA: ");
        //Serial.print(cmdA);
        //Serial.print(" cmdB: ");
        //Serial.println(cmdB);
        //Atualiza se cmdA ou cmdB seja diferente do anterior
        //if (oldcmdA != cmdA || oldcmdB != cmdB) {
        PWM_valA = map(cmdA, 0, 100, 0, 255);
        PWM_valB = map(cmdB, 0, 100, 0, 255);
        motorupdate();
        loopMilli = millis();
        treta = 0;
        atualizado = 0;
        get_char = ' ';
    }
}


/****************************************************************************************************/
/****************************************************************************************************/

