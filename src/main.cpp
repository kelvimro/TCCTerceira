#include <Arduino.h>
#include <SoftwareSerial.h>
#include "../lib/QueueArray/QueueArray.h"

/****************************************************************************************************/
/****************************************************************************************************/

//Motor Esquerdo = A || 5 9 10
const int PWMA = 5;                      // PWM motor pin
const int InA1 = 9;                     // INA1 motor pin - Para frente
const int InA2 = 10;                    // INA2 motor pin - Para trás

//Motor Direito = B || 6 11 12
const int PWMB = 6;                       // PWM motor pin
const int InB1 = 11;                      // INB1 motor pin - Para frente
const int InB2 = 12;                      // INB2 motor pin - Para trás

const int motorSTBY = 13;  //STBY pin on TB6612FNG. Must be HIGH to enable motor

/****************************************************************************************************/
/****************************************************************************************************/


const int encodPinA1 = 2;                       // encoder A pin || 2
const int encodPinB1 = 3;                       // encoder B pin || 3

/****************************************************************************************************/
/****************************************************************************************************/


#define Vpin A7                      // battery monitoring analog pin
const int Apin = 1;                       // motor current monitoring analog pin

/****************************************************************************************************/
/****************************************************************************************************/

// Numero de amostras para a media movel
int NUM_AMOSTRA = 15;
// Lenta
const int LOW_SPEED = 25;
// Calibrate millis
const int CALIBMILLIS = 1000;
// samples for Amp average
const int NUMREADINGS = 10;


/****************************************************************************************************/
/****************************************************************************************************/


int readings[NUMREADINGS];
// loop timing
unsigned long lastMilliPrint = 0;
//PWM
double PWM_valA = 0;                               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
double PWM_valB = 0;                               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
//Bateria
int voltage = 0;                                // in mV
int current = 0;                                // in mA

int volatile countA = 0;                        // rev counter
int volatile countB = 0;                        // rev counter


/****************************************************************************************************/
/****************************************************************************************************/
//Velocidade atual
double speed_actA = 0;                             // speed (actual value)
double speed_actB = 0;                             // speed (actual value)

// Filas para calibragem por media movel
QueueArray<int> pwmArray;
QueueArray<int> calibA;
QueueArray<int> calibB;
QueueArray<double> mediaA;
QueueArray<double> mediaB;

// Peso dos motores
double pesoA = 1;
double pesoB = 1;


// Flags
boolean calibrating = false;

//Comandos
double cmdA = 0;                // Comando rescebido pelo controlador
double cmdB = 0;                // Comando rescebido pelo controlador


/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/


#define START_CMD_CHAR '*'
#define DIV_CMD_CHAR '|'
#define END_CMD_CHAR '#'
#define CALIB_CMD_CHAR '@'

/****************************************************************************************************/
/****************************************************************************************************/

char get_char = ' ';
static String print = " ";

/****************************************************************************************************/
/****************************************************************************************************/


void rencoderA() {
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

/*****************************************************************************************/
/*****************************************************************************************/
// remove signal noise
int digital_smooth(int value, int *data_array) {
    static int ndx;
    ndx = 0;
    static int count;
    count = 0;
    static int total;
    total = 0;
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
    // Atualiza contagens do encoder
    speed_actA = countA;
    speed_actB = countB;
    // speedMilli = millis();
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
/****************************************************************************************************/
/****************************************************************************************************/

void motorupdate() {
    // + + Ambos para frente
    if (PWM_valA >= 0 && PWM_valB >= 0) {
        digitalWrite(InA1, HIGH);
        digitalWrite(InA2, LOW);
        digitalWrite(InB1, HIGH);
        digitalWrite(InB2, LOW);
        digitalWrite(motorSTBY, HIGH);
    }
        // + - Curva a direita (B)
    else if (PWM_valA > 0 && PWM_valB < 0) {
        PWM_valB *= PWM_valB * -1;
        digitalWrite(InA1, HIGH);
        digitalWrite(InA2, LOW);
        digitalWrite(InB1, LOW);
        digitalWrite(InB2, HIGH);
        digitalWrite(motorSTBY, HIGH);
    }
        // - + Curva a esquerda(A)
    else if (PWM_valA < 0 && PWM_valB > 0) {
        PWM_valA *= -1;
        digitalWrite(InA1, LOW);
        digitalWrite(InA2, HIGH);
        digitalWrite(InB1, HIGH);
        digitalWrite(InB2, LOW);
        digitalWrite(motorSTBY, HIGH);
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
    }
    PWM_valA *= pesoA;
    PWM_valB *= pesoB;
    PWM_valA = int(PWM_valA);
    PWM_valB = int(PWM_valB);
    analogWrite(PWMA, PWM_valA);
    analogWrite(PWMB, PWM_valB);
    //getMotorData();
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


/****************************************************************************************************/
/****************************************************************************************************/


/****************************************************************************************************/

/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/

boolean processMedia() {
    static double _soma;
    _soma = 0;
    static double _med;
    _med = 0;
    static int _mark;
    _mark = 0;
    print = "\nMédia\t";
    while (!calibA.isEmpty()) {
        _soma += calibA.pop();
        _mark++;
    }
    _med = _soma / _mark;
    mediaA.push(_med);
    print += _med;                            // Imprime média de A
    _soma = 0;
    _med = 0;
    _mark = 0;
    while (!calibB.isEmpty()) {
        _soma += calibB.pop();
        _mark++;
    }
    _med = _soma / _mark;
    mediaB.push(_med);
    print += "\t";              // Imprime indicadores e média de B
    print += _med;
    delayMicroseconds(999);
    Serial.println(print);

    return true;
}

/****************************************************************************************************/
/****************************************************************************************************/


/****************************************************************************************************/
/******************************* Calibrando médias de velocidade ************************************/
/****************************************************************************************************/

void getAmostras(int _PWM) {
    print = "PWM =\t";
    print += _PWM;
    // Disables interrupts (you can re-enable them with interrupts()).
    // https://www.arduino.cc/en/Reference/NoInterrupts
    noInterrupts();
    // Media de NUM_AMOSTRAS com PWM _PWM
    // _motor A = motorA | B = motorB
    // Set PWM
    print += "\nA:\n";
    PWM_valA = _PWM;
    PWM_valB = 1;
    motorRefresh();
    countB = countA = 0;
    // calibMillis timer de loop
    static double calibMillis;
    calibMillis = millis();
    interrupts();
    while (PWM_valA >= 5) {
        for (int i = 0; i < NUM_AMOSTRA; ++i) {
            // Enquanto conta durante CALIBMILLIS, faça nada ;)
            while ((millis() - calibMillis) <= CALIBMILLIS) {
            }
            noInterrupts();            // Desarma interrupts
            calibA.push(countA);       // Adiciona a ultima contagem a fila de amostras
            print += countA;
            print += "\n";
            countA = 0;                // Zera contadores de marcos do encoder
            calibMillis = millis();    // Zera o contador de tempo
            interrupts();              // Arma interrupsts
        }
        // Reduz vel. mas mantem sentido
        PWM_valA = PWM_valB = 2;
        motorRefresh();
    }
    // Disables interrupts (you can re-enable them with interrupts()).
    // https://www.arduino.cc/en/Reference/NoInterrupts
    noInterrupts();
    // Set PWM - config inicial
    print += "\nB:\n";
    PWM_valA = 1;
    PWM_valB = _PWM;
    motorRefresh();
    countB = countA = 0;
    calibMillis = millis();
    interrupts();
    while (PWM_valB >= 5) {
        for (int i = 0; i < NUM_AMOSTRA; ++i) {
            // Enquanto conta durante CALIBMILLIS, faça nada :)
            while ((millis() - calibMillis) <= CALIBMILLIS) {
            }
            // Desarma interrupts
            noInterrupts();
            calibB.push(countB);
            print += countB;
            print += "\n";
            // Zera contadores de marcos do encoder
            countB = 0;
            calibMillis = millis();
            // Arma interrupsts
            interrupts();
        }
        PWM_valA = PWM_valB = 2;
        motorRefresh();
    }
    //Serial.print(print);
    print = " ";

    if (processMedia()) {
    }
}

/****************************************************************************************************/
/****************************************************************************************************/


void setMarc() {
    PWM_valA = PWM_valB = 50;
    motorupdate();
    delayMicroseconds(7000);
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
    // True False
    while (digitalRead(encodPinA1)) {
        PWM_valA = LOW_SPEED;
        motorRefresh();
    }
    PWM_valA = PWM_valB = 1;
    motorRefresh();
    // Flase True
    while (digitalRead(encodPinB1)) {
        PWM_valB = LOW_SPEED;
        motorRefresh();
    }
    PWM_valA = PWM_valB = 1;
    motorRefresh();
    // Rodar até tornar verdadeiro *******************************************************
    while (!digitalRead(encodPinA1)) {
        PWM_valA = LOW_SPEED;
        motorRefresh();
    }
    PWM_valA = PWM_valB = 1;
    motorRefresh();
    while (!digitalRead(encodPinB1)) {
        PWM_valB = LOW_SPEED;
        motorRefresh();
    }
    PWM_valA = PWM_valB = 0;
    motorRefresh();
    // Log
    print = "\n";
    print += digitalRead(encodPinA1);
    print += "\t<-encodA encodB->\t";
    print += digitalRead(encodPinB1);
    Serial.println(print);
    print = " ";
}



/****************************************************************************************************/
/****************************************************************************************************/

// Processa os pesos com o vetor de medias
boolean finalMedia() {
    static double _soma;
    _soma = 0;
    static double _med;
    _med = 0;
    static int _mark;
    _mark = 0;
    while (!mediaA.isEmpty()) {
        _soma += mediaA.pop();
        _mark++;
    }
    _med = _soma / _mark;
    pesoA = _med;

    _soma = _med = 0;
    _mark = 0;
    while (!mediaB.isEmpty()) {
        _soma += mediaB.pop();
        _mark++;
    }
    _med = _soma / _mark;
    pesoB = _med;

    if (pesoA > pesoB) {
        pesoA = pesoB / pesoA;
        pesoB = 1;
        return true;
    } else if (pesoA < pesoB) {
        pesoB = pesoA / pesoB;
        pesoA = 1;
        return true;
    } else if (pesoA == pesoB) {
        pesoA = pesoB = 1;
        return true;
    } else {
        // Log de erro em caso de cmd não localizado
        Serial.println("#### PESOS não mapeados #####");
        Serial.print(pesoA);
        Serial.print(" <- pesoA pesoB -> ");
        Serial.print(pesoB);
        return false;
    }
}

/****************************************************************************************************/
/****************************************************************************************************/

/****************************************************************************************************/
/****************************************************************************************************/

// Processa os pesos com o vetor de medias
boolean getPercent() {
    static double _soma;
    _soma = 0;
    static double _med;
    _med = 0;
    static int _mark;
    _mark = 0;
    while (!mediaA.isEmpty()) {
        _soma += mediaA.pop();
        _mark++;
    }
    _med = _soma / _mark;
    static double _pA;
    _pA = _med;

    _soma = _med = 0;
    _mark = 0;
    while (!mediaB.isEmpty()) {
        _soma += mediaB.pop();
        _mark++;
    }
    _med = _soma / _mark;
    static double _pB;
    _pB = _med;
    delayMicroseconds(8000);
    print = "\nPerc\t";
    print += ((_pA / _pB) * 100);
    print += "%";
    Serial.println(print);
}

/****************************************************************************************************/
/****************************************************************************************************/


/****************************************************************************************************/
/****************************************************************************************************/

// Processa os pesos com o vetor de medias
boolean finalTeste() {
    static double _soma;
    _soma = 0;
    static double _med;
    _med = 0;
    // Marcador #de testes
    static int _mark;
    _mark = 0;
    static String _print;
    _print = "\nTeste\n";
    while (!mediaA.isEmpty()) {
        _soma += mediaA.pop();
        _mark++;
    }
    _med = _soma / _mark;
    // pesoA para print, utilizado apenas localmente
    static double _pA;
    _pA = _med;

    _soma = _med = 0;
    _mark = 0;
    while (!mediaB.isEmpty()) {
        _soma += mediaB.pop();
        _mark++;
    }
    _med = _soma / _mark;
    // pesoB para print, utilizado apenas localmente
    static double _pB;
    _pB = _med;

    if (_pA > _pB) {
        _pA = _pB / _pA;
        _print += (_pA * 100);
    } else if (_pA < _pB) {
        _pB = _pA / _pB;
        _print += (_pB * 100);
    } else if (_pA == _pB) {
        _print += 100;
    } else {
        // Log de erro em caso de cmd não localizado
        Serial.println("#### PESOS não mapeados #####");
        Serial.print(_pA);
        Serial.print(" <- _pA _pB -> ");
        Serial.print(_pB);
        return false;
    }
    _print += "%\tde precisão\n";
    _print += pesoA;
    _print += "\t<-pesoA pesoB->\t";
    _print += pesoB;
    _print += "\n";
    _print += NUM_AMOSTRA;
    _print += "\t<-Amostras Invervalo(ms)->\t";
    _print += CALIBMILLIS;
    _print += "\n";
    Serial.print(_print);
    return true;
}

/****************************************************************************************************/
/****************************************************************************************************/



boolean calibrate() {
    calibrating = true;
    setMarc();

    PWM_valA = PWM_valB = 7;
    motorupdate();

    // Coleta todos os PWMs do array
    while (Serial.available()) {
        pwmArray.push(Serial.parseInt());
    }
    while (!pwmArray.isEmpty()) {
        static int _pwm;
        _pwm = pwmArray.pop();
        getAmostras(_pwm);
    }
    //getAmostras(150);
    //getPercent();
/*
    getAmostras(255);
    getAmostras(200);
    getAmostras(150);
    getAmostras(100);
*/
    if (finalMedia()) {
        print = "\n";
        print += "~Calibragem finalizada~\n";
        print += pesoA, 5;
        print += "\t<-pesoA pesoB->\t";
        print += pesoB, 5;
        print += "\n";
        print += NUM_AMOSTRA;
        print += "\t<-Amostras Invervalo(ms)->\t";
        print += CALIBMILLIS;
        Serial.println(print);
        print = " ";
        Serial.print(pesoA, 5);
        Serial.print(pesoB, 5);
    } else {
        Serial.print("+++ Med final False");
        return false;
    }
    /*
    setMarc();
    getAmostras(255);
    getAmostras(200);
    getAmostras(150);
    getAmostras(100);

    finalTeste();
    */

    calibrating = false;
    return true;
}

/****************************************************************************************************/
/****************************************************************************************************/
void loop() {
    // Espera por comando
    if (Serial.available()) {
        // Parce do primero char do comando
        get_char = Serial.read();
        // Primeoro Char for igual @ (CALIB_CMD_CHAR)
        if (get_char == CALIB_CMD_CHAR) calibrate();
        // Se diferente de comando inicial retorna loop
        if (get_char != START_CMD_CHAR) return;
        //Parceia os proximos 2 inteiros descatando o '|'
        cmdA = Serial.parseInt();
        cmdB = Serial.parseInt();
        PWM_valA = map(cmdA, 0, 100, 0, 255);
        PWM_valB = map(cmdB, 0, 100, 0, 255);
        motorupdate();
        print = "\n";
        print += "PWM SETS AS\n";
        print += PWM_valA;
        print += "\t<- PWMA pesoA ->\t";
        print += pesoA, 5;
        print += "\n";
        print += PWM_valB;
        print += "\t<- PWMB pesoB ->\t";
        print += pesoB, 5;
        Serial.print(print);
        print = " ";
    }
}


/****************************************************************************************************/
/****************************************************************************************************/

