#include <Arduino.h>
#include <SoftwareSerial.h>

const byte PIN_ANALOG_X = 0;
const byte PIN_ANALOG_Y = 1;
double cmdX;
double cmdY;
double sendA;
double sendB;
int looptime = 200;
int oldtime = 0;
String command = "";
//RX e TX para o Bluetooth
SoftwareSerial mySerial(10, 11);


void setup() {
    Serial.begin(9600);
    mySerial.begin(9600);
    oldtime = millis();
}


void loop() {
    if ((millis() - oldtime) >= looptime) {
        //Leitura do eixo Y = vertical = Potencias dos motores
        cmdY = analogRead(PIN_ANALOG_Y);
        //Leitura do eixo X = horizontal = Utilizado como "diferença de potencia entre A e B"
        cmdX = analogRead(PIN_ANALOG_X);

        /***************************************************************************/
        /*** as funções MAP são aqui utilizadas para "calibrar" o controle       ***/
        /*** Potenciometros em "neutro"(sem movimento) encontram-se com valor de ***/
        /*** 502 para o eixo X=0 e 509 para o eixo Y=0,                          ***/
        /***************************************************************************/
        if (cmdX >= 0 && cmdX <= 501) cmdX = map(cmdX, 0, 501, -100, 0);
        else cmdX = map(cmdX, 502, 1023, 0, 100);

        // 1 a 100 = para frente
        //-1 a -100 = para trás
        if (cmdY >= 0 && cmdY <= 509) cmdY = map(cmdY, 0, 509, -100, 0);
        else cmdY = map(cmdY, 510, 1023, 0, 100);
        /***********************************************************************************/
        sendA = sendB = cmdY;

        //cmdX é usado como proporção, ou seja cmdX/100 = X% ao multiplicar X% por cmdY
        //ou seja se cmy(potencia) for = 100 e cmdX(diferença) for de 25%
        //então um motor vai rodar com 100% da sua potencia e o outro com 75%
        if (cmdX < -1) {
            //Dividido por -100 pois o X é negativo, dando resultado positivo
            sendA -= (cmdY * (cmdX / -100));
        } else if (cmdX > 1) {
            sendB -= (cmdY * (cmdX / 100));
        }

        int a, b;
        a = sendA;
        b = sendB;

        //Concatenação dos comandos
        String espaco = "|";
        String vai = "*";
        vai = a + espaco + b;

        //Enviando via BT
        mySerial.print(vai);
        if (mySerial.available()) {
            while (mySerial.available()) { // While there is more to be read, keep reading.
                command += (char) mySerial.read();
            }
            Serial.println(command);
            command = ""; // No repeats
        }
        //Zera timer
        oldtime = millis();
    }
}



