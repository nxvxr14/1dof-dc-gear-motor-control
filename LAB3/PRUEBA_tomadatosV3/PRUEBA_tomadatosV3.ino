#include "MeanFilterLib.h"
#include <BasicLinearAlgebra.h>
#include <SimpleKalmanFilter.h>

MeanFilter<float> pwmFil(3);
MeanFilter<float> pFil(3);
MeanFilter<float> rpmFil(3);

SimpleKalmanFilter KpwmFil(0.0001, 0.0001, 0.000001);
SimpleKalmanFilter KpFil(0.0001, 0.0001, 0.000001);
SimpleKalmanFilter KrpmFil(0.0001, 0.0001, 0.000001);


const byte Encoder_C1 = 2; // Cable amarillo pin 3 digital
const byte Encoder_C2 = 3; // Cable verde al pin 4 digital
byte Encoder_C1Last;
int paso = 0;
boolean direccion, flagPWM, autoincrement;

int entrada3 = 13;
int entrada4 = 12;
int enableB = 11;

int maxPWM = 130; // VCC 7V

int userE, Sp;

const int tMuestro = 30;

volatile int  n    = 0;
volatile byte ant  = 0;
volatile byte act  = 0;

double P = 0;
double Pf = 0;
double R = 3840;

float arduinoPWM;

float rpm = 0;
float rpmf = 0;
unsigned long timeold = 0;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;

float Du = 0;
float ut,us = 0;


void reiniciarVAR() {
  autoincrement = false;
  startTime = 0; // Reiniciar el tiempo de inicio si autoincrement es false
  elapsedTime = 0;
  timeold = 0;
  rpm = 0;
  P = 0;
  Sp = 0;
  flagPWM = false;
  analogWrite(enableB, 0); // Equivalente a 50% del máximo (128 de 255)
}


void calculapulso() {

  int Lstate = digitalRead(Encoder_C1);
  if ((Encoder_C1Last == LOW) && Lstate == HIGH)
  {
    int val = digitalRead(Encoder_C2);
    if (val == LOW && direccion)
    {
      direccion = false; //Reverse
    }
    else if (val == HIGH && !direccion)
    {
      direccion = true;  //Forward
    }
  }
  Encoder_C1Last = Lstate;

  if (!direccion)  paso++;
  else  paso--;


  ant = act;

  if (digitalRead(Encoder_C1)) bitSet(act, 1); else bitClear(act, 1);
  if (digitalRead(Encoder_C2)) bitSet(act, 0); else bitClear(act, 0);



  if (ant == 2 && act == 0) n++;
  if (ant == 0 && act == 1) n++;
  if (ant == 3 && act == 2) n++;
  if (ant == 1 && act == 3) n++;

  if (ant == 1 && act == 0) n--;
  if (ant == 3 && act == 1) n--;
  if (ant == 0 && act == 2) n--;
  if (ant == 2 && act == 3) n--;

}


void direMotor() {
  // Cambia la dirección del motor y establece la velocidad
  if (flagPWM) {
    digitalWrite(entrada3, LOW);
    digitalWrite(entrada4, HIGH);

  } else {

    // IZQUIERDA POSITIVO
    digitalWrite(entrada3, HIGH);
    digitalWrite(entrada4, LOW);
  }

  analogWrite(enableB, map(abs(arduinoPWM), 0, 100, 0, maxPWM));
}

void setup()
{
  Serial.begin(9600);
  pinMode(entrada3, OUTPUT);
  pinMode(entrada4, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(Encoder_C2, INPUT);
  flagPWM = false;
  autoincrement = false;

  attachInterrupt(digitalPinToInterrupt(Encoder_C1), calculapulso, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_C2), calculapulso, CHANGE);
  Serial.println("LISTO");

}

void loop()
{


  if (Serial.available()) {       // Verificar si hay datos disponibles en el puerto serial
    userE = Serial.parseInt();  // Leer el valor entero enviado por el puerto serial
    // Verificar si el valor recibido está dentro del rango válido (0-100)
    if (userE != 0 && userE != 404 ) {
      flagPWM = (userE <  0) ? true : false;
      autoincrement = true;
      //Sp = userE; // Para enviar por teclado

    }

    if (userE == 404) {
      reiniciarVAR();

    }

  }

  if (autoincrement == true) {
    if (startTime == 0) {
      startTime = millis(); // Registrar el tiempo de inicio
    }

    elapsedTime = millis() - startTime; // Calcular el tiempo transcurrido



    if (elapsedTime > 3000) {




      if (millis() - timeold >= tMuestro) {
        escalonPWM();
        rpm = (paso * (0.100 / (millis() - timeold)) * (60000 / 360)) * -1 ;
        rpmf = KrpmFil.updateEstimate(rpm);
        //rpmf = rpmFil.AddValue(rpm);
        P = (n * 360.0) / R;
        //Pf = pFil.AddValue(P);
        Pf = KpFil.updateEstimate(P);

        direMotor();
        impresionSerial();
        timeold = millis();
        paso = 0;
      }

      if (elapsedTime >= 43000 ) {
        reiniciarVAR();
      }

    }



  }
}

void escalonPWM() {
  if (elapsedTime > 3001) arduinoPWM = 30; // PWMX60x80

  if (elapsedTime > 13000) arduinoPWM = -50; // PWMX60x80
  if (elapsedTime > 23000) arduinoPWM = 35; // PWMX60x80
  if (elapsedTime > 33000) arduinoPWM = -25; // PWMX60x80
//  if (elapsedTime > 43000) arduinoPWM = 30; // PWMX60x80
//  if (elapsedTime > 53000) arduinoPWM = -40; // PWMX60x80
//  if (elapsedTime > 63000) arduinoPWM = 28; // PWMX60x80

    flagPWM = (arduinoPWM <  0) ? true : false;


}




void impresionSerial() {

  Serial.print(elapsedTime); Serial.print(", "); Serial.print(P ); Serial.print(", "); Serial.print(Pf ); Serial.print(", ");  Serial.print(rpm ); Serial.print(", "); Serial.print(rpmf ); Serial.print(", ");   Serial.print(arduinoPWM);Serial.print("\n");

  //Serial.print(elapsedTime ); Serial.print(", "); Serial.print(Sp ); Serial.print(", "); Serial.print(P ); Serial.print(", "); Serial.print(rpm ); Serial.print(", "); Serial.print(arduinoPWM); Serial.print("\n");
}
