#include "MeanFilterLib.h"
#include <BasicLinearAlgebra.h>

MeanFilter<float> pwmFil(3);
MeanFilter<float> pFil(3);
MeanFilter<float> rpmFil(3);

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
double R = 3840;

float arduinoPWM;

float rpm = 0;
unsigned long timeold = 0;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;

float ut, us = 0;
float u1max = 100;
float u1min = -100;
float en_1, en_2, en_3 = 0;

// MIXSYN
/*
  BLA::Matrix<4, 4> Gc = {0.999707105424673,0.00151771236953691,0.000689904726261718,-0.000339220737457422,
  -0.00151772398850250,0.756753569411887,-0.321971210474965,0.0952642884793470,
  0.000689878641928251,0.321971196611839,0.627937488408591,0.244883619731361,
  -0.000339193385452838,-0.0952642611575348,0.244883581403790,0.768717841221103};
  BLA::Matrix<4, 1> Hc = {13.3897425159846,
  34.4454472814104,
  -15.9073661414958,
  7.78030172576614};
  BLA::Matrix<1, 4> Cc = {0.100423050184375,-0.258340858953882,-0.119305245563825,0.0583522705240672};
  BLA::Matrix<1, 1> Dc = {34.7870470020576};
  BLA::Matrix<1, 1> Ka = {0};
  BLA::Matrix<1, 1> Du = {0};
  BLA::Matrix<1, 1> u = {0};
  BLA::Matrix<4, 1> xk = {0, 0, 0, 0};
  BLA::Matrix<4, 1> xk_1 = {0, 0, 0, 0};
*/

// LOOPSING 1DOF
/*
  BLA::Matrix<3, 3> Gc = {0.3963,  -0.3751, 0,
0.3751,  0.0019,  0,
0, 0, 1.0000};
  BLA::Matrix<3, 1> Hc = {73.1508,
-13.2603,
-293.4822};
  BLA::Matrix<1, 3> Cc = {-0.5486,  -0.0995, -0.0039};
  BLA::Matrix<1, 1> Dc = {64.6527};
  BLA::Matrix<1, 1> Ka = {0};
  BLA::Matrix<1, 1> Du = {0};
  BLA::Matrix<1, 1> u = {0};
  BLA::Matrix<3, 1> xk = {0, 0, 0};
  BLA::Matrix<3, 1> xk_1 = {0, 0, 0};
*/  

// NFCSYN

  BLA::Matrix<3, 3> Gc = {0.853862925588747,  0.187830959530800, 0,
-0.187830959530800,  0.954205726727783, 0,
0, 0, 1};
  BLA::Matrix<3, 1> Hc = {-8.29512388959399,
-1.99815784321956,
-101.934103302858};
  BLA::Matrix<1, 3> Cc = {-0.0622134291719549,  0.0149861838241466,  -0.00900000000000000};
  BLA::Matrix<1, 1> Dc = {0};
  BLA::Matrix<1, 1> Ka = {0};
  BLA::Matrix<1, 1> Du = {0};
  BLA::Matrix<1, 1> u = {0};
  BLA::Matrix<3, 1> xk = {0, 0, 0};
  BLA::Matrix<3, 1> xk_1 = {0, 0, 0};



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

  analogWrite(enableB, map(abs(ut), 0, 100, 0, maxPWM));
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
        //rpm = (paso * (0.100 / (millis() - timeold)) * (60000 / 360)) * -1 ;
        //rpm = rpmFil.AddValue(rpm);
        P = (n * 360.0) / R;
        P = pFil.AddValue(P);
        controlador();
        direMotor();
        impresionSerial();
        timeold = millis();
        paso = 0;
      }

      if (elapsedTime >= 113000 ) {
        reiniciarVAR();
      }

    }



  }
}

void escalonPWM() {
  if (elapsedTime > 3001) Sp = 50; // PWMX60x80
  if (elapsedTime > 13000) Sp = -60; // PWMX60x80
  if (elapsedTime > 23000) Sp = 40; // PWMX60x80
  if (elapsedTime > 33000) Sp = 50; // PWMX60x80
  if (elapsedTime > 43000) Sp = 70; // PWMX60x80
  if (elapsedTime > 53000) Sp = -70; // PWMX60x80
  if (elapsedTime > 63000) Sp = -30; // PWMX60x80
  if (elapsedTime > 73000) Sp = 30; // PWMX60x80
  if (elapsedTime > 83000) Sp = -40; // PWMX60x80
  if (elapsedTime > 93000) Sp = -50; // PWMX60x80
  if (elapsedTime > 103000) Sp = -60; // PWMX60x80
}




void impresionSerial() {
  arduinoPWM = pwmFil.AddValue(arduinoPWM);

  //Serial.print(Sp ); Serial.print(", "); Serial.print(P ); Serial.print(", "); Serial.print(arduinoPWM); Serial.print("\n");
  Serial.print(elapsedTime ); Serial.print(", "); Serial.print(Sp ); Serial.print(", "); Serial.print(P ); Serial.print(", "); Serial.print(arduinoPWM); Serial.print("\n");
;
}




// CONTROLADORES

  void controlador() {

  BLA::Matrix<1, 1> en = {(Sp * 0.0174533) - (P * 0.0174533)};

    double  enn = (Sp * 0.0174533) - (P * 0.0174533); // Corregido el acceso al elemento del arreglo X_est


  if (abs(en(0, 0)) > abs(0.072 * Sp * 0.0174533)) {

xk_1 = Gc*xk+Hc*en;
u = Cc*xk+Dc*en;

  us = u(0, 0);


  if (u(0, 0) >= u1max) {
    us = u1max;
  }
  if (u(0, 0) <= u1min) {
    us = u1min;
  }

  ut = us;
  //a
  Du = u(0, 0) - ut;
  xk = xk_1;

    // mayor

    if (en_3 == enn) ut = ut * 1.9;

  arduinoPWM = ut;


    /*
         if (abs(rpm) >= 0.7 && abs(rpm) < 3 && (abs(Sp) > abs(P)) && abs(P) < 100) ut = ut * 2.6;

      if (abs(rpm) >= 3 && abs(rpm) < 7 && (abs(Sp) > abs(P)) && abs(P) < 100) ut = ut * 2.4;
      if (abs(rpm) >= 7 && abs(rpm) < 10 && (abs(Sp) > abs(P)) && abs(P) < 100) ut = ut * 2.2;
      if (abs(rpm) >= 10 && abs(rpm) < 15 && (abs(Sp) > abs(P)) && abs(P) < 100) ut = ut * 2;
    */

    en_1 = enn;
    float en_2 = en_1;
    en_3 = en_2;



  flagPWM = (ut <  0) ? true : false;

  }


  }
