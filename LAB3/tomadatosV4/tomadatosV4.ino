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

float Du = 0;
float ut, us = 0;
float u1max = 100;
float u1min = -100;
float en_1, en_2, en_3 = 0;

//KALMAN2DOF
/*

BLA::Matrix<4, 4> Gc = {0.523966818008876, 0.0782855124325541, -0.100820826191194, 0.105348989718112,
                        0.0132088738475635, 0.991367484590253, 0.0289530105661962, 0.000651705484506732,
                        0.791279836062320, -0.555522722213387, 0.909698456385056, 0.0619332601210404,
                        0, 0, 0, 1
                       };
BLA::Matrix<4, 2> Hc = {0.00174756461598906, -0.00174430609133644,
                        5.03642823964450e-06, 3.97073674635666e-05,
                        0.000651713016402831, -0.000663568108338496,
                        0.0300000000000000, -0.0300000000000000
                       };
BLA::Matrix<1, 4> Cc = { -102.796775094683, 18.8434497806398, -37.6657368965828, 44.7213595499950};
BLA::Matrix<1, 2> Dc = {0, 0};
BLA::Matrix<4, 1> Ka = {0, 0, 0, 0};
BLA::Matrix<4, 1> xk = {0, 0, 0, 0}; // Inicializar xk
BLA::Matrix<4, 1> xk_1 = {0, 0, 0, 0};
BLA::Matrix<1, 1> u = {0};
*/
  //DISCRETADESACOPLADO
  BLA::Matrix<3, 3> GL = {0.784437384092603, 0.00531930682271875, -0.0168735214917902,
                        0.0147638228309042, 0.991053908598640, 0.0294892447225049,
                        0.940790474483479, -0.592389086150106, 0.960114981651867
                       };
  BLA::Matrix<3, 1> HL = {0.00279792290617123,
                        1.58234863317437e-05,
                        0.00154787234807474
                       };
  BLA::Matrix<1, 3> CL = {0, 1, 0};
  BLA::Matrix<1, 1> Ki = {2.208069660656375};
  BLA::Matrix<1, 3> Kest = {106.389627162668, 4.54202985096176, 22.5481367796194};
  /*
  BLA::Matrix<3, 1> L = { 0.0908370092518715,
                        0.122492624265940,
                        -1.34405909478463
                      }; // L - LUMBERGUER
                      */
                        BLA::Matrix<3, 1> L = { 1.97870165160901e-06,
                        4.44347573756390e-05,
                        -2.44210589797653e-05
                      }; // L - LUMBERGUER
  BLA::Matrix<1, 1> Ka = {0};
  BLA::Matrix<1, 1> u = {0};
  BLA::Matrix<1, 1> Vk = {0};
  BLA::Matrix<1, 1> Vk_1 = {0};
  BLA::Matrix<3, 1> Xob = {0, 0, 0};
  BLA::Matrix<3, 1> Xk_1 = {0, 0, 0};




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
        controlDesacoplado();
        direMotor();
        impresionSerial();
        timeold = millis();
        paso = 0;
      }

      if (elapsedTime >= 63000 ) {
        reiniciarVAR();
      }

    }



  }
}

void escalonPWM() {
  if (elapsedTime > 3001) Sp = 40; // PWMX60x80

  if (elapsedTime > 13000) Sp = 60; // PWMX60x80
  if (elapsedTime > 23000) Sp = -40; // PWMX60x80
  if (elapsedTime > 33000) Sp = -50; // PWMX60x80
  if (elapsedTime > 43000) Sp = 27; // PWMX60x80
  if (elapsedTime > 53000) Sp = -45; // PWMX60x80

}




void impresionSerial() {
  arduinoPWM = pwmFil.AddValue(arduinoPWM);

  //Serial.print(Sp ); Serial.print(", "); Serial.print(P ); Serial.print(", "); Serial.print(arduinoPWM); Serial.print("\n");

  Serial.print(elapsedTime ); Serial.print(", "); Serial.print(Sp ); Serial.print(", "); Serial.print(P ); Serial.print(", "); Serial.print(Xob(0,0) ); Serial.print(", "); Serial.print(Xob(1,0)*57.2958 ); Serial.print(", "); Serial.print(Xob(2,0) ); Serial.print(", "); Serial.print(arduinoPWM); Serial.print("\n");
}




// CONTROLADORES
/*
void controlKalman2DOF() {
  BLA::Matrix<2, 1> Var = {(Sp * 0.0174533), (P * 0.0174533)};


  double  en = (Sp * 0.0174533) - (P * 0.0174533); // Corregido el acceso al elemento del arreglo X_est

  if (abs(en) > abs(0.065 * Sp * 0.0174533)) {
    u = Cc * xk + Dc * Var;
    xk_1 = Gc * xk + Hc * Var - Ka * Du;


    float u1s = u(0, 0);


    //if (en < 0 && abs(P) > abs(Sp+2)) u1s = u1s*4;



    if (u(0, 0) >= u1max) {
      u1s = u1max;
    }
    if (u(0, 0) <= u1min) {
      u1s = u1min;
    }

    ut = u1s;
    arduinoPWM = ut;

    Du = u(0, 0) - ut;
    xk = xk_1;

    // mayor

    if (en_3 == en) ut = ut * 2;



    /*
         if (abs(rpm) >= 0.7 && abs(rpm) < 3 && (abs(Sp) > abs(P)) && abs(P) < 100) ut = ut * 2.6;

      if (abs(rpm) >= 3 && abs(rpm) < 7 && (abs(Sp) > abs(P)) && abs(P) < 100) ut = ut * 2.4;
      if (abs(rpm) >= 7 && abs(rpm) < 10 && (abs(Sp) > abs(P)) && abs(P) < 100) ut = ut * 2.2;
      if (abs(rpm) >= 10 && abs(rpm) < 15 && (abs(Sp) > abs(P)) && abs(P) < 100) ut = ut * 2;
    */
/*
    en_1 = en;
    float en_2 = en_1;
    en_3 = en_2;




    flagPWM = (ut <  0) ? true : false;

  }

}

*/



  void controlDesacoplado() {

  BLA::Matrix<1, 1> en = {(Sp * 0.0174533) - (P * 0.0174533)};
  BLA::Matrix<1, 1> Pv = {P * 0.0174533};

    double  enn = (Sp * 0.0174533) - (P * 0.0174533); // Corregido el acceso al elemento del arreglo X_est


  if (abs(en(0, 0)) > abs(0.065 * Sp * 0.0174533)) {

  Vk = en + Vk_1 - Ka * Du;
  u = Ki * Vk - Kest * Xk_1;
  Xob = (GL - L * CL) * Xk_1 + HL * u + L * Pv;

  us = u(0, 0);


  if (u(0, 0) >= u1max) {
    us = u1max;
  }
  if (u(0, 0) <= u1min) {
    us = u1min;
  }

  ut = us;
  arduinoPWM = ut;
  Du = u(0, 0) - ut;
  Xk_1 = Xob;
  Vk_1 = Vk;

    // mayor

    if (en_3 == enn) ut = ut * 1.8;



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
