#include "MeanFilterLib.h"
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

float Umax = 70;
float Umin = -70;


// 1DOF
double en_2, en_1, un_2, un_1, un, difact, us, I_error_1, errPer, I_error, Ad_1, en_3;
// 2DOF
double Ad1_, yn_1;
// Estados
double Vk_1, dif_act;
// Cascada
double un1, us1,I_error_1_2,difact2,un2,us2,en_1_2,en_3_2;

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

  analogWrite(enableB, map(abs(us), 0, 100, 0, maxPWM));
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
        rpm = rpmFil.AddValue(rpm);
        P = (n * 360.0) / R;
        P = pFil.AddValue(P);
        controlEstados();
        direMotor();
        impresionSerial();
        timeold = millis();
        paso = 0;
      }

      if (elapsedTime >= 73000 ) {
        reiniciarVAR();
      }

    }



  }
}

void escalonPWM() {
  if (elapsedTime > 3001) Sp = 90; // PWMX60x80

  if (elapsedTime > 13000) Sp = -60; // PWMX60x80
  if (elapsedTime > 23000) Sp = 35; // PWMX60x80
  if (elapsedTime > 33000) Sp = -70; // PWMX60x80
  if (elapsedTime > 43000) Sp = 45; // PWMX60x80
  if (elapsedTime > 53000) Sp = -130; // PWMX60x80
  if (elapsedTime > 63000) Sp = 50; // PWMX60x80

}




void impresionSerial() {
  arduinoPWM = pwmFil.AddValue(arduinoPWM);

  Serial.print(Sp ); Serial.print(", "); Serial.print(P ); Serial.print(", "); Serial.print(arduinoPWM); Serial.print("\n");

  //Serial.print(elapsedTime ); Serial.print(", "); Serial.print(Sp ); Serial.print(", "); Serial.print(P ); Serial.print(", "); Serial.print(rpm ); Serial.print(", "); Serial.print(arduinoPWM); Serial.print("\n");
}



// CONTROLADORES




/*
  void control1DOF() {

  //  float Kp = 30;
  //  float Ki = 2;
  //  float Kd = 180;
  //  float N = 0.000000243345;
  //  float kaw = 0.1;

  //  float Kp = 80;
  //  float Ki = 2;
  //  float Kd = 40;
  //  float N = 0.000000243345;
  //  float kaw = 0.1;


  float Kp = 12.194801852041582;
  float Ki = 1.836264365827931;
  float Kd = 58.215749090323079;
  float N = 0.135321750384943;
  float kaw = 0.1;

  double en = (Sp * 0.0174533) - (P * 0.0174533);

  if (abs(en) > abs(0.065 * Sp * 0.0174533) ) {
    float Ap = Kp * en;
    float Ad = Kd * (en - en_1) + N * Ad_1;
    float I_error = en + I_error_1 + kaw * difact;
    float Ai = Ki * I_error;
    un = Ap + Ai + Ad;
    us = un;
    float ud = un;
    if (en<0 && abs(ud) < 18) us = -18;
    if ( en>0 && abs(ud) < 18) us = 18;


    if (un < -100) {
      us = -100;
    }
    if (un > 100) {
      us = 100;
    }


    en_1 = en;
    float en_2 = en_1;
    en_3 = en_2;
    I_error_1 = I_error;
    difact = us - un;
    Ad_1 = Ad;

    arduinoPWM = us;
    if(en_3 == en && abs(rpm) <2) us = us*1.6;
    if (abs(rpm) >= 0 && abs(rpm) < 0.7) us = us * 1.5;
    if (abs(rpm) >= 0.7 && abs(rpm) < 1) us = us * 1.4;
    if (abs(rpm) >= 1 && abs(rpm) < 2) us = us * 1.3;
    flagPWM = (us <  0) ? true : false;

  }


  }

*/

/*
  void control2DOF() {
  double Kp = 0.01357670445385118;
  double Ki = 1.822951239710286;
  double Kd = 10.464560395668343;
  double N = 0.782109908780621;
  double Kpd = 1.285776886757601;
  double Kdp = 13.408050560944484;
  double N1 = 0.997168715666942;
  double Ka = 0.1;

  double en = (Sp * 0.0174533) - (P * 0.0174533);

  if (abs(en) > abs(0.065 * Sp * 0.0174533) ) {

    float Ap = Kp * en;
    float Ad = Kd * (en - en_1) + N * Ad1_;
    float I_error = en + I_error_1 + Ka * difact;
    float Ai = Ki * I_error;
    float U1 = Ap + Ai + Ad;

    float Ap1 = Kpd * (P * 0.0174533);
    float Ad1 = Kdp * ((P * 0.0174533) - yn_1) + N1 * Ad_1;
    float U2 = Ap1 + Ad1;

    float Uc = U1 - U2;
    us = Uc;
    float ud = Uc;
    if (en < 0 && abs(ud) < 18) us = -18;
    if ( en > 0 && abs(ud) < 18) us = 18;

    if (Uc < -100) {
      us = -100;
    }
    if (Uc > 100) {
      us = 100;
    }

    en_1 = en;
    float en_2 = en_1;
    en_3 = en_2;
    yn_1 = P * 0.0174533;
    I_error_1 = I_error;
    difact = us - Uc;
    Ad_1 = Ad1;
    Ad1_ = Ad;


    arduinoPWM = us;
    if (en_3 == en && abs(rpm) < 2) us = us * 1.4;
    if (abs(rpm) >= 0 && abs(rpm) < 0.7) us = us * 1.3;
    //if (abs(rpm) >= 0.7 && abs(rpm) < 1) us = us * 1.2;
    //if (abs(rpm) >= 1 && abs(rpm) < 2) us = us * 1.1;
    flagPWM = (us <  0) ? true : false;

  }
  }

*/


  void controlEstados() {

  double kaw = 0.1;
  double CL[] = {1, 0};
  double Ki[] = {13.038404810405284};
  double Kes[] = {8.392729648618287, 6.196276250778606};
  double X_est[] = {(P * 0.0174533), (rpm * 0.104719755)}; // Corregir según valores de P y rpm
  double backlashCompensation = 30;
  double Kid = Ki[0] * tMuestro / 1000;

  double Pv = CL[0] * X_est[0] + CL[1] * X_est[1]; // Corregido el acceso a los elementos de los arreglos
  double  en = (Sp * 0.0174533) - X_est[0]; // Corregido el acceso al elemento del arreglo X_est

  if (abs(en) > abs(0.065 * Sp * 0.0174533)) {

    double Vk = en + Vk_1 + kaw * dif_act;
    double un = Kid * Vk - X_est[1] * Kes[0]; // Corregido el acceso a los elementos de los arreglos


/*
    // Implementación del compensador de backlash
    if (en > 0 && en_1 < 0) { // Intento de movimiento positivo
      un += backlashCompensation; // Agregamos una compensación positiva
    } else if (en < 0 && en_1 > 0) { // Intento de movimiento negativo
      un -= backlashCompensation; // Agregamos una compensación negativa
    }
*/
    us = un;
    double  ud = un;

    if (en < 0 && abs(ud) < 25) us = -25;
    if ( en > 0 && abs(ud) < 25) us = 25;

   // if (en < 0 && abs(ud) < 10 && abs(Sp) < 40) us = -10;
    //if ( en > 0 && abs(ud) < 10 && abs(Sp) < 40) us = 10;

    
    if (un > Umax) {
      us = Umax;
    }
    if (un < Umin) {
      us = Umin;
    }


 if (abs(rpm) <= 0.7 && en > 0) us = (us+10);
 if (abs(rpm) <= 0.7 && en < 0) us = (us-10);


//if (abs(rpm) <= 0.7 && en > 0) us = (us+15);
//if (abs(rpm) <= 0.7 && en < 0) us = (us+15);


    dif_act = us - un;
    Vk_1 = Vk;
    arduinoPWM = us;
/*
    //if (en_3 == en && elapsedTime >= 4000) us = us * 3.3;
    //if (en_3 == en && elapsedTime < 4000) us = us * 2;
    if (en_3 == en&& abs(P) < 100) us = us * 3.3;
        if (en_3 == en&& abs(P) > 100) us = us * 3.1;

*/
    en_1 = en;
    float en_2 = en_1;
    en_3 = en_2;



/*

    // mayor
    if (abs(rpm) >= 0.7 && abs(rpm) < 3 && (abs(Sp) > abs(P)) && abs(P) < 100) us = us * 3;
    if (abs(rpm) >= 3 && abs(rpm) < 7 && (abs(Sp) > abs(P)) && abs(P) < 100) us = us * 2.8;
    if (abs(rpm) >= 7 && abs(rpm) < 10 && (abs(Sp) > abs(P)) && abs(P) < 100) us = us * 2.6;
    if (abs(rpm) >= 10 && abs(rpm) < 15 && (abs(Sp) > abs(P)) && abs(P) < 100) us = us * 2.4;
*/




    flagPWM = (us <  0) ? true : false;

  }


  }



/*
void controlCascada() {


  float Kp1 = 0;
  float Ki1 = 93.836264365827931;
  float kaw1 = 0.1;

  float Kp2 = 0.09194801852041582;
  float Ki2 = 2.03836264365827931;
  float kaw2 = 0.1;

  double en = (Sp * 0.0174533) - (P * 0.0174533);

  if (abs(en) > abs(0.065 * Sp * 0.0174533) ) {
    float Ap = Kp1 * en;
    float I_error = en + I_error_1 + kaw1 * difact;
    float Ai = Ki1 * I_error;
    un1 = Ap + Ai;
    us1 = un1;
    float ud = un1;

    if (un1 < -40) {
      us1 = -40;
    }
    if (un1 > 40) {
      us1 = 40;
    }

    en_1 = en;
    float en_2 = en_1;
    en_3 = en_2;
    I_error_1 = I_error;
    difact = us - un;


    double en2 = (us1) - (rpm * 0.104719755);

    float Ap2 = Kp2 * en2;
    float I_error_2 = en + I_error_1_2 + kaw2 * difact2;
    float Ai2 = Ki2 * I_error_2;
    un2 = Ap2 + Ai2;
    us2 = un2;
    float ud2 = un2;

            if (en2<0 && abs(ud2) < 18) us2 = -18;
        if ( en2>0 && abs(ud2) < 18) us2 = 18;



    if (un2 < -100) {
      us2 = -100;
    }
    if (un2 > 100) {
      us2 = 100;
    }


    en_1_2 = en2;
    float en_2_2 = en_1_2;
    en_3_2 = en_2_2;
    I_error_1_2= I_error_2;
    difact2 = us2 - un2;

    arduinoPWM = us2;

    /*
    if (en_3_2 == en2 && abs(rpm) < 2) us2 = us2 * 1.6;
    if (abs(rpm) >= 0 && abs(rpm) < 0.7) us2 = us2 * 1.5;
    if (abs(rpm) >= 0.7 && abs(rpm) < 1) us2 = us2 * 1.4;
    if (abs(rpm) >= 1 && abs(rpm) < 2) us2 = us2 * 1.3;
    
    flagPWM = (us2 <  0) ? true : false;

  }

}

*/
