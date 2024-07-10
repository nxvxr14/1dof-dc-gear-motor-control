
const byte Encoder_C1 = 2; // Cable amarillo pin 3 digital
const byte Encoder_C2 = 3; // Cable verde al pin 4 digital
byte Encoder_C1Last;
int paso = 0;
boolean direccion, flagPWM, autoincrement;

int entrada3 = 13;
int entrada4 = 12;
int enableB = 11;

int maxPWM = 130; // VCC 7V

int userE, Sp, Spp;

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

// 2DOF
double en, Ap, Ad, Ai, Ap1, Ad1, U2, U1, Uc, us, yn_1, Ad1_, Ad_1;

// 1DOF
double en_2, en_1, un_2, un_1, un, difact, difact2;
double I_error_1, enn, errPer, I_error;

// CASCADA
double un1, un2, u1, u1_1, u1_2, en2, en2_1, en2_2, u2, u2_1, u2_2;

// ESTADOS
float Vk_1 = 0;
float dif_act = 0;
float Tm = 0.30;


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
      if (userE < 0) {
        flagPWM = true;
      } else {
        flagPWM = false;
      }
      autoincrement = true;
      //      Sp = userE; // Para enviar por teclado
      //      errPer = (0.07 * Sp * 0.0174533);

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

      if (elapsedTime > 3001) Sp = 90; // PWMX60x80
      
      if (elapsedTime > 13000) Sp = -60; // PWMX60x80
      if (elapsedTime > 23000) Sp = 40; // PWMX60x80
      if (elapsedTime > 33000) Sp = -40; // PWMX60x80
      if (elapsedTime > 43000) Sp = 20; // PWMX60x80
      if (elapsedTime > 53000) Sp = -50; // PWMX60x80


      if (millis() - timeold >= tMuestro) {
        rpm = (paso * (0.100 / (millis() - timeold)) * (60000 / 360)) * -1 ;
        P = (n * 360.0) / R;
        if (Sp < 0 ) {
          Spp = Sp * -1;
          errPer = (0.065 * Spp * 0.0174533);
        }
        if (Sp > 0 ) errPer = (0.065 * Sp * 0.0174533);


        control1DOF();
        direMotor();
        impresionSerial();
        timeold = millis();
        paso = 0;
      }

      if (elapsedTime >= 6300000 ) {
        reiniciarVAR();
      }

    }



  }
}


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



void impresionSerial() {
   Serial.print(Sp ); Serial.print(", "); Serial.print(P ); Serial.print(", ");Serial.print(arduinoPWM); Serial.print(", ");Serial.print(us); Serial.print("\n"); 

  //Serial.print(elapsedTime ); Serial.print(", "); Serial.print(Sp ); Serial.print(", "); Serial.print(P ); Serial.print(", "); Serial.print(rpm ); Serial.print(", "); Serial.print(arduinoPWM); Serial.print("\n");
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


// CONTROLADORES



void control1DOF() {
  
//    float Kp = -1.101543753570772; // MAS FUERTE
//    float Ki = 2.065219414420463; // MAS RAPIDO
//
//    float Kd = 30.183054627013306;
//    float N = 0.742509213223832;
//    float kaw = 0.1;
  

  float Kd = 180;
  float Kp = 30;
  float Ki = 2;
  float N = 0;
  float kaw = 0.1;
    


  en = (Sp * 0.0174533) - (P * 0.0174533);
  if (en < 0 ) enn = en * -1;

  if (en > errPer || enn > errPer ) {


  float Ap = Kp * en;
  float Ad = Kd * (en - en_1) + N * Ad_1;
  float I_error = en + I_error_1 + kaw * difact;
  float Ai = Ki * I_error;
  un = Ap + Ai + Ad;
  us = un;
  float ud = un;
  if(en <0 && abs(ud) < 25) us = -20; 
    if(en >0 && abs(ud) < 25) us = 20;



  if (un < -100) {
    us = -100;
  }
  if (un > 100) {
    us = 100;
  }


  en_1 = en;
  I_error_1 = I_error;
  difact = us - un;
  Ad_1 = Ad;

    arduinoPWM = us;
    if (rpm < 0.7) us = us * 2;

    if (us < 0) {
      flagPWM = true;


    } else {
      flagPWM = false;

    }
  }


}




/*
  void control1DOF(){

  // SIRVE
  //float Kp = 80.746325533681762; // MAS FUERTE
  //float Ki = 1.208194038461562; // MAS RAPIDO
  //if (rpm == 0 && elapsedTime > 5000) float Ki = 7.208194038461562; // MAS RAPIDO
  //float Kd = -2.640519572143324;
  //float N = 1;
  //float kaw=1;

  float Kp = 50.828941195007669; // MAS FUERTE
  if (rpm == 0) float Kp = 120.828941195007669; // MAS FUERTE
  float Ki = 2.061532401048105; // MAS RAPIDO
  if (rpm == 0) float Ki = 79.208194038461562; // MAS RAPIDO
  float Kd = 2.802268846371352;
  float N = 9.358000000000000e-14;
  float kaw=0.1;


  en = (Sp*0.0174533) - (P*0.0174533);
  if (en < 0 ) enn = en*-1;


  float Ap = Kp*en;
  float Ad = Kd*(en-en_1) + N*Ad_1;
  float I_error = en + I_error_1 + kaw*difact;
  float Ai = Ki*I_error;
  un = Ap + Ai + Ad;
  float us = un;

  if(un < -100) {
  us = -100;
  }
  if(un > 100) {
  us = 100;
  }


  en_1 = en;
  I_error_1 = I_error;
  difact = us - un;
  Ad_1 = Ad;

  if(en > errPer || enn > errPer ) {

  if(us < 0){
  us = us*-1;
  if(rpm == 0 && elapsedTime == 4500) us = us*2;
  arduinoPWM = us;
  flagPWM = true;


  } else{
  if(rpm == 0 && elapsedTime == 4500) us = us*2;
  arduinoPWM = us;
  flagPWM = false;

  }
  }


  }

*/



/*
  void control2DOF() {
  double Kp = -3.357670445385118;
  double Ki = 3.922951239710286;
  double Kd = 92.464560395668343;
  double N = 0.782109908780621;
  double Kpd = 0.285776886757601;
  double Kdp = 9.408050560944484;
  double N1 = 0.997168715666942;
  double Ka = 0.1;

  en = (Sp * 0.0174533) - (P * 0.0174533);
  if (en < 0 ) enn = en * -1;
  Ap = Kp * en;
  Ad = Kd * (en - en_1) + N * Ad1_;
  I_error = en + I_error_1 + Ka * difact;
  Ai = Ki * I_error;
  U1 = Ap + Ai + Ad;

  Ap1 = Kpd * (P * 0.0174533);
  Ad1 = Kdp * ((P * 0.0174533) - yn_1) + N1 * Ad_1;
  U2 = Ap1 + Ad1;

  Uc = U1 - U2;
  us = Uc;

  if (Uc < -100) {
    us = -100;
  }
  if (Uc > 100) {
    us = 100;
  }

  en_1 = en;
  yn_1 = P * 0.0174533;
  I_error_1 = I_error;
  difact = us - Uc;
  Ad_1 = Ad1;
  Ad1_ = Ad;


  if (en > errPer || enn > errPer ) {

    if (us < 0) {
      us = us * -1;
      arduinoPWM = us;
      flagPWM = true;


    } else {
      arduinoPWM = us;
      flagPWM = false;

    }
  }

  }
*/


/*
  void control1DOF() { // SIN INTEGRAL

  float Kp = 835.0865453598250; // MAS FUERTE
  float Kd = -780.8107963787454;
  if (rpm == 0) Kp = 900.0865453598250; // MAS FUERTE


  float N = 0.997168715666942;


  en = (Sp * 0.0174533) - (P * 0.0174533);
  if (en < 0 ) enn = en * -1;


  float Ap = Kp * en;
  float Ad = Kd * (en - en_1) + N * Ad_1;
  un = Ap + Ad;
  us = un;
  if (us > -15 && us < 15)  us = us * 2;


  if (un < -100) {
    us = -100;
  }
  if (un > 100) {
    us = 100;
  }



  en_1 = en;
  Ad_1 = Ad;

  if (en > errPer || enn > errPer ) {

    if (us < 0) {
      us = us * -1;

      arduinoPWM = us;
      flagPWM = true;


    } else {

      arduinoPWM = us;
      flagPWM = false;

    }
  }


  }
*/


/*
  void controlCascada() {

  float kaw1 = 20;
  float kaw2 =20;

  en = (Sp * 0.0174533) - (P * 0.0174533);
  u1 = 25.896804588396275*en - 26.906839434709909*en_1 + 7.014080201415831*en_2 + 1.371242422443549*u1_1 - 0.371242422443549*u1_2 +kaw1*difact;

  un1 = u1;

  if (u1 < -100) {
    un1 = -100;
  }
  if (u1 > 100) {
    un1 = 100;
  }

  en2 = un1 - (rpm*0.104719755);
  u2 = 3.269366127619914*en2 - 0.112980779954379*en2_1 + 1.106272350916589*en2_2 + 1.999700044995500*u2_1 - 0.999700044995500*u2_2 + kaw2*difact2;

  un2 = u2;

  if (u2 < 0) {
    un2 = 0;
  }
  if (u2 > 10) {
    un2 = 10;
  }

  difact2 = un2 - u2;
  difact = un1 - u1;

  en_2 = en_1;
  en_1 = en;
  u1_2 = u1_1;
  u1_1 = u1;

  en2_2 = en2_1;
  en2_1 = en2;
  u2_2 = u2_1;
  u2_1 = u2;


  if (en > errPer || enn > errPer ) {

    if (un2 < 0) {
      un2 = un2 * -1;

      arduinoPWM = un2;
      flagPWM = true;


    } else {

      arduinoPWM = un2;
      flagPWM = false;

    }
  }


  }
*/


/*
  void controlEstados() {

  float kaw = 10; // Declaración corregida del arreglo
  float CL[] = {1, 0}; // Declaración de arreglo corregida
  float Ki[] = {13.038404810405284}; // Declaración de arreglo corregida
  float Kes[] = {8.39272964861829, 6.19627625077861}; // Declaración de arreglo corregida
  float X_est[] = {(P * 0.0174533), (rpm * 0.104719755)};

  float Kid = Ki[0] * Tm;

  float Pv = CL[0] * X_est[0] + CL[1] * X_est[1]; // Multiplicación suministrada corregida
  float en = (Sp * 0.0174533) - (P * 0.0174533);
  if (en < 0 ) enn = en * -1;
        if (en > errPer || enn > errPer ) {

  float Vk = en + Vk_1 + kaw * dif_act;
  float un = Kid * Vk - Kes[0] * X_est[0] - Kes[1] * X_est[1]; // Corregida la multiplicación de Kes

  //    float Pv = (CL[0]*(P* 0.0174533)) + CL[1] * (rpm*0.104719755); // Multiplicación suministrada corregida
  //    float en = (Sp * 0.0174533) - (P * 0.0174533);
  //    float Vk = en + Vk_1 + kaw * dif_act;
  //    float un = Kid * Vk - (Kes[0]*(P* 0.0174533)) - (Kes[1]*(rpm*0.104719755)); // Corregida la multiplicación de Kes
  us = un;
 float  ud = un;
  
  if(en <0 && abs(ud) < 25) us = -20; 
    if(en >0 && abs(ud) < 25) us = 20; 

  if (un > 100) {
    us = 100;
  }
  if (un < -100) {
    us = -100;
  }

    dif_act = us - un;
  Vk_1 = Vk;


    arduinoPWM = us;
        if (rpm < 0.7) us = us * 2.5;

    if (us < 0) {
      flagPWM = true;


    } else {
      flagPWM = false;

    }
  }



  }

  */
