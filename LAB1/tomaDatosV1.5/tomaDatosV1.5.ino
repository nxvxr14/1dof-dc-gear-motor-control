
const byte Encoder_C1 = 2; // Cable amarillo pin 3 digital
const byte Encoder_C2 = 3; // Cable verde al pin 4 digital
byte Encoder_C1Last;
int paso = 0;
boolean direccion, flagPWM, autoincrement, invFlag;

int entrada3 = 13;
int entrada4 = 12;
int enableB = 11;

int maxPWM = 130; // VCC 7V

int userE, arduinoPWM;

const int tMuestro = 30;
unsigned long tiempoDatos = 3000;

volatile int  n    = 0;
volatile byte ant  = 0;
volatile byte act  = 0;

double P = 0;
double R = 3840;

float rpm = 0;
unsigned long timeold = 0;
unsigned long timeoldPWM = 0;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;


void setup()
{
  Serial.begin(9600);
  pinMode(entrada3, OUTPUT);
  pinMode(entrada4, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(Encoder_C2, INPUT);
  flagPWM = false;
  autoincrement = false;
  invFlag = false;

  attachInterrupt(digitalPinToInterrupt(Encoder_C1), calculapulso, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_C2), calculapulso, CHANGE);
}

void loop()
{
  // digitalWrite(entrada3, HIGH);
  // digitalWrite(entrada4, LOW);
  //
  //        analogWrite(enableB, map(25, 0, 100, 0, maxPWM)); // 37.5
  //        analogWrite(enableB, map(50, 0, 100, 0, maxPWM)); // 129



  if (Serial.available()) {       // Verificar si hay datos disponibles en el puerto serial
    userE = Serial.parseInt();  // Leer el valor entero enviado por el puerto serial
    // Verificar si el valor recibido está dentro del rango válido (0-100)
    if (userE > 5 && userE != 101 ) autoincrement = true;

    if (userE == 101) {
      autoincrement = false;
      startTime = 0; // Reiniciar el tiempo de inicio si autoincrement es false
      elapsedTime = 0;
      timeold = 0;
      timeoldPWM = 0;
      tiempoDatos = 3000;
      rpm = 0;
      P = 0;
      analogWrite(enableB, 0); // Equivalente a 50% del máximo (128 de 255)

    }

  }

  if (autoincrement == true) {
    if (startTime == 0) {
      startTime = millis(); // Registrar el tiempo de inicio
    }

    elapsedTime = millis() - startTime; // Calcular el tiempo transcurrido


    if (elapsedTime > 3000) {

      // PWM FIJO
      //            if (elapsedTime == 3001) arduinoPWM = 10; // PWMX60x80

      if (elapsedTime > 3001) {
        arduinoPWM = 60; // PWMX60x80
      }

      if (elapsedTime > 13000) {
        arduinoPWM = 0; // PWMX60x80
      }
      if (elapsedTime > 23000) {
        if (invFlag == false) {
          flagPWM = !flagPWM;
          invFlag = true;
        }
        arduinoPWM = 60; // PWMX60x80
      }

      if (elapsedTime > 33000) {
        arduinoPWM = 0; // PWMX60x80
      }



      if (millis() - timeoldPWM >= 700) {
        // Cambia la dirección del motor y establece la velocidad
        if (flagPWM) {
          digitalWrite(entrada3, LOW);
          digitalWrite(entrada4, HIGH);

        } else {

          digitalWrite(entrada3, HIGH);
          digitalWrite(entrada4, LOW);
        }
        analogWrite(enableB, map(arduinoPWM, 0, 100, 0, maxPWM));

        // Invierte el estado de flagPWM para la próxima iteración
        //        flagPWM = !flagPWM;

        // Actualiza el tiempo anterior
        timeoldPWM = millis();
      }


      if (millis() - timeold >= tMuestro) {
        rpm = (paso * (0.100 / (millis() - timeold)) * (60000 / 360)) * -1 ;
        P = (n * 360.0) / R;
        impresionSerial();
        if (!flagPWM) {
          Serial.print(arduinoPWM); Serial.print("\n");
        } else {
          Serial.print("-"); Serial.print(arduinoPWM); Serial.print("\n");

        }
        timeold = millis();
        paso = 0;
      }

      if (elapsedTime >= 43000 ) {
        autoincrement = false;
        startTime = 0; // Reiniciar el tiempo de inicio si autoincrement es false
        elapsedTime = 0;
        timeold = 0;
        timeoldPWM = 0;
        tiempoDatos = 3000;
        rpm = 0;
        P = 0;
        analogWrite(enableB, 0); // Equivalente a 50% del máximo (128 de 255)
      }
    }



  }
}

void impresionSerial() {
  Serial.print(elapsedTime ); Serial.print(", "); Serial.print(rpm ); Serial.print(", "); Serial.print(P ); Serial.print(", ");

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
