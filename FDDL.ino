#include <QTRSensors.h>

int D1_M1 = 3;  //M2-dreapta     //D1-fata
int D2_M1 = 5;  //M1- stanga   //D2-spate
int D1_M2 = 6;
int D2_M2 = 9;

#define basePWM 255
#define turnPWM 200

float KP = 0.4;
float KD = 0.00001;   // pid ok : 0.4/0.00001/0.000007
float KI = 0.000005;  //0.0000007
long int I;
float Pvalue;
float Ivalue;
float Dvalue;

enum Stare {
  Urmareste_Linie = 1,      //ma asigur ca prima valoare este 1 
  INTERSECTIE,
  Scara_Stg,
  Scara_Drp,
  Perete
};
Stare StateId = Urmareste_Linie;

unsigned long timp_intersectie = 0;

#define TRESHOLD 0
#define BOOST 2       // Chesti de viitor

QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];


void setup() {
  // put your setup code here, to run once:
  pinMode(D1_M1, OUTPUT);
  pinMode(D2_M1, OUTPUT);
  pinMode(D1_M2, OUTPUT);
  pinMode(D2_M2, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, SensorCount);
  qtr.setEmitterPin(2);

  delay(250);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
}

float PIDval(int position, int target_position) {       //calculam pid
  static uint16_t lastError = 0;

  int16_t error = position - target_position;
  I += error;

  Pvalue = KP * error;
  Ivalue = KI * I;
  Dvalue = KD * (error - lastError);

  int16_t PID = Pvalue + Ivalue + Dvalue;
  lastError = error;
  return PID;
}
void loop() {

  uint16_t sensors[6];
  int16_t position = qtr.readLineBlack(sensorValues);      
  float pidValue = PIDval(position, 2500);          //aflu pid-ul si imi calculez vitezele
  int m1Speed = basePWM - pidValue;        
  int m2Speed = basePWM + pidValue;

  switch (StateId) {

    case Urmareste_Linie:
      Control_Motoare(m1Speed, m2Speed);
      //Intersectie
      if (
        (sensorValues[0] > 700 && sensorValues[2] > 700 && sensorValues[3] > 700 && sensorValues[5] > 700) || (sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[3] > 700 && sensorValues[4] > 700) || (sensorValues[0] > 700 && sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[3] > 700 && sensorValues[4] > 700 && sensorValues[5] > 700)) {
        StateId = INTERSECTIE;
      }
      StateId = INTERSECTIE;

      //Scara stg
      if (sensorValues[0] < 300 && sensorValues[1] < 600 && sensorValues[3] > 700 && sensorValues[4] > 700 && sensorValues[5] > 700) {
        StateId = Scara_Stg;
      }
      //Scara drp
      if (sensorValues[0] > 700 && sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[4] < 600 && sensorValues[5] < 300) {
        StateId = Scara_Drp;
      }

      break;
    case INTERSECTIE:

      Control_Motoare(0, 0);
      delay(5000);
      Intersectie();

      break;
    case Scara_Stg:

      Control_Motoare(0, 0);
      delay(2000);
      Intoarce_stanga_90();
      break;

    case Scara_Drp:
      Control_Motoare(0, 0);
      delay(2000);
      Intoarce_dreapta();
      break;

    default:
      StateId = Urmareste_Linie;
      break;
  }



  delay(10);
}


void Control_Motoare(int Speed_M1, int Speed_M2) {
  // Limitare între -255 și 255 (inclusiv pentru sens invers)
  Speed_M1 = constrain(Speed_M1, -255, 255);
  Speed_M2 = constrain(Speed_M2, -255, 255);

  // Motorul 1 (stânga)
  if (Speed_M1 < 0) {
    analogWrite(D2_M1, abs(Speed_M1));  // înapoi
    analogWrite(D1_M1, 0);
  } else {
    analogWrite(D2_M1, 0);
    analogWrite(D1_M1, Speed_M1);  // înainte
  }

  // Motorul 2 (dreapta)
  if (Speed_M2 < 0) {
    analogWrite(D2_M2, abs(Speed_M2));  // înapoi
    analogWrite(D1_M2, 0);
  } else {
    analogWrite(D2_M2, 0);
    analogWrite(D1_M2, Speed_M2);  // înainte
  }
}

void Intoarce_dreapta() {      

  qtr.readLineBlack(sensorValues);
  int linie = 1;
  int millis3 = millis();                   //merg in fata cateva ms ca sa ma asigur ca pot sa ma intorc
  if (millis() - millis3 >= 500) {
    Control_Motoare(255, 255);
  }
  if ((sensorValues[0] < 300 && sensorValues[1] < 300 && sensorValues[2] < 300 && sensorValues[3] < 300 && sensorValues[4] < 300 && sensorValues[5] < 300) || (sensorValues[3] < 300 && sensorValues[2] < 300 ))  {   //nu vad nimic inseamna ca linia e sub mine  -> ma intorc
    linie = 0;
  }
  if (linie == 0) {
    Control_Motoare(turnPWM, -turnPWM);
  }
  if (sensorValues[3] > 900 || sensorValues[2] > 900) {      //vad linia ma opresc
    linie = 1;
    StateId = Urmareste_Linie;
  }
  Control_Motoare(255, 255);
}

void Intoarce_stanga_90() {

 qtr.readLineBlack(sensorValues);
  int linie = 1;
  int millis3 = millis();                   //merg in fata cateva ms ca sa ma asigur ca pot sa ma intorc
  if (millis() - millis3 >= 500) {
    Control_Motoare(255, 255);
  }
  if ((sensorValues[0] < 300 && sensorValues[1] < 300 && sensorValues[2] < 300 && sensorValues[3] < 300 && sensorValues[4] < 300 && sensorValues[5] < 300) || (sensorValues[3] < 300 && sensorValues[2] < 300 ))  {   //nu vad nimic inseamna ca linia e sub mine  -> ma intorc
    linie = 0;
  }
  if (linie == 0) {
    Control_Motoare(-turnPWM, turnPWM);
  }
  if (sensorValues[3] > 900 || sensorValues[2] > 900) {      //vad linia ma opresc
    linie = 1;
    StateId = Urmareste_Linie;
  }
  Control_Motoare(255, 255);
}

void Intersectie() {
  // intersectie EU

  Control_Motoare(255, 255);
  if (timp_intersectie == 0) { timp_intersectie = millis(); }
  if (millis() - timp_intersectie >= 5000) {
    StateId = Urmareste_Linie;                             // daca  nu sunt in caz de giratoriu merg unpic  astept  si ma intorc la celalat mod
    timp_intersectie = 0;
  }
  if (sensorValues[0] < 400 && sensorValues[2] < 400 && sensorValues[3] < 400 && sensorValues[5] < 400) {
    Intoarce_dreapta();
  }

  //Motor cu PID
  uint16_t sensors[6];
  int16_t position = qtr.readLineBlack(sensorValues);
  float pidValue = PIDval(position, 2500);
  int m1Speed = basePWM - pidValue;
  int m2Speed = basePWM + pidValue;

  Control_Motoare(m1Speed, m2Speed);
  //

  if (sensorValues[0] > 700 && sensorValues[2] > 700 && sensorValues[3] > 700) {
    Control_Motoare(0, 0);
    int millis1 = millis();
    if (millis() - millis1 >= 500) {
      Control_Motoare(175, -175);  //sunt la iesire din giratoriu , intorc ca sa fiu cu senzorul pe alb  ca sa ma intorc corect
    }
    Intoarce_dreapta();
    StateId = Urmareste_Linie;
  }
  
}
