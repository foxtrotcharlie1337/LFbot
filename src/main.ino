#include <PID_v1.h>

// PID
double Diff = 0;
double sp = 0;
double Err = 0;


double KP = 25.0;
double KD = 0.0;
double KI = 0.0;
#define MAX_ERROR 3
#define BASE_SPEED 100
#define SPEED_DIFFERENCE 80
#define NUMBER_OF_SENSORS 5

PID pid(&Err, &Diff, &sp, KP, KI, KD, DIRECT);

#define CZYDEBUG true
#define LAST_COUNTER_MAX 30

// functions

void debug();
void debugMotors();
void debugSensors();
void debugPID();
void readSensors();
void calcSpeeds();
void setAEngine();
void setBEngine();
void setError();

// Definiujemy piny służące do odczytu linii
#define OUT1 8
#define OUT2 9
#define OUT3 10
#define OUT4 11
#define OUT5 12


int Sensors[5] = {OUT1, OUT2, OUT3, OUT4, OUT5};

// Deklarujemy zmienne globalne
int SReading[5] = {0}; // odczyt sensoru 0

// Deklarujemy wagi czujników

int Weight[5] = {-2, -1, 0, 1, 2};

// Silnik I - prawy

int pwmA = 5;
int in1A = 3;
int in2A = 4;

// Silnik II - lewy

int pwmB = 6;
int in1B = 7;
int in2B = 2;

// Motor Speed Values - Start at zero

int SpeedA = 0;
int SpeedB = 0;
int last = 0;


float numberOfDetectedSensors;
// Bufor do debugowania
char buff[50] = {0};

void setup() {
  Serial.begin(9600);
  
  Serial.println("Setting up sensors pins");
  pinMode(OUT1, INPUT); // Ustawienie pinu 8 w
  pinMode(OUT2, INPUT); // tryb wejścia
  pinMode(OUT3, INPUT);
  pinMode(OUT4, INPUT);
  pinMode(OUT5, INPUT);

  Serial.println("Setting up pins for mottor stearing");
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  // Initializacja PID
  Serial.println("Setting UP pid");
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-SPEED_DIFFERENCE, SPEED_DIFFERENCE);
  pid.Compute();
  setEnginesSpeeds();
}

void loop() {
  readSensors();
  setError();
  pid.Compute();
  calcSpeeds();
  setEnginesSpeeds();
//#ifdef CZYDEBUG
  debug();
//#endif
  delay(50);
}
void debug() {
  if (last == LAST_COUNTER_MAX) {
    debugSensors();
    debugMotors();
    debugPID();
    last = 0;
  }
  else {
    last++;
  }
}

void debugSensors() {
  sprintf(buff, 
          "Odczyt: %d%d%d%d%d",
          SReading[0], SReading[1], SReading[2], SReading[3], SReading[4]);
  Serial.println(buff);
}
void debugMotors() {
  sprintf(buff, 
          "MotorA: %d\nMotorB: %d", SpeedA, SpeedB);
  Serial.println(buff);
}

void debugPID() {
    sprintf(buff, 
          "PID: Err: %d.%02d, Diff: %d.%02d, KI:%d.%02d", 
           int(Err), int(Err*100)%100, int(Diff), int(Diff*100)%100, int(sp), int(sp*100)%100);  
    Serial.println(buff);
}

void readSensors() {
  for (int i = 0; i < 5; i++) {
    SReading[i] = !digitalRead(Sensors[i]);
  }
}


void calcSpeeds() {
  int difference = map(Diff, -SPEED_DIFFERENCE, SPEED_DIFFERENCE, -SPEED_DIFFERENCE, SPEED_DIFFERENCE);
  SpeedA = BASE_SPEED + difference;
  SpeedB = BASE_SPEED - difference;
}

void setEnginesSpeeds() {
    // Set Motor A forward

  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);

 // Set Motor B forward

  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  setAEngine();
  setBEngine();
}

void setAEngine() {
  analogWrite(pwmA, SpeedA);
}

void setBEngine() {
  analogWrite(pwmB, SpeedB);
}

void setError() {
  numberOfDetectedSensors = 0;

  for (int i = 0; i < NUMBER_OF_SENSORS; i++)
  {
    Err += SReading[i] * Weight[i];
    numberOfDetectedSensors += SReading[i];       
  }
  if (numberOfDetectedSensors == 0 || numberOfDetectedSensors == NUMBER_OF_SENSORS) {
    Err = 0;
  }
  else {
    Err /= numberOfDetectedSensors;
  }
}