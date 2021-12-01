#include <Ultrasonic.h>

Ultrasonic ultrasonic(A3, A2);

#define WEEL_LEFT_FORWARD 3
#define WEEL_LEFT_BACK 9
#define WEEL_RIGHT_FORWARD 10
#define WEEL_RIGHT_BACK 11

#define SIGNAL_PIN 5
#define LIGHT_PIN A0

#define LEFT_STOP_PIN 2
#define RIGHT_STOP_PIN A1

#define SPEED_LEVELS_COUNT 10
#define SPEED_AW_MIN_VAL 80
#define SPEED_AW_MAX_VAL 255
#define SPEED_ACCELERATION_SMOOTH_TIME 1000 // ms
#define SPEED_ACCELERATION_SMOOTH_STEPS 4
#define SPEED_ACCELERATION_SMOOTH_STEP_TIME SPEED_ACCELERATION_SMOOTH_TIME/SPEED_ACCELERATION_SMOOTH_STEPS

#define PATROL_ECHO_STEP_TIME 200 // ms
#define PATROL_ECHO_DISTANCE_MIN 30 // sm
#define PATROL_ECHO_DISTANCE_REDIRECT_OK 50 // sm
#define PATROL_ECHO_DISTANCE_MAX_WRONG 300 // sm
#define PATROL_STOP_ROTATE_TIME 50 // ms

byte valSerial;
char valSerialChar;

bool isSppedSmoothMode = false;
bool isPatrolMode = false;

byte speed = SPEED_LEVELS_COUNT;
byte speedAw = SPEED_AW_MAX_VAL;

float distanceSm = 0;

uint32_t tmrWeels;
byte i=0;
byte speedAwCurrent = 0;
byte speedAwCurrentStep = 1;

uint32_t tmrPatrol = 0;
float sonarDistancePatrol = 0;

void setup()
{
  // Make less noize with motors
  // Pins D9 & D10 - 62.5 kHz
  TCCR1A = 0b00000001;  // 8bit
  TCCR1B = 0b00001001;  // x1 fast pwm
  // Pins D3 & D11 - 62.5 kHz
  TCCR2B = 0b00000001;  // x1
  TCCR2A = 0b00000011;  // fast pwm
  
  Serial.begin(9600);
  
  pinMode(WEEL_RIGHT_BACK, OUTPUT);
  pinMode(WEEL_LEFT_FORWARD, OUTPUT);
  pinMode(WEEL_LEFT_BACK, OUTPUT);
  pinMode(WEEL_RIGHT_FORWARD, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(SIGNAL_PIN, OUTPUT);
  pinMode(LEFT_STOP_PIN, INPUT);
  pinMode(RIGHT_STOP_PIN, INPUT);
  
  digitalWrite(LIGHT_PIN, LOW); 
}

void loop()
{ 
  if (Serial.available())
  {
    valSerial = Serial.read();
    valSerialChar = valSerial;
    Serial.println(String(valSerialChar));

    wheelInit(valSerial);
    lightInit(valSerial);
    signalInit(valSerial);
    speedInit(valSerial);
    sonarInit(valSerial);
    patrolInit(valSerial);
  }
  patrolModeInit();
}

void speedInit(byte val){
    valSerialChar = val;
    if (valSerialChar == '1' || valSerialChar == '2' || valSerialChar == '3' || valSerialChar == '4' || valSerialChar == '5' || valSerialChar == '6' || valSerialChar == '7' || valSerialChar == '8' || valSerialChar == '9')
    {
      speed = valSerialChar - '0';
    }
    else if (valSerialChar == 'q')
    {
      speed = 10;
    }
}

void signalInit(byte val){
    if (val == 'V')
    { 
      tone(SIGNAL_PIN, 1000, 500);
    }
}

void lightInit(byte val){
    if (val == 'w')
    {
      analogWrite(LIGHT_PIN, 130);
    }
    if (val == 'W')
    {
      analogWrite(LIGHT_PIN, 0); 
    }
}

float sonarGetDistance(bool print=true){
  distanceSm = ultrasonic.read();
  if (print) {
    Serial.println(":echo=" + String(distanceSm));
  }
  
  return distanceSm;  
}

void sonarInit(byte val){
    if (val == 'e')
    {
      sonarGetDistance();
    }   
}

void patrolInit(byte val){
    if (val == 'p')
    {
      isPatrolMode = true;
    }
    if (val == 'P')
    {
      isPatrolMode = false; 
      weelsStop();
    }
}

void weelsStop() {
  analogWrite(WEEL_LEFT_FORWARD, 0); 
  analogWrite(WEEL_LEFT_BACK, 0);
  analogWrite(WEEL_RIGHT_FORWARD, 0);
  analogWrite(WEEL_RIGHT_BACK, 0); 

  speedAwCurrentStep = 1;
  tmrWeels = 0;
}

void wheelInit(byte val){
    if (val == 'S')
    {
      weelsStop();
    } 
    if (val == 'L' || val == 'R' || val == 'F' || val == 'B') {
      tmrWeels = millis();
      speedAw = (speed-1)*(SPEED_AW_MAX_VAL - SPEED_AW_MIN_VAL)/(SPEED_LEVELS_COUNT - 1) + SPEED_AW_MIN_VAL;

      if (isSppedSmoothMode) {
        while (true && !Serial.available()) {
          if (speedAwCurrentStep == SPEED_ACCELERATION_SMOOTH_STEPS) {
            tmrWeels = 0;
            speedAwCurrentStep = 1;
            break;
          }
          if (tmrWeels && millis() - tmrWeels >= SPEED_ACCELERATION_SMOOTH_STEP_TIME) {
            tmrWeels = millis();
            speedAwCurrentStep++;
          }
          
          speedAwCurrent = speedAw*speedAwCurrentStep/SPEED_ACCELERATION_SMOOTH_STEPS;    
          Serial.println(speedAwCurrent);
          wheelMove(val, speedAwCurrent);
          
          delay(10);
        }
      } else {
        wheelMove(val, speedAw);
      }
    }    
}

void wheelMoveLeft (byte level) {
  analogWrite(WEEL_LEFT_FORWARD, level); 
  analogWrite(WEEL_LEFT_BACK, 0);
  analogWrite(WEEL_RIGHT_FORWARD, 0);
  analogWrite(WEEL_RIGHT_BACK, level);
}

void wheelMoveForward (byte level) {
  analogWrite(WEEL_LEFT_FORWARD, level); 
  analogWrite(WEEL_LEFT_BACK, 0);
  analogWrite(WEEL_RIGHT_FORWARD, level);
  analogWrite(WEEL_RIGHT_BACK, 0);
}

void wheelMove (byte direction, byte level) {
  if (direction == 'L')
  {
    wheelMoveLeft(level);
  }
  else if (direction == 'R')
  {
    analogWrite(WEEL_LEFT_FORWARD, 0); 
    analogWrite(WEEL_LEFT_BACK, level);
    analogWrite(WEEL_RIGHT_FORWARD, level);
    analogWrite(WEEL_RIGHT_BACK, 0); 
  }
  else if (direction == 'F')
  {
    wheelMoveForward(level);
  }
  else if (direction == 'B')
  {
    analogWrite(WEEL_LEFT_FORWARD, 0); 
    analogWrite(WEEL_LEFT_BACK, level);
    analogWrite(WEEL_RIGHT_FORWARD, 0);
    analogWrite(WEEL_RIGHT_BACK, level);
  }
}

void patrolModeInit() {
  if (isPatrolMode) {
    if (millis() - tmrPatrol >= PATROL_ECHO_STEP_TIME) {
      tmrPatrol = millis();
      sonarDistancePatrol = sonarGetDistance();
      Serial.println(String(digitalRead(RIGHT_STOP_PIN)));
      if (sonarDistancePatrol < PATROL_ECHO_DISTANCE_MIN 
          || digitalRead(LEFT_STOP_PIN) == LOW
          || digitalRead(RIGHT_STOP_PIN) == LOW
      ) {
        weelsStop();
        wheelMoveLeft(SPEED_AW_MIN_VAL);
        delay(PATROL_STOP_ROTATE_TIME);
        weelsStop();
      } else {
        wheelMoveForward(SPEED_AW_MIN_VAL);
      }
    }
  }
}
