#include <LSM303.h>
#include <Wire.h>
#include "Timer.h"
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <RomniMotor.h>

LSM303 compass;
char incoming;
Timer timer;

const int panicBttnPin = 28;
const int buzzerPin = 13;

// INFRARED AND ULTRASONIC PINS
const int IR_Pin1 = 0;
const int IR_Pin2 = 1;
const int IR_Pin3 = 2;

const int pingPin1 = 4;
const int pingPin2 = 5;
const int pingPin3 = 6;
const int pingPin4 = 7;
const int pingPin5 = 11;
const int pingPin6 = 12;

//MOTOR PINS
const int motor1_dir = 22;
const int motor1_pwm = 8;
const int motor2_dir = 24;
const int motor2_pwm = 9;
const int motor3_dir = 26;
const int motor3_pwm = 10;

unsigned int duration, inches;
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
const long COMPASS_DELAY = 3000;
const long SENSOR_DELAY = 200;
const long IR_DELAY = 900;


// Ultrasonic direction block
boolean isNorthLocked = false;
boolean isNorthWestLocked = false;
boolean isNorthEastLocked = false;
boolean isSouthLocked = false;
boolean isSouthWestLocked = false;
boolean isSouthEastLocked = false;

boolean isPanic = false;

  void setup() {  
    pinMode(motor1_dir, OUTPUT);
    pinMode(motor1_pwm, OUTPUT);
    pinMode(motor2_dir, OUTPUT);
    pinMode(motor2_pwm, OUTPUT);
    pinMode(motor3_dir, OUTPUT);
    pinMode(motor3_pwm, OUTPUT);  
    pinMode(buzzerPin,OUTPUT);  
    
    pinMode(IR_Pin1, INPUT);
    pinMode(IR_Pin2, INPUT);
    pinMode(IR_Pin3, INPUT);
    
    pinMode(panicBttnPin, INPUT);  
    
    digitalWrite(motor1_dir, HIGH);
    digitalWrite(motor2_dir, HIGH);
    digitalWrite(motor3_dir, HIGH);
    analogWrite(motor1_pwm, 0);
    analogWrite(motor2_pwm, 0);
    analogWrite(motor3_pwm, 0);
    
    Serial.begin(9600);
    /*Wire.begin();
    compass.init();
    compass.enableDefault();
    compass.m_min = (LSM303::vector<int16_t>) {-32767, -32767, -32767};
    compass.m_max = (LSM303::vector<int16_t>) {+32767, +32767, +32767};
    
    //attachInterrupt(panicBttnPin, panicButtonPressed, FALLING);
    PCintPort::attachInterrupt(panicBttnPin, panicButtonPressed,FALLING);*/
    timer.every(2, readIncoming);
  }

  void loop() {
    if(!isPanic){
      unsigned long currentMillis = millis();
      if(currentMillis - previousMillis1 >= SENSOR_DELAY){
        Update_Ultra_Sonic();
        previousMillis1 = millis();
      }   
      
      /*if(currentMillis - previousMillis2 >= COMPASS_DELAY) {
        Serial.flush();
        compass.read();
        int heading = (int)compass.heading();
        Serial.println(heading);
        Serial.flush();
        previousMillis2 = currentMillis;
      } */  
      
      if(currentMillis - previousMillis3 >= IR_DELAY){
        Update_Infrared();
        previousMillis3 = millis();
      }
    }
    timer.update();
  }

  void readIncoming(){
    //Serial.println(isPanic);
    if(!isPanic){
      if(Serial.available() > 0) {
          incoming = Serial.read();
          Serial.flush();
          if(!isNorthLocked && incoming == 'w'){
            moveNorth();
          }  
          else if(!isSouthWestLocked && !isSouthEastLocked && incoming == 's'){
            moveSouth();
          }
          else if(!isSouthWestLocked && incoming == 'a'){
            moveWest();
          }
          else if(!isSouthEastLocked && incoming == 'd'){
            moveEast();
          }
          else if(!isNorthLocked && incoming == 'q'){
            moveNorthWest();
          }    
          else if(!isNorthLocked && incoming == 'e'){
            moveNorthEast();
          }
          else if(!isSouthWestLocked && incoming == 'z'){
            moveSouthWest();
          }
          else if(!isSouthEastLocked && incoming == 'x'){
            moveSouthEast();
          }
          else if(incoming == 'r'){
            Rotate_Through_Left();
          }
          else if(incoming == 't'){
            Rotate_Through_Right();
          }
          else{
            analogWrite(motor1_pwm, 0);
            analogWrite(motor2_pwm, 0);
            analogWrite(motor3_pwm, 0);
          }
        }  
     }
  }

  void moveNorth(){
    digitalWrite(motor1_dir, HIGH);
    analogWrite(motor1_pwm, 255);
    digitalWrite(motor2_dir, HIGH);
    analogWrite(motor2_pwm, 255);
    analogWrite(motor3_pwm, 0);
  }
  
  void moveSouth(){
    digitalWrite(motor1_dir, LOW);
    analogWrite(motor1_pwm, 255);
    digitalWrite(motor2_dir, LOW);
    analogWrite(motor2_pwm, 255);
    analogWrite(motor3_pwm, 0);
  }
  
  void moveWest(){
    digitalWrite(motor1_dir, HIGH);
    analogWrite(motor1_pwm, 150);
    digitalWrite(motor2_dir, LOW);
    analogWrite(motor2_pwm, 100);
    digitalWrite(motor3_dir, HIGH);
    analogWrite(motor3_pwm, 255);
  }
  
  void moveEast(){
    digitalWrite(motor1_dir, LOW);
    analogWrite(motor1_pwm, 150);
    digitalWrite(motor2_dir, HIGH);
    analogWrite(motor2_pwm, 100);
    digitalWrite(motor3_dir, LOW);
    analogWrite(motor3_pwm, 255);
  }
  
  void moveNorthWest(){
    digitalWrite(motor1_dir, HIGH);
    analogWrite(motor1_pwm, 255);
    digitalWrite(motor2_dir, HIGH);
    analogWrite(motor2_pwm, 60);
    analogWrite(motor3_pwm, 0);
  }
  
  void moveNorthEast(){
    digitalWrite(motor1_dir, HIGH);
    analogWrite(motor1_pwm, 60);
    digitalWrite(motor2_dir, HIGH);
    analogWrite(motor2_pwm, 255);
    analogWrite(motor3_pwm, 0);
  }
  
  void moveSouthWest(){
    digitalWrite(motor1_dir, LOW);
    analogWrite(motor1_pwm, 255);
    digitalWrite(motor2_dir, LOW);
    analogWrite(motor2_pwm, 60);
    analogWrite(motor3_pwm, 0);  
  }
  
  void moveSouthEast(){
    digitalWrite(motor1_dir, LOW);
    analogWrite(motor1_pwm, 60);
    digitalWrite(motor2_dir, LOW);
    analogWrite(motor2_pwm, 255);
    analogWrite(motor3_pwm, 0);
  }
  
  void Rotate_Through_Left(){
    digitalWrite(motor1_dir, HIGH);
    analogWrite(motor1_pwm, 255);
    digitalWrite(motor2_dir, LOW);
    analogWrite(motor2_pwm, 255);
    digitalWrite(motor3_dir, LOW);
    analogWrite(motor3_pwm, 255);
  }
  
  void Rotate_Through_Right(){
    digitalWrite(motor1_dir, LOW);
    analogWrite(motor1_pwm, 255);
    digitalWrite(motor2_dir, HIGH);
    analogWrite(motor2_pwm, 255);
    digitalWrite(motor3_dir, HIGH);
    analogWrite(motor3_pwm, 255);
  }
  
void Update_Ultra_Sonic(){
    int inch;
    // North West ultrasonic
    inch = Read_Ultrasonic(pingPin1);
    if(inch < 8 && inch > 1)    
      lockOrUnlockDirection(&isNorthWestLocked, true);     
    else
      lockOrUnlockDirection(&isNorthWestLocked, false);   
    
    // North ultrasonic
    inch = Read_Ultrasonic(pingPin2);
    if(inch < 8 && inch > 1)  
      lockOrUnlockDirection(&isNorthLocked, true);      
    else
      lockOrUnlockDirection(&isNorthLocked, false);
    
    // North East ultrasonic
    inch = Read_Ultrasonic(pingPin3);
    if(inch < 8 && inch > 1) 
      lockOrUnlockDirection(&isNorthEastLocked, true);      
    else
      lockOrUnlockDirection(&isNorthEastLocked, false);
      
    // South East ultrasonic
    inch = Read_Ultrasonic(pingPin4);
    if(inch < 8 && inch > 1)   
      lockOrUnlockDirection(&isSouthEastLocked, true);      
    else
      lockOrUnlockDirection(&isSouthEastLocked, false);
    
    // South ultrasonic
    inch = Read_Ultrasonic(pingPin5);
    if(inch < 8 && inch > 1)  
      lockOrUnlockDirection(&isSouthLocked, true);      
    else
      lockOrUnlockDirection(&isSouthLocked, false);
    
    // South West ultrasonic
    inch = Read_Ultrasonic(pingPin6);
    if(inch < 8 && inch > 1)  
      lockOrUnlockDirection(&isSouthWestLocked, true);      
    else
      lockOrUnlockDirection(&isSouthWestLocked, false);
}

int Read_Ultrasonic(int ultrasonicPin){
  pinMode(ultrasonicPin, OUTPUT);       // Set pin to OUTPUT         
  digitalWrite(ultrasonicPin, LOW);        // Ensure pin is low
  delayMicroseconds(2);
  digitalWrite(ultrasonicPin, HIGH);       // Start ranging
  delayMicroseconds(5);              //   with 5 microsecond burst
  digitalWrite(ultrasonicPin, LOW);        // End ranging
  pinMode(ultrasonicPin, INPUT);           // Set pin to INPUT
  duration = pulseIn(ultrasonicPin, HIGH, 1000); // Read echo pulse
  inches = duration / 74 / 2;
  return inches;
}

void Update_Infrared(){
  int distance;
  distance = readInfrared(IR_Pin1);
  if(distance > 60)  
      lockOrUnlockDirection(&isNorthLocked, true);      
    else
      lockOrUnlockDirection(&isNorthLocked, false);
      
  distance = readInfrared(IR_Pin2);
  if(distance > 60)  
      lockOrUnlockDirection(&isSouthEastLocked, true);      
    else
      lockOrUnlockDirection(&isSouthEastLocked, false);
  
  distance = readInfrared(IR_Pin3);
  if(distance > 60)  
      lockOrUnlockDirection(&isSouthWestLocked, true);      
    else
      lockOrUnlockDirection(&isSouthWestLocked, false);
} 

int readInfrared(int pin){
  int val = analogRead(pin);
  //int distance = map(val, 0, 550, 150, 20);
  int distance = map(val, 40, 620, 80, 10);
  return distance;
}

void lockOrUnlockDirection(boolean *dir, boolean condition){
  if(condition){
    stopMotors();
    if(!*dir){
      analogWrite(buzzerPin, 20); // beep
    }      
    *dir = true; 
  }
  else{
    if(*dir){
        analogWrite(buzzerPin, 0); // bee
        *dir = false; 
      }    
  }
}

void stopMotors(){
 digitalWrite(motor1_dir, HIGH);
 analogWrite(motor1_pwm, 255);
 digitalWrite(motor2_dir, HIGH);
 analogWrite(motor2_pwm, 255);
 digitalWrite(motor3_dir, LOW);
 analogWrite(motor3_pwm, 255);
 analogWrite(motor1_pwm, 0);
 analogWrite(motor2_pwm, 0);
 analogWrite(motor3_pwm, 0);
}
 
  void panicButtonPressed(){
     isPanic = true;
     /*analogWrite(motor1_pwm, 0);
     analogWrite(motor2_pwm, 0);
     analogWrite(motor3_pwm, 0);*/
     stopMotors();
  }

