#include "Wire.h"
//Processing incoming serial data
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>
#include <RomniMotor.h>
#include <RomniEncoder.h>

//Messenger object
Messenger Messenger_Handler = Messenger();

//DMP options
//Set true if DMP init was successful
bool dmpReady = false;
//Holds actual interrupt status byte from MPU
uint8_t mpuIntStatus;
//return status after each device operation
uint8_t devStatus;
//Expected DMP paclet size
uint16_t packetSize;
//count of all bytes currently in FIFO
uint16_t fifoCount;
//FIFO storate buffer
uint8_t fifoBuffer[64];

#define OUTPUT_READABLE_QUATERNION

char incoming;
float euler[3];
float ypr[3];

//Current Sensors
const int CURRENTA = A13;
const int CURRENTB = A14;
const int CURRENTC = A15;
const int CURRENT_LIMIT = (1024 / 5) * 2.6;  // amps

// Encoder pins definition
const int encoder1A = 2;
const int encoder1B = 3;
const int encoder2A = 21;
const int encoder2B = 20;
const int encoder3A = 19;
const int encoder3B = 18;

// Encoder Tick counts
volatile long Encoder1_Ticks = 0;
volatile long Encoder2_Ticks = 0;
volatile long Encoder3_Ticks = 0;

// Ultrasonic Pins
const int pingPin1 = 4;
const int pingPin2 = 5;
const int pingPin3 = 6;
const int pingPin4 = 7;
const int pingPin5 = 11;
const int pingPin6 = 12;

// Ultrasonic direction block
boolean isNorthLocked = false;
boolean isNorthWestLocked = false;
boolean isNorthEastLocked = false;
boolean isSouthLocked = false;
boolean isSouthWestLocked = false;
boolean isSouthEastLocked = false;

// Check if the robot can move
boolean moveCheck;

// Infrared pins
const int IR_Pin1 = 0;
const int IR_Pin2 = 1;
const int IR_Pin3 = 2;

// Buzzer pin
const int buzzerPin= 13;

// duration and inch values for ultrasonic data
unsigned int duration, inches;
// timer values for ultrasonic sensors
unsigned long previousMillis1 = 0;
// timer values for infrared sensors
unsigned long previousMillis2 = 0;

const long SENSOR_DELAY = 200;
const long IR_DELAY = 900;

//Motor Pin definition
#define motor1_dir 22
#define motor2_dir 24
#define motor3_dir 26
#define motor1_pwm 8
#define motor2_pwm 9
#define motor3_pwm 10

//Time  update variables
unsigned long LastUpdateMicrosecs = 0;
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;

//Motor speed from PC
//Motor left and right speed
volatile float motor_left_speed = 0;
volatile float motor_right_speed = 0;
volatile float motor_back_speed = 0;

RomniMotor motor1(motor1_pwm, motor1_dir, CURRENTA);
RomniMotor motor2(motor2_pwm, motor2_dir, CURRENTB);
RomniMotor motor3(motor3_pwm, motor3_dir, CURRENTC);

RomniEncoder encoder1(encoder1A, encoder1B);
RomniEncoder encoder2(encoder2A, encoder2B);
RomniEncoder encoder3(encoder3A, encoder3B);

//Setup serial, encoders, ultrasonic, MPU6050 and Reset functions
void setup(){
  Serial.begin(115200);
  SetupEncoders();
  SetupMotors();
  pinMode(IR_Pin1, INPUT);
  pinMode(IR_Pin2, INPUT);
  pinMode(IR_Pin3, INPUT);
  //SetupUltrasonic();
  Messenger_Handler.attach(OnMssageCompleted);
}

// Attach interrupt to motor1's encoder A output
void encoder1A_CHANGE() {
  encoder1.readEncoderA();
  Encoder1_Ticks = encoder1.getPulses();
}

// Attach interrupt to motor1's encoder B output
void encoder1B_CHANGE() {
  encoder1.readEncoderB();
  Encoder1_Ticks = encoder1.getPulses();
}

// Attach interrupt to motor2's encoder A output
void encoder2A_CHANGE() {
  encoder2.readEncoderA();
  Encoder2_Ticks = encoder2.getPulses();
}

// Attach interrupt to motor2's encoder B output
void encoder2B_CHANGE() {
  encoder2.readEncoderB();
  Encoder2_Ticks = encoder2.getPulses();
}

// Attach interrupt to motor3's encoder A output
void encoder3A_CHANGE() {
  encoder3.readEncoderA();
  Encoder3_Ticks = encoder3.getPulses();
}

// Attach interrupt to motor3's encoder B output
void encoder3B_CHANGE() {
  encoder3.readEncoderB();
  Encoder3_Ticks = encoder3.getPulses();
}

// Setup Encoders' pins
void SetupEncoders(){
  // Quadrature encoders
  pinMode(encoder1A, INPUT_PULLUP);      // sets pin A as input
  pinMode(encoder1B, INPUT_PULLUP);      // sets pin B as input
  attachInterrupt(1, encoder1A_CHANGE, CHANGE);
  attachInterrupt(0, encoder1B_CHANGE, CHANGE);

  pinMode(encoder2A, INPUT_PULLUP);      // sets pin A as input
  pinMode(encoder2B, INPUT_PULLUP);      // sets pin B as input
  attachInterrupt(2, encoder2A_CHANGE, CHANGE);
  attachInterrupt(3, encoder2B_CHANGE, CHANGE);

  pinMode(encoder3A, INPUT_PULLUP);      // sets pin A as input
  pinMode(encoder3B, INPUT_PULLUP);      // sets pin B as input
  attachInterrupt(5, encoder3A_CHANGE, CHANGE);
  attachInterrupt(4, encoder3B_CHANGE, CHANGE);
}

//Setup Motors' pins
void SetupMotors(){
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor3_pwm, OUTPUT);
  pinMode(motor3_dir, OUTPUT);
}

// MAIN LOOP
void loop(){
    //moveCheck = true;
    unsigned long currentMillis = millis();
    Read_From_Serial(); //Read from Serial port
    Update_Time();  // Update time
    Update_Encoders();    //Send encoders values through serial port
    
    if(currentMillis - previousMillis1 >= SENSOR_DELAY){
      Update_Ultra_Sonic();  // Read input from ultrasonic sensors
      previousMillis1 = millis();  // Update timer
    }
    
    if(currentMillis - previousMillis2 >= IR_DELAY){
      Update_Infrared(); // Read input from infrared sensors     
      previousMillis2 = millis();  // Update timer
    }
    
    Update_Motors();  //Update motor values with corresponding speed and send speed values through serial port
    Update_Battery();   //Send battery values through serial port
}


// Read from Serial Function
void Read_From_Serial(){
   while(Serial.available() > 0){
       int data = Serial.read();
       Messenger_Handler.process(data);
    }
}


// OnMssg Complete function definition
void OnMssageCompleted(){
  char set_speed[] = "s";
  
  if(Messenger_Handler.checkString(set_speed)){
     //This will set the speed
    Set_Speed();
    return;
  }
}

// Set speed
void Set_Speed(){
  motor_left_speed = Messenger_Handler.readLong();
  motor_right_speed = Messenger_Handler.readLong();
  motor_back_speed = Messenger_Handler.readLong();
}

// Will update all three motors
void Update_Motors(){ 
  if(motor_right_speed > 0 && motor_left_speed > 0){
    if(!isNorthLocked && !isNorthWestLocked && !isNorthEastLocked){
      moveCheck = true;
    }
    else{
      moveCheck = false; 
    }
  }
  else if(motor_right_speed < 0 && motor_left_speed < 0){
    if(!isSouthLocked && !isSouthWestLocked && !isSouthEastLocked){
      moveCheck = true;
    }
    else{
      moveCheck = false; 
    }
  }
  else if ((motor_right_speed > 0 && motor_left_speed < 0 && motor_back_speed < 0) 
            || (motor_right_speed < 0 && motor_left_speed > 0 && motor_back_speed > 0)){
              moveCheck = true;
            }
  if(moveCheck){
    moveRightMotor(motor_right_speed);
    moveLeftMotor(motor_left_speed);
    moveBackMotor(motor_back_speed);  
  }
  else{
   stopMotors(); 
  }
     
}

// Will write all three encoder value through serial port
void Update_Encoders(){
  Serial.print("e");
  Serial.print("\t");
  Serial.print(Encoder1_Ticks);
  Serial.print("\t");
  Serial.print(Encoder2_Ticks);
  Serial.print("\t");
  Serial.print(Encoder3_Ticks);
  Serial.print("\n");
}


// Will read ultrasonic sensors and detects obstacles for all 6 directions.
void Update_Ultra_Sonic(){
    int inch;
    // North West ultrasonic
    inch = Read_Ultrasonic(pingPin1);
    if(inch < 6 && inch > 0)    
      lockOrUnlockDirection(&isNorthWestLocked, true);     
    else
      lockOrUnlockDirection(&isNorthWestLocked, false);    
    
    // North ultrasonic
    inch = Read_Ultrasonic(pingPin2);
    if(inch < 6 && inch > 0)  
      lockOrUnlockDirection(&isNorthLocked, true);      
    else
      lockOrUnlockDirection(&isNorthLocked, false);
    
    // North East ultrasonic
    inch = Read_Ultrasonic(pingPin3);
    if(inch < 6 && inch > 0) 
      lockOrUnlockDirection(&isNorthEastLocked, true);      
    else
      lockOrUnlockDirection(&isNorthEastLocked, false);
      
    // South East ultrasonic
    inch = Read_Ultrasonic(pingPin4);
    if(inch < 6 && inch > 0)   
      lockOrUnlockDirection(&isSouthEastLocked, true);      
    else
      lockOrUnlockDirection(&isSouthEastLocked, false);
    
    // South ultrasonic
    inch = Read_Ultrasonic(pingPin5);
    if(inch < 6 && inch > 0)  
      lockOrUnlockDirection(&isSouthLocked, true);      
    else
      lockOrUnlockDirection(&isSouthLocked, false);
    
    // South West ultrasonic
    inch = Read_Ultrasonic(pingPin6);
    if(inch < 6 && inch > 0)  
      lockOrUnlockDirection(&isSouthWestLocked, true);      
    else
      lockOrUnlockDirection(&isSouthWestLocked, false);
}

// Read Ultrasonic sensors
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

// Will read infrared sensors and detects stairs for all 3 directions.
void Update_Infrared(){
  int distance;
  
  distance = readInfrared(IR_Pin1);
  if(distance > 75)  
      lockOrUnlockDirection(&isNorthLocked, true);      
    else
      lockOrUnlockDirection(&isNorthLocked, false);
      
  distance = readInfrared(IR_Pin2);
  if(distance > 75)  
      lockOrUnlockDirection(&isSouthEastLocked, true);      
    else
      lockOrUnlockDirection(&isSouthEastLocked, false);
  
  distance = readInfrared(IR_Pin3);
  if(distance > 75)  
      lockOrUnlockDirection(&isSouthWestLocked, true);      
    else
      lockOrUnlockDirection(&isSouthWestLocked, false);
} 

// Read Infrared sensors
int readInfrared(int pin){
  int val = analogRead(pin);
  int distance = map(val, 0, 550, 150, 20);
  return distance;
}

//Update time function
void Update_Time(){
  CurrentMicrosecs = micros();
  LastUpdateMillisecs = millis();
  MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
  if (MicrosecsSinceLastUpdate < 0){
        MicrosecsSinceLastUpdate = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;
    }
  LastUpdateMicrosecs = CurrentMicrosecs;
  SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;
}

//Conversion of microsecond to centimeter
long microsecondsToCentimeters(long microseconds){
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29 / 2;
}

// Update battery function
void Update_Battery(){
 /*battery_level = analogRead(PC_4);

 Serial.print("b");
 Serial.print("\t");
 Serial.print(battery_level);
 Serial.print("\n");
*/
}

// Motor1 running function
void moveLeftMotor(float leftWheelSpeed){  
  if (leftWheelSpeed > 0 || leftWheelSpeed == 0)  {
    motor2.setMotorDirection(1);
    motor2.setSpeed(leftWheelSpeed);
  }
  else {
    motor2.setMotorDirection(0);
    motor2.setSpeed(abs(leftWheelSpeed));
  }
}

// Motor2 running function
void moveRightMotor(float rightWheelSpeed){
  if (rightWheelSpeed > 0 || rightWheelSpeed == 0)  {
    motor1.setMotorDirection(1);
    motor1.setSpeed(rightWheelSpeed);
  }
  else {
    motor1.setMotorDirection(0);
    motor1.setSpeed(abs(rightWheelSpeed));
  }
}

// Motor3 running function
void moveBackMotor(float backWheelSpeed){
  if (backWheelSpeed > 0 || backWheelSpeed == 0)  {
    motor3.setMotorDirection(1);
    motor3.setSpeed(backWheelSpeed);
  }
  else {
    motor3.setMotorDirection(0);
    motor3.setSpeed(abs(backWheelSpeed));
  }
}

// Function to stop all three motors
void stopMotors(){
 motor1.setMotorDirection(!motor1.getMotorDirection());
 motor1.setSpeed(255);
 motor2.setMotorDirection(!motor2.getMotorDirection());
 motor2.setSpeed(255);
 motor3.setMotorDirection(!motor3.getMotorDirection());
 motor3.setSpeed(255);
 motor1.setSpeed(0);
 motor2.setSpeed(0);
 motor3.setSpeed(0);
 /*moveLeftMotor(0);
 moveRightMotor(0);
 moveBackMotor(0); */
}

// Function to set obstacle or stairs in the given direction
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
