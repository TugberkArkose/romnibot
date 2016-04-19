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

//Encoder pins definition
const int encoder1A = 2;
const int encoder1B = 3;
const int encoder2A = 21;
const int encoder2B = 20;
const int encoder3A = 19;
const int encoder3B = 18;
volatile long Encoder1_Ticks = 0;
volatile long Encoder2_Ticks = 0;
volatile long Encoder3_Ticks = 0;

/*const int Left_Encoder_PinA = 2;
const int Left_Encoder_PinB = 3;

volatile long Left_Encoder_Ticks = 0;
volatile bool LeftEncoderBSet;

//Right Encoder

const int Right_Encoder_PinA = 21;
const int Right_Encoder_PinB = 20;
volatile long Right_Encoder_Ticks = 0;
volatile bool RightEncoderBSet;*/

//Motor Pin definition
#define motor1_dir 22
#define motor2_dir 24
#define motor3_dir 26
#define motor1_pwm 8
#define motor2_pwm 9
#define motor3_pwm 10

//Ultrasonic pins definition
const int echo = 9, Trig = 10;
long duration, cm;

//Battery level monitor for future upgrade
#define BATTERY_SENSE_PIN 33
float battery_level = 12;

//Reset pin for resetting Tiva C, if this PIN set high, Tiva C will reset
#define RESET_PIN PB_2

//Time  update variables
unsigned long LastUpdateMicrosecs = 0;
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;

//Motor speed from PC
//Motor left and right speed
float motor_left_speed = 0;
float motor_right_speed = 0;
float motor_back_speed = 0;

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
  SetupUltrasonic();
  SetupReset();
  Messenger_Handler.attach(OnMssageCompleted);
}

  void encoder1A_CHANGE() {
    encoder1.readEncoderA();
    Encoder1_Ticks = encoder1.getPulses();
  }

  void encoder1B_CHANGE() {
    encoder1.readEncoderB();
    Encoder1_Ticks = encoder1.getPulses();
  }

  void encoder2A_CHANGE() {
    encoder2.readEncoderA();
    Encoder2_Ticks = encoder2.getPulses();
  }

  void encoder2B_CHANGE() {
    encoder2.readEncoderB();
    Encoder2_Ticks = encoder2.getPulses();
  }

  void encoder3A_CHANGE() {
    encoder3.readEncoderA();
    Encoder3_Ticks = encoder3.getPulses();
  }

  void encoder3B_CHANGE() {
    encoder3.readEncoderB();
    Encoder3_Ticks = encoder3.getPulses();
  }

/*void do_Left_Encoder(){
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  Left_Encoder_Ticks -= LeftEncoderBSet ? -1 : +1;
}

void do_Right_Encoder(){
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
}*/

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
  attachInterrupt(4, encoder3A_CHANGE, CHANGE);
  attachInterrupt(5, encoder3B_CHANGE, CHANGE);

  // Quadrature encoders
  // Left encoder
  /*pinMode(Left_Encoder_PinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(Left_Encoder_PinB, INPUT_PULLUP);      // sets pin B as input
  //Attaching interrupt in Left_Enc_PinA.
  attachInterrupt(0, do_Left_Encoder, RISING);*/


  // Right encoder
  /*pinMode(Right_Encoder_PinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(Right_Encoder_PinB, INPUT_PULLUP);      // sets pin B as input
  //Attaching interrupt in Right_Enc_PinA.
  attachInterrupt(2, do_Right_Encoder, RISING); */
}

//Setup Motors() function
void SetupMotors(){
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
}


//Setup UltrasonicsSensor() function
void SetupUltrasonic(){
 pinMode(Trig, OUTPUT);
 pinMode(echo, INPUT);
}

//Setup Reset() function

void SetupReset(){
  //pinMode(RESET_PIN,OUTPUT);
  ///Conenect RESET Pins to the RESET pin of launchpad,its the 16th PIN
  //digitalWrite(RESET_PIN,HIGH);
}

//MAIN LOOP
void loop(){
    Read_From_Serial(); //Read from Serial port
    Update_Time();  //Send time information through serial port
    Update_Encoders();    //Send encoders values through serial port
    Update_Ultra_Sonic();      //Send ultrasonic values through serial port
    Update_Motors();  //Update motor values with corresponding speed and send speed values through serial port
    Update_Battery();   //Send battery values through serial port
}


//Read from Serial Function
void Read_From_Serial(){
   while(Serial.available() > 0)
    {
       int data = Serial.read();
       Messenger_Handler.process(data);
    }
}


//OnMssg Complete function definition
void OnMssageCompleted(){
  char reset[] = "r";
  char set_speed[] = "s";

  if(Messenger_Handler.checkString(reset)){
     Serial.println("Reset Done");
     Reset();
  }
  if(Messenger_Handler.checkString(set_speed)){
     //This will set the speed
    Set_Speed();
     return;
  }
}


//Set speed
void Set_Speed()
{
  motor_left_speed = Messenger_Handler.readLong();
  motor_right_speed = Messenger_Handler.readLong();
}

void moveNorth(){
    motor1.setMotorDirection(1);
    motor1.setSpeed(30);
    motor2.setMotorDirection(1);
    motor2.setSpeed(30);
    motor3.setSpeed(0);
}

//Reset function
void Reset(){
  /*digitalWrite(GREEN_LED,HIGH);
  delay(1000);
  digitalWrite(RESET_PIN,LOW);
  digitalWrite(GREEN_LED,LOW); */
}

//Will update both motors
void Update_Motors(){
  moveRightMotor(motor_right_speed);
  moveLeftMotor(motor_left_speed);

  Serial.print("s");
  Serial.print("\t");
  Serial.print(motor_left_speed);
  Serial.print("\t");
  Serial.print(motor_right_speed);
  Serial.print("\n");
}

//Will update both encoder value through serial port
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


//Will update ultrasonic sensors through serial port
void Update_Ultra_Sonic(){
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(echo, HIGH);
  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);

  //Sending through serial port
  Serial.print("u");
  Serial.print("\t");
  Serial.print(cm);
  Serial.print("\n");
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

  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");
}

//Conversion of microsecond to centimeter
long microsecondsToCentimeters(long microseconds){
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29 / 2;
}

//Update battery function
void Update_Battery(){
 /*battery_level = analogRead(PC_4);

 Serial.print("b");
 Serial.print("\t");
 Serial.print(battery_level);
 Serial.print("\n");
*/
}

//Motor running function
void moveRightMotor(float rightServoValue)
{
  if (rightServoValue>0)
  {

    digitalWrite(motor1_dir,HIGH);
    analogWrite(motor1_pwm,rightServoValue);

  }
  else if(rightServoValue<0)
  {
    digitalWrite(motor1_dir,LOW);
    analogWrite(motor1_pwm,abs(rightServoValue));

  }

  else if(rightServoValue == 0)
  {
    digitalWrite(motor1_dir,HIGH);
  }
}


void moveLeftMotor(float leftServoValue)
{
 if (leftServoValue > 0)
  {
    digitalWrite(motor2_dir,HIGH);
    analogWrite(motor2_pwm,leftServoValue);
  }
  else if(leftServoValue < 0)
  {
    digitalWrite(motor2_dir,LOW);
    analogWrite(motor2_pwm,abs(leftServoValue));

  }
  else if(leftServoValue == 0)
  {
    digitalWrite(motor2_dir,HIGH);
   }

}
