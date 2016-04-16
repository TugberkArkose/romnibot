#include <RomniMotor.h>

//Serial input
// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

//Character arrays to hold the first argument
char argv1[16];

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index_ = 0;

// The arguments converted to integers
long arg1;

bool boolMove = false;

unsigned time = millis();

//Motors
const int pwm_a = 7;  //PWM for CH4 RR  - Revserved Encoder pins to make postive
const int pwm_b = 9;  //PWM for CH2 LR
const int pwm_c = 10;  //PWM for CH3 RF
const int dir_a = 43;  //DIR for CH4
const int dir_b = 42;  //DIR for CH2
const int dir_c = 44;  //DIR for CH3

//Current Sensors
const int CURRENTA = A13; 
const int CURRENTB = A14; 
const int CURRENTC = A15; 
const int CURRENT_LIMIT = (1024 / 5) * 2.6;  // amps

//Encoder Interrupts
//Needs Teensy Encoder library
const int encA1 = 2;
const int encA2 = 46;
const int encB1 = 3;
const int encB2 = 47;
const int encC1 = 18;
const int encC2 = 48;

RomniMotor motor1(pwm_a, dir_a, CURRENTA); 
RomniMotor motor2(pwm_b, dir_b, CURRENTB); 
RomniMotor motor3(pwm_c, dir_c, CURRENTC); 

void mReverse(int speed)
{
	motor1.setMotorDirection(1);
	motor2.setMotorDirection(0);
	motor3.setMotorDirection(1);
	motor1.setSpeed(speed);
        motor2.setSpeed(speed);
        motor3.setSpeed(speed);
        boolMove = true;
}

void mForward(int speed)
{
	motor1.setMotorDirection(0);
	motor2.setMotorDirection(1);
	motor3.setMotorDirection(0);
	motor1.setSpeed(speed);
        motor2.setSpeed(speed);
        motor3.setSpeed(speed);
        boolMove = true;
}

void mRight(int speed)
{
	motor1.setMotorDirection(1);
	motor2.setMotorDirection(0);
	motor3.setMotorDirection(0);
	motor1.setSpeed(speed);
        motor2.setSpeed(speed);
        motor3.setSpeed(speed);
        boolMove = true;        
}

void mLeft(int speed)
{
	motor1.setMotorDirection(0);
	motor2.setMotorDirection(1);
	motor3.setMotorDirection(1);
	motor1.setSpeed(speed);
        motor2.setSpeed(speed);
        motor3.setSpeed(speed);
        boolMove = true;
}

void mStop()
{
	motor1.stopMotors();
	motor2.stopMotors();
	motor3.stopMotors();
        boolMove = false;
}

void runCommand(){
   arg1 = atoi(argv1);   
    switch (cmd){
    
      case 'w':
            Serial.print("Forward: ");  
            Serial.println(arg1);   
            mForward(arg1);
      break;
      
      case 'a':
          Serial.print("Left: "); 
          Serial.println(arg1);  
          mLeft(arg1); 
      break;
      
      case 'd':
         Serial.print("Right: ");
         Serial.println(arg1);
         mRight(arg1);     
      break;
      
      case 's':
        Serial.print("Reverse: "); 
        Serial.println(arg1); 
        mReverse(arg1);      
      break;
      
      case 'z':
        mStop();
        Serial.println("Stopped");
      break;
  }
  
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  arg = 0;
  arg1 = 0;
  index_ = 0;
}

void setup(){
  Serial.begin(115200);
  pinMode(pwm_a, OUTPUT);
  pinMode(dir_a, OUTPUT);
}

void loop(){   
  while (Serial.available() > 0) {    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index_] = NULL;
      runCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index_] = NULL;
        arg = 2;
        index_ = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index_] = chr;
        index_++;
      }
      
    }
  } 
 
  if (boolMove){  
    if(millis() > (time+1000)){    
      Serial.print("M1-Current: ");
      Serial.println(motor1.getCurrent());      
      Serial.print("M2-Current: ");
      Serial.println(motor2.getCurrent());      
      Serial.print("M3-Current: ");
      Serial.println(motor3.getCurrent());      
      time=millis();    
    }  
  } 

}


