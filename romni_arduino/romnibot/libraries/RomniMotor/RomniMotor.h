#ifndef RomniMotor_h  
#define RomniMotor_h
#include "Arduino.h" 

class RomniMotor  
{  
public:  
    RomniMotor(int pwmPin, int dirPin, int currPin); 
	void begin();
    void stopMotor();  
    void setSpeed(int speedMotor);  
    void setMotorDirection(bool isMotor);  
	int getCurrent();
	int getSpeed();
	
private:  
    int _pwm;  
    int _dir;  
    int _curr;
    int _currRate;
    int _speed;
};
#endif
