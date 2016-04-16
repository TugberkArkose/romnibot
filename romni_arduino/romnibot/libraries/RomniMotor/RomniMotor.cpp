#include "Arduino.h"  
#include "RomniMotor.h"

RomniMotor::RomniMotor(int pwmPin, int dirPin, int currPin)
{  
    _pwm = pwmPin;  
    _dir = dirPin;  
    _curr = currPin;
    _currRate = 0;
    _speed = 0;		
}

void RomniMotor::begin()
{
    pinMode(_pwm, OUTPUT);  
    pinMode(_dir, OUTPUT); 
    pinMode(_curr, INPUT);	
}

void RomniMotor::stopMotor()  
{  
    digitalRead(_dir) == HIGH ? digitalWrite(_dir, LOW) : digitalWrite(_dir, HIGH);  
    setSpeed(0);  
}  

void RomniMotor::setSpeed(int speedMotor)  
{  
    speedMotor = constrain(speedMotor, 0, 255);
    analogWrite(_pwm, speedMotor);
    _speed=speedMotor;
}

void RomniMotor::setMotorDirection(bool isMotor)  
{  
    isMotor ?  digitalWrite(_dir, HIGH) : digitalWrite(_dir, LOW);	
}

int RomniMotor::getCurrent()
{
     _currRate = analogRead(_curr);
return _currRate;
}

int RomniMotor::getSpeed()
{
     return _speed;
}