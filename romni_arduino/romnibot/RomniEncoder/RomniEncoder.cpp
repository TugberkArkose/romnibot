#include "Arduino.h"  
#include "RomniEncoder.h"

RomniEncoder::RomniEncoder(int encA, int encB)
{  
    _encA = encA;  
    _encB = encB;  
	_pulses = 0;
}

void RomniEncoder::begin()
{
    pinMode(_encA, INPUT);  
    pinMode(_encB, INPUT);	
}

int RomniEncoder::getPulses()
{
	return _pulses;
}

void RomniEncoder::readEncoderA()
{
	if ( digitalRead(_encB) == 0 ) {
		if ( digitalRead(_encA) == 0 ) {
		  // A fell, B is low
		  _pulses--; // moving reverse
		} else {
		  // A rose, B is low
		  _pulses++; // moving forward
		}
	}
	else {
		if ( digitalRead(_encA) == 0 ) {
		  // A fell, B is high
		  _pulses++; // moving forward
		} else {
		  // A rose, B is high
		  _pulses--; // moving reverse
		}
	}
}

void RomniEncoder::readEncoderB()
{
	if ( digitalRead(_encA) == 0 ) {
		if ( digitalRead(_encB) == 0 ) {
		  // B fell, A is low
		  _pulses++; // moving forward
		} else {
		  // B rose, A is low
		  _pulses--; // moving reverse
		}
	} 
	else {
		if ( digitalRead(_encB) == 0 ) {
		  // B fell, A is high
		  _pulses--; // moving reverse
		} else {
		  // B rose, A is high
		  _pulses++; // moving forward
		}
	}
}

void RomniEncoder::resetEncoder(){
	_pulses = 0;
}