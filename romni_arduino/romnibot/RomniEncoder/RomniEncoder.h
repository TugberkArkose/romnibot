#ifndef RomniEncoder_h  
#define RomniEncoder_h
#include "Arduino.h" 

class RomniEncoder  
{  
public:  
    RomniEncoder(int encA, int encB);
	void begin();
	void readEncoderA();
	void readEncoderB();
	void resetEncoder();
	int getPulses();
	
private:  
    int _encA;  
    int _encB;  
    int _pulses;
};
#endif