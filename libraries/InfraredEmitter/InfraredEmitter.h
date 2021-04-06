
#ifndef _Infrared_Emitter_H_
#define _Infrared_Emitter_H_


#include <Arduino.h>


#define __DEBUG    0


class InfraredEmitter
{   
public:
	
	InfraredEmitter(int senPin);
	void Init(void);
	void InfraredEmitterWrite(unsigned char *idata, unsigned char num);
    void InfraredEmitterWriteCodeA(int code);
    void InfraredEmitterWriteCodeB(int code);
    
	void sendRaw(unsigned int buf[], int len);
    void mark(int usec);
    void space(int usec);
	void enableIROut(void);
private:

    
	
	int _pin;
};

#endif