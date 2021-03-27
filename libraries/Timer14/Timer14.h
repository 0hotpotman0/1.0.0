
#ifndef _TIMER_14_CPP_
#define _TIMER_14_CPP_


#include <Arduino.h>


#define CPU_HZ 48000000
#define RESOLUTION 0xffff    // Timer 14 is 16 bit


class Timer14Class
{
	
public:

	void init(uint16_t microseconds = 1000);
	void setPeriod(uint16_t microseconds);	
	void start();
	void stop();
	void attachInterrupt(void (*isr)());
	void detachInterrupt();
	void (*isrCallback)();
	
};


extern Timer14Class Timer14;


#endif