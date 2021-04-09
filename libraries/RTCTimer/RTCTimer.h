#ifndef _RTC_TIMER_H_
#define _RTC_TIMER_H_


#include <Arduino.h>


#define RTC_TIMER_CLK	40000 // LSI clock is 40 KHz


class RTCTimerClass
{
	public:
		void begin(uint16_t times, void(*)(void), bool clockRenew = false); // Min is 1ms, max is 8s
		void setNewPeriod(uint16_t times);
		void end(void);
		void (*rtcCallback)(void);
		bool clockRenewCmd;
		
	private:
	
};


extern RTCTimerClass RTCTimer;


#endif