#ifndef _LOW_POWER_H_
#define _LOW_POWER_H_


#include <Arduino.h>


typedef void (*voidFuncPtr)( void ) ;


class LowPower
{
	public:
		void begin(unsigned int inter_pin, voidFuncPtr callback, unsigned int pin_mode = FALLING);
		void standby(bool flag = false);

	private:

};

#endif
