#ifndef _WATCH_DOG_H_
#define _WATCH_DOG_H_


#include <Arduino.h>


// Max timeout is 43.7 ms
class WWDGClass
{
	public:
		
		void begin(void);
		void reset(void);
		void end(void);
		
	private:
	
};


extern WWDGClass wwdg;


#endif