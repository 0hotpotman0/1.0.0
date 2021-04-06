
#include <RTCTimer.h>

uint16_t period = 10;

void setup()
{
	RTCTimer.begin(period, dummy);
	pinMode(5, OUTPUT);
	delay(1000);
}

void loop()
{	
	period += 10;
	if(period > 100)period = 10;
	RTCTimer.setNewPeriod(period);
	delay(1000);
}

void dummy(void)
{
	for(uint8_t i = 0; i < 4; i ++)
	{
		digitalWrite(5, LOW);
		digitalWrite(5, HIGH);
	}
}