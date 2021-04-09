
#include <RTCTimer.h>
#include <LowPower.h>

LowPower nrgSave;

void setup()
{
	nrgSave.begin(0, NULL);
	RTCTimer.begin(5000, NULL);
	pinMode(5, OUTPUT);
	digitalWrite(5, HIGH);
}

void loop()
{	
	nrgSave.standby();
	
	for(uint8_t i = 0; i < 4; i ++)
	{
		digitalWrite(5, LOW);
		digitalWrite(5, HIGH);
	}
	
	delay(1000);
}