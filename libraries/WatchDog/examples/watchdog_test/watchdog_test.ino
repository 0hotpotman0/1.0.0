
#include <WatchDog.h>


void setup()
{
	wwdg.begin();
	
	pinMode(5, OUTPUT);
	digitalWrite(5, HIGH);
	delay(10);
	digitalWrite(5, LOW);
	delay(10);
	digitalWrite(5, HIGH);
}

void loop()
{	
	delay(24);
	wwdg.reset();
}
