#include <Timer3.h>

bool isLEDOn = false;

void setup() 
{    
    pinMode(PA4, OUTPUT); 
	digitalWrite(PA4, HIGH);

    Timer3.init(1000);
    Timer3.attachInterrupt(timerIsr);
}
 
void loop()
{

}

void timerIsr()
{    
    digitalWrite(PA4, isLEDOn);
    isLEDOn = !isLEDOn;
}