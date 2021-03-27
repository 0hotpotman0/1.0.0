
#include <LowPower.h>

LowPower nrgSave;

void setup()
{
	nrgSave.begin(0, dummy);  //standby setup for external interrupts
	
	// pinMode(5, OUTPUT);
	// digitalWrite(5, HIGH);
}

void loop()
{
	nrgSave.standby();  //now mcu goes in stop mode, wait for exit
	
	// for(unsigned int i = 0; i < 5; i ++)
	// {
		// digitalWrite(5, LOW);
		// delay(200);
		// digitalWrite(5, HIGH);
		// delay(200);
	// }
	
	delay(3000);
}


void dummy(void)  //interrupt routine (isn't necessary to execute any tasks in this routine
{

}
