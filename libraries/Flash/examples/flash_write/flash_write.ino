
#include <Flash.h>

void setup()
{
	Serial.begin(115200);
	
	for(int i = 0; i < 128; i++)
	{		
		Serial.print("Addr:\t");
		Serial.print(i);
		Serial.print("\tWriting: ");
		Flash.write32(i, i); 
		Serial.println(i);
	}
	Serial.println("done writing");
}

void loop()
{
	Serial.println("loop ...");
	delay(1000);
}