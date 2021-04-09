
#include <Flash.h>

void setup()
{
	Serial.begin(115200);
	
	unsigned long value;
	for(int i = 0; i < 128; i++)
	{		
		Serial.print("Addr:\t");
		Serial.print(i);
		Serial.print("\tReading: ");
		value = Flash.read32(i); 
		Serial.println(value);
	}
	Serial.println("done reading");
}

void loop()
{
	
}
