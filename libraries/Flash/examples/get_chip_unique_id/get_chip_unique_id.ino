
#include <Flash.h>

uint32_t *id = NULL;
	
void setup()
{
	Serial.begin(115200);
	
	id = Flash.getChipUniqueID();
	
	Serial.print("0x");
	Serial.print(id[2], HEX);
	Serial.print("-");
	Serial.print(id[1], HEX);
	Serial.print("-");
	Serial.println(id[0], HEX);
}

void loop()
{
	
}