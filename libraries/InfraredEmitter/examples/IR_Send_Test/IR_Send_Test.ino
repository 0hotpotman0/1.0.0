
#include "InfraredEmitter.h"


// InfraredEmitter ir(PA6);
InfraredEmitter ir(PA7);

void setup()
{
	Serial.begin(115200);
	Serial.println("Infrared Emitter");
    
    pinMode(PB1, OUTPUT); 
    digitalWrite(PB1, HIGH); // power on grove zero ir sensor board
    
	ir.Init();
}

void loop()
{
    for(int i = 0; i < 24; i ++)
    {
        ir.InfraredEmitterWriteCodeB(i);
        delay(2000);
    }
}
