
#include "InfraredReceiver.h"

// 0 - '0'
// 1 - '1'
// 2 - '2'
// 3 - '3'
// 4 - '4'
// 5 - '5'
// 6 - '6'
// 7 - '7'
// 8 - '8'
// 9 - '9'
// 10 - 'Power'
// 11 - 'Mode'
// 12 - 'Speaker'
// 13 - 'Play/Pause'
// 14 - 'Fast backward'
// 15 - 'Fast forward'
// 16 - 'EQ'
// 17 - 'VOL-'
// 18 - 'VOL+'
// 19 - 'RPT'
// 20 - 'U/SD'

InfraredReceiver infraredReceiver(PA0);

void setup()
{
	Serial.begin(115200);
	Serial.println("Infrared Receiver");
	infraredReceiver.Init();
}

void loop()
{
	int data = infraredReceiver.InfraredReceiverReadCode();
    if(data >= 0)
    {
        Serial.print("Code: ");
        Serial.println(data);
    }
}
