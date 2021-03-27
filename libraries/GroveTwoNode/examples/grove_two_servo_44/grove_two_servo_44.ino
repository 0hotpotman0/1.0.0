#include <Wire.h>

#define GROVE_LED_PIN_NUM	PA1

/***************************************************************
 Board defines
 ***************************************************************/

/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x24

void receiveEvent(int howMany);

/***************************************************************
Basic defines
 ***************************************************************/
uint8_t commandReceive = 0;

#define LED_FLASH_TIME	250

// bool ledFlashCommand = true;
bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;

/***************************************************************
 Device initialization
 ***************************************************************/
void setup()
{

	pinMode(GROVE_LED_PIN_NUM, OUTPUT);
    digitalWrite(GROVE_LED_PIN_NUM, HIGH);

	Wire.begin(DEVICE_I2C_ADDRESS);
	Wire.onReceive(receiveEvent);
  
}

void loop()
{
	uint32_t CurrentMillis = millis();
			
	if(ledFlashCommand)
	{
		if(CurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
		{
			ledFlashPreviousMillis = CurrentMillis;
            digitalWrite(GROVE_LED_PIN_NUM, ledFlashStatus);
            ledFlashStatus = !ledFlashStatus;
		}
	}
	delay(10);
}

void receiveEvent(int howMany)
{
	uint8_t i = 0, receiveBuffer[16] = {0,};
	// autoSleepPreviousMillis = millis();
	
	while(Wire.available())
	{	
		receiveBuffer[i ++] = Wire.read();
		if(i >= 16)i = 0;
	}

	commandReceive = receiveBuffer[1];

	switch(commandReceive)
	{        
		case 0xB0:
			ledFlashCommand = true;
			commandReceive = 0;
		break;
		
		case 0xB1:
			ledFlashCommand = false;
			ledFlashStatus = false;
			digitalWrite(GROVE_LED_PIN_NUM, HIGH);
			commandReceive = 0;
        break;

		case 0xF0:
			commandReceive = 0;
			jumpToBootloader();
		break;
		
		default:
		break;
	}
}
