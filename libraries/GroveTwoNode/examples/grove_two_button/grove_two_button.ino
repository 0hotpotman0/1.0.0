
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>

#define GROVE_TWO_TX_PIN_NUM	PA2
#define GROVE_TWO_RX_PIN_NUM	PA3

#define BUTTON_LED_PIN_NUM		PA1

#define BUTTON_A_PIN_NUM		PA0
#define BUTTON_B_PIN_NUM		PA4

#ifdef BUTTON_V_3
#define BUTTON_ADDRESS_A		PA5
#define BUTTON_ADDRESS_B		PA6
#endif

/***************************************************************

 ***************************************************************/
#define SINGLE_KEY_TIME     3       //  SINGLE_KEY_TIME*10MS = 30MS
#define KEY_INTERVAL        30      //  KEY_INTERVAL*10MS    = 300MS
#define LONG_KEY_TIME       200     //  LONG_KEY_TIME*10MS   = 2S

#define KEY_STATE_0         0
#define KEY_STATE_1         1
#define KEY_STATE_2         2
#define KEY_STATE_3         3
#define KEY_STATE_4         4
#define KEY_STATE_5         5

#define N_KEY               0       //  no click
#define S_KEY               1       //  single click
#define D_KEY               2 
#define L_KEY               3      //   long press

enum button_status_type_t
{ 
	BUTTON_NO_EVENT = 0,
	BUTTON_A_CLICK = 1,
	BUTTON_A_DOUBLE_CLICK = 2,
	BUTTON_A_LONG_PRESS = 3,
	BUTTON_B_CLICK = 4,
	BUTTON_B_DOUBLE_CLICK = 5,
	BUTTON_B_LONG_PRESS = 6,
	BUTTON_A_AND_B_CLICK = 7,
	BUTTON_A_AND_B_DOUBLE_CLICK = 8,
	BUTTON_A_AND_B_LONG_PRESS = 9,
  BUTTON_A_PRESS = 10,
  BUTTON_B_PRESS = 11,
  BUTTON_A_AND_B_PRESS = 12
};

uint32_t clickPreviousMillis = 0;
uint8_t buttonAddValue = 0;
uint8_t buttonDriver(void);
uint8_t buttonRead(void);



uint8_t	ledFlashTimes = 0;


//#ifdef BLE_SUPPORT
//
//uint8_t Core_mode = 0, ErrorCount=0;
uint32_t StartMillis = 0;
//uint32_t preEvent;
//
//typedef struct
//{
//	uint8_t Datalen;
//	uint8_t type;
//	uint8_t Address;
//	uint8_t Option;
//}packet_header_t;
//
//typedef struct
//{
//	uint8_t 	Raw_data_type;
//	uint8_t		delay[2];
//}packet_raw_t;
//
//typedef struct
//{
//	uint8_t 	pid[2];
//	uint8_t 	chipid;
//	uint8_t 	Newaddress;
//	uint8_t 	option[5];
//}packet_got_atr;
//
//
//typedef struct
//{
//	packet_header_t	Header;
//	uint8_t		data[2];
//}packet_raw;
//
//typedef struct
//{
//	packet_header_t	Header;
//	uint8_t		Event;
//}packet_event;
//
//typedef struct
//{
//	packet_header_t	Header;
//	uint8_t 	pid[2];
//	uint8_t 	chipid;
//	uint8_t 	hwaddress;
//	uint8_t 	version[3];
//	uint8_t 	option[2];
//}packet_atr;

uint8_t	LightRawData = 0;
uint32_t RawPreviousMillis = 0;

//#endif

//typedef struct
//{
//	uint16_t deviceVID;
//	uint16_t devicePID;
//	uint32_t deviceEvent;
//}packet_01_t; // 8 bytes


/***************************************************************

 ***************************************************************/
//LowPower nrgSave;

#define CLICK_CHECK_TIMEOUT	2000
#define AUTO_SLEEP_TIMEOUT	25000

uint32_t clickCheckPreviousMillis = 0;
uint32_t autoSleepPreviousMillis = 0;
bool autoSleepFlag = false;

#define LED_FLASH_TIME	250

bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0, PreMillis = 0;

bool buttonDetectMode = false;

bool testFlag = false;
char *versions = "V20";
uint16_t NodeVersion = 0x6100;

uint32_t intStart = 0;
uint32_t intEnd = 0;

/***************************************************************

 ***************************************************************/
uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/***************************************************************

 ***************************************************************/
void setup()
{
    uint8_t *ptr3 = (uint8_t *)Flash.getChipUniqueID();

    for(uint8_t i = 0; i < 12; i ++)chipId[i] = *(ptr3 + i);



	pinMode(BUTTON_LED_PIN_NUM, OUTPUT);


	digitalWrite(BUTTON_LED_PIN_NUM, HIGH);


//#ifdef BLE_SUPPORT
	StartMillis = millis();
//#endif

	wwdg.begin();
}

void loop()
{
	uint32_t CurrentMillis = millis();
			
	if(Wire.isbusidle())PreMillis = CurrentMillis;



	if(ledFlashCommand)
	{
		uint32_t ledFlashCurrentMillis = millis();
		if(ledFlashCurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
		{
			ledFlashPreviousMillis = ledFlashCurrentMillis;
			digitalWrite(BUTTON_LED_PIN_NUM, ledFlashStatus);
			ledFlashStatus = !ledFlashStatus;
			if(ledFlashTimes < LED_FLASH_COUNT)
			{
				if (!ledFlashStatus)
				{
					ledFlashTimes++;
					if (ledFlashTimes >= LED_FLASH_COUNT)ledFlashCommand = false;
				}
			}
		}
	}


    
    if(testFlag)
    {
        wwdg.end();
        pinMode(GROVE_TWO_TX_PIN_NUM, OUTPUT);
        pinMode(GROVE_TWO_RX_PIN_NUM, OUTPUT);
        
        while(1)
        {           
            digitalWrite(GROVE_TWO_TX_PIN_NUM, HIGH);
            digitalWrite(GROVE_TWO_RX_PIN_NUM, HIGH);
            delay(1);
            digitalWrite(GROVE_TWO_TX_PIN_NUM, LOW);
            delay(1);
            
            digitalWrite(GROVE_TWO_TX_PIN_NUM, HIGH);
            digitalWrite(GROVE_TWO_RX_PIN_NUM, LOW);
            delay(1);
            digitalWrite(GROVE_TWO_TX_PIN_NUM, LOW);
            delay(1);
            
            if(testFlag == false)break;
        }
        
        wwdg.begin();
 
    }

	wwdg.reset();
}
