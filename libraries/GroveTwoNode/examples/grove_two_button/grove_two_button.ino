
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

/***************************************************************

 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x02
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x0002

#define I2C_DEF_ADDR_FLASH_LOC	0x00
#define I2C_CUR_ADDR_FLASH_LOC	0x01

#define I2C_CMD_GET_DEV_ID		0x00 // 
#define I2C_CMD_GET_DEV_EVENT	0x01 //
#define I2C_CMD_EVENT_DET_MODE  0x02 //
#define I2C_CMD_BLOCK_DET_MODE  0x03 //
#define I2C_CMD_LED_ON			0xb0 // 
#define I2C_CMD_LED_OFF			0xb1 // 
#define I2C_CMD_AUTO_SLEEP_ON	0xb2 // 
#define I2C_CMD_AUTO_SLEEP_OFF	0xb3 // 
#define I2C_CMD_SET_ADDR		0xc0 //
#define I2C_CMD_RST_ADDR		0xc1 // 
#define I2C_CMD_TEST_TX_RX_ON   0xe0 // 
#define I2C_CMD_TEST_TX_RX_OFF  0xe1 // 
#define I2C_CMD_TEST_GET_VER    0xe2 // 
#define I2C_CMD_JUMP_TO_BOOT	0xf0 // 
#define I2C_CMD_GET_DEVICE_UID  0xf1 // 
#define I2C_CMD_NULL			0xff // 

uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS;
uint8_t commandReceive = I2C_CMD_NULL;
uint8_t	ledFlashTimes = 0;


#ifdef BLE_SUPPORT

uint8_t Core_mode = 0, ErrorCount=0;
uint32_t StartMillis = 0;
uint32_t preEvent;

typedef struct
{
	uint8_t Datalen;
	uint8_t type;
	uint8_t Address;
	uint8_t Option;
}packet_header_t;

typedef struct
{
	uint8_t 	Raw_data_type;
	uint8_t		delay[2];
}packet_raw_t;

typedef struct
{
	uint8_t 	pid[2];
	uint8_t 	chipid;
	uint8_t 	Newaddress;
	uint8_t 	option[5];
}packet_got_atr;

typedef struct
{
	packet_header_t	Header;
	union 
	{
		packet_raw_t		raw;
		packet_got_atr		atr;
		uint8_t Option[MAINBOARD_BLE_I2C_DATALEN-3];
	}commands;

}packet_thld_t; // 8 bytes

union
{
    packet_thld_t data;
    uint8_t bytes[MAINBOARD_BLE_I2C_DATALEN]; 
}commandOption;

typedef struct
{
	packet_header_t	Header;
	uint8_t		data[2];
}packet_raw;

typedef struct
{
	packet_header_t	Header;
	uint8_t		Event;
}packet_event;

typedef struct
{
	packet_header_t	Header;
	uint8_t 	pid[2];
	uint8_t 	chipid;
	uint8_t 	hwaddress;
	uint8_t 	version[3];
	uint8_t 	option[2];
}packet_atr;

union
{
	packet_atr		atr;
	packet_event	event;
	packet_raw		raw_data;
    uint8_t 		bytes[MAINBOARD_BLE_I2C_DATALEN]; 
}InstructionOption;


uint8_t	LightRawData = 0;
uint32_t RawDelayMillis = I2C_CMD_RAW_DATA_TIME;
uint32_t RawPreviousMillis = 0;
uint8_t *ptr2 = (uint8_t *)&InstructionOption;

#endif

typedef struct
{
	uint16_t deviceVID;
	uint16_t devicePID;
	uint32_t deviceEvent;
}packet_01_t; // 8 bytes

union
{
    packet_01_t data;
    uint8_t bytes[8]; 
}packet_01_data;

uint8_t *ptr1 = (uint8_t *)&packet_01_data;

void requestEvent();
void receiveEvent(int howMany);

/***************************************************************

 ***************************************************************/
LowPower nrgSave;

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
	uint8_t i2cDefaultAddr = Flash.read8(I2C_DEF_ADDR_FLASH_LOC); 
	uint8_t i2cCurrentAddr = Flash.read8(I2C_CUR_ADDR_FLASH_LOC);
    uint8_t *ptr3 = (uint8_t *)Flash.getChipUniqueID();

    for(uint8_t i = 0; i < 12; i ++)chipId[i] = *(ptr3 + i);

	if(i2cDefaultAddr == 0xff)Flash.write8(I2C_DEF_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	if(i2cCurrentAddr == 0xff)Flash.write8(I2C_CUR_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	else deviceI2CAddress = i2cCurrentAddr;
	
	packet_01_data.data.deviceVID = DEVICE_VID;
	packet_01_data.data.devicePID = DEVICE_PID;
	packet_01_data.data.deviceEvent = (uint32_t)BUTTON_NO_EVENT;
	
	nrgSave.begin(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance

	pinMode(BUTTON_LED_PIN_NUM, OUTPUT);

#ifdef BUTTON_V_3
	pinMode(BUTTON_ADDRESS_A, INPUT);
	pinMode(BUTTON_ADDRESS_B, INPUT);
#endif

	digitalWrite(BUTTON_LED_PIN_NUM, HIGH);

	attachInterrupt(BUTTON_A_PIN_NUM, butttonAEvent, FALLING, INPUT);
	attachInterrupt(BUTTON_B_PIN_NUM, butttonBEvent, FALLING, INPUT);

#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif

#ifdef BUTTON_V_3
// 0x02, 0x0A, 0x12, 0x1A.
	delay(100);
	deviceI2CAddress += digitalRead(BUTTON_ADDRESS_A)<<4;
	deviceI2CAddress += digitalRead(BUTTON_ADDRESS_B)<<3;
#endif
	
	Wire.begin(deviceI2CAddress);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);

	wwdg.begin();
}

void loop()
{
    button_status_type_t button_status;
	uint32_t CurrentMillis = millis();
			
	if(Wire.isbusidle())PreMillis = CurrentMillis;
	if ((CurrentMillis - PreMillis) > 20)
	{
		Wire.reset();
		Wire.begin(deviceI2CAddress);
		Wire.onReceive(receiveEvent);
		Wire.onRequest(requestEvent);
		PreMillis = CurrentMillis;
	}

    if(buttonDetectMode == false)button_status = buttonEventdetect();
    else button_status = buttonBlockDetect();
	
	if(button_status != BUTTON_NO_EVENT)
	{
		packet_01_data.data.deviceEvent = button_status;
		clickCheckPreviousMillis = millis();
#ifdef BLE_SUPPORT
		if(Core_mode == CORE_BLE_MODE)
		{
		//	if(preEvent != button_status)
			{
				InstructionOption.event.Header.type		= I2C_CMD_NOTIFY_EVENT;
				InstructionOption.event.Header.Address	= deviceI2CAddress;
				InstructionOption.event.Event 			= packet_01_data.data.deviceEvent;
				InstructionOption.event.Header.Datalen	= sizeof(packet_event);
				Wire.MasterGPIOTransmission(ptr2, sizeof(packet_event));
				preEvent = packet_01_data.data.deviceEvent;
			}
		}
#endif
	}
	else if(packet_01_data.data.deviceEvent)
	{
		uint32_t clickCheckCurrentMillis = millis();
		if(clickCheckCurrentMillis - clickCheckPreviousMillis >= CLICK_CHECK_TIMEOUT)
		{
			clickCheckPreviousMillis = clickCheckCurrentMillis;
			packet_01_data.data.deviceEvent = BUTTON_NO_EVENT;	
#ifdef BLE_SUPPORT
			preEvent = BUTTON_NO_EVENT;
#endif
		}
	}

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
#ifdef BLE_SUPPORT
	if (Core_mode == 0)
	{// ATR
		uint32_t CurrentMillis = millis();
		
		
		if(CurrentMillis - StartMillis >= (I2C_CMD_SYS_READY_TIME+deviceI2CAddress*2))
		{
#if 1
			Core_mode = CORE_BLE_MODE;
#else
			StartMillis = CurrentMillis;
			commandReceive = I2C_CMD_NOTIFY_ATR;
#endif

		}
	}
	if(Core_mode == CORE_BLE_MODE)
	{
		uint32_t CurrentMillis = millis();

		if (LightRawData)
		{
			if (CurrentMillis - RawPreviousMillis >= RawDelayMillis)
			{
				
				InstructionOption.raw_data.Header.type = I2C_CMD_GET_RAW_DATA;
				InstructionOption.raw_data.Header.Address = deviceI2CAddress;
				InstructionOption.raw_data.data[0] = buttonBlockDetect();
				InstructionOption.raw_data.data[1] = 0;
				InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);
				if(Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw)) == 0)
				{
					ErrorCount++;
					RawPreviousMillis += 13; // Retry in 3 ms later.
				}else
				{
					ErrorCount = 0;
					RawPreviousMillis = CurrentMillis;
				}
				if (ErrorCount > 10)
				{// I2C bus error, reset I2C bus.
					Wire.end();
					Wire.begin(deviceI2CAddress);
					Wire.onReceive(receiveEvent);
					Wire.onRequest(requestEvent);
					ErrorCount = 0;
				}
			}
		}
		switch (commandReceive)
		{

			case I2C_CMD_NOTIFY_ATR:
				InstructionOption.atr.Header.type	= I2C_CMD_ATR;
				InstructionOption.atr.Header.Address= deviceI2CAddress;
				InstructionOption.atr.pid[0]		= DEVICE_PID&0xff;
				InstructionOption.atr.pid[1]		= (DEVICE_PID>>8 ) & 0xff;
				InstructionOption.atr.chipid		= chipId[0];
				InstructionOption.atr.hwaddress		= DEVICE_I2C_ADDRESS;
				InstructionOption.atr.version[0]	= versions[0];
				InstructionOption.atr.version[1]	= versions[1];
				InstructionOption.atr.version[2]	= versions[2];
				InstructionOption.atr.option[0]		= NodeVersion&0xFF;
				InstructionOption.atr.option[1]		= (NodeVersion>>8)&0xFF;
				InstructionOption.atr.Header.Datalen = sizeof(packet_atr);
				Wire.MasterGPIOTransmission(ptr2, sizeof(packet_atr));
				commandReceive = I2C_CMD_NULL;
				break;
			case I2C_CMD_ATR:
				if ((commandOption.data.commands.atr.pid[0] == (DEVICE_PID&0xFF)) && 
					(commandOption.data.commands.atr.pid[1] == ((DEVICE_PID>>8)&0xFF)) && 
					(commandOption.data.commands.atr.chipid == chipId[0]))
				{// It's for current device
					if(commandOption.data.commands.atr.Newaddress != deviceI2CAddress)
					{
						deviceI2CAddress = commandOption.data.commands.atr.Newaddress;
						Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
						Wire.begin(deviceI2CAddress);
					}
					
				}
				commandReceive = I2C_CMD_NULL;
				break;
			default:
				break;
		}
	}else 
	{
#endif

	if(commandReceive == I2C_CMD_SET_ADDR) // change i2c address
	{
		commandReceive = I2C_CMD_NULL;
		Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
		Wire.begin(deviceI2CAddress);
	}
	else if(commandReceive == I2C_CMD_RST_ADDR) // reset i2c address
	{
		commandReceive = I2C_CMD_NULL;
		deviceI2CAddress = Flash.read8(I2C_DEF_ADDR_FLASH_LOC);
		Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
		Wire.begin(deviceI2CAddress);
	}
	
#ifdef BLE_SUPPORT
	}
#endif
	if(autoSleepFlag)
	{
		uint32_t autoSleepCurrentMillis = millis();
		if((autoSleepCurrentMillis - autoSleepPreviousMillis) > AUTO_SLEEP_TIMEOUT)
		{
			autoSleepPreviousMillis = autoSleepCurrentMillis;
			
			ledFlashCommand = false;
			ledFlashStatus = false;
			digitalWrite(BUTTON_LED_PIN_NUM, HIGH);
			
			wwdg.end();
			Wire.end();
			pinMode(PA9, INPUT_PULLUP);
			pinMode(PA10, INPUT_PULLUP);
			
			nrgSave.standby();
			
			ledFlashCommand = true;
			Wire.begin(deviceI2CAddress);
			Wire.onReceive(receiveEvent);
			Wire.onRequest(requestEvent);
			wwdg.begin();
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
        attachInterrupt(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE, INPUT_PULLUP);	
    }

	wwdg.reset();
}

button_status_type_t buttonEventdetect(void)
{
	button_status_type_t button_status = BUTTON_NO_EVENT;
	uint32_t clickCurrentMillis = millis();
	
    if(clickCurrentMillis - clickPreviousMillis >= 10)
	{
		clickPreviousMillis = clickCurrentMillis;
		button_status = (button_status_type_t)buttonRead();
	}
	
	return button_status;
}

button_status_type_t buttonBlockDetect(void)
{
    button_status_type_t button_status = BUTTON_NO_EVENT;
	uint32_t clickCurrentMillis = millis();
	
//    if(clickCurrentMillis - clickPreviousMillis >= 10)
	{
//		clickPreviousMillis = clickCurrentMillis;
        
        if(digitalRead(BUTTON_A_PIN_NUM) == LOW && digitalRead(BUTTON_B_PIN_NUM) == LOW)button_status = BUTTON_A_AND_B_PRESS;
		else if(digitalRead(BUTTON_A_PIN_NUM) == LOW && digitalRead(BUTTON_B_PIN_NUM) == HIGH)button_status = BUTTON_A_PRESS;
        else if(digitalRead(BUTTON_A_PIN_NUM) == HIGH && digitalRead(BUTTON_B_PIN_NUM) == LOW)button_status = BUTTON_B_PRESS;
        else if(digitalRead(BUTTON_A_PIN_NUM) == HIGH && digitalRead(BUTTON_B_PIN_NUM) == HIGH)button_status = BUTTON_NO_EVENT;
	}
    
	return button_status;
}

void dummy(void)
{
	autoSleepPreviousMillis = millis();    
    
    if(digitalRead(GROVE_TWO_RX_PIN_NUM) == LOW)intStart = millis();
    else 
    {
        intEnd = millis();
        if((intEnd - intStart) > 20)delay(500);
        else intStart = intEnd;
    }
}

void butttonAEvent()
{
	autoSleepPreviousMillis = millis();
}

void butttonBEvent()
{
	autoSleepPreviousMillis = millis();
}

void receiveEvent(int howMany)
{
	uint8_t i = 0, receiveBuffer[MAINBOARD_BLE_I2C_DATALEN] = {0,};
	// autoSleepPreviousMillis = millis();
	
	while(Wire.available())
	{	
		receiveBuffer[i ++] = Wire.read();
		if(i >= MAINBOARD_BLE_I2C_DATALEN)i = 0;
	}
#ifdef BLE_SUPPORT
	if((receiveBuffer[0] >= MAINBOARD_BLE_COMMAND_LOW) && (receiveBuffer[0] <  MAINBOARD_BLE_COMMAND_HIGH))
	{// BLE command: len,cmd,opt,...
		Core_mode = CORE_BLE_MODE;
		memcpy(commandOption.bytes, receiveBuffer, i);
		commandOption.data.Header.Datalen -= MAINBOARD_BLE_COMMAND_LOW;
		if(i != commandOption.data.Header.Datalen)return;
		commandReceive = commandOption.data.Header.type;
	}else
	{
	Core_mode = CORE_ATMEL_MODE;
#endif

	commandReceive = receiveBuffer[0];
#ifdef BLE_SUPPORT
	}
#endif
	
	switch(commandReceive)
	{
#ifdef BLE_SUPPORT
		case I2C_CMD_GET_RAW_DATA:
			// Raw data request, 
				
			if (commandOption.data.commands.raw.Raw_data_type > 0)
			{
				LightRawData = commandReceive;
				RawDelayMillis = commandOption.data.commands.raw.delay[0]+commandOption.data.commands.raw.delay[1]*256+(chipId[0]&0x03);
				if (RawDelayMillis == 0)
					RawDelayMillis = I2C_CMD_RAW_DATA_TIME;
				RawPreviousMillis = 0;
			}else
				LightRawData = 0;
			commandReceive = I2C_CMD_NULL;
			
			break;
#endif
        case I2C_CMD_EVENT_DET_MODE:
			buttonDetectMode = false;
			commandReceive = I2C_CMD_NULL;
		break;
        
        case I2C_CMD_BLOCK_DET_MODE:
			buttonDetectMode = true;
			commandReceive = I2C_CMD_NULL;
		break;
        
		case I2C_CMD_LED_ON:
			ledFlashCommand = true;
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_LED_OFF:
			ledFlashCommand = false;
			ledFlashStatus = false;
			digitalWrite(BUTTON_LED_PIN_NUM, HIGH);
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_AUTO_SLEEP_ON:
			autoSleepFlag = true;
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_AUTO_SLEEP_OFF:
			autoSleepFlag = false;
			commandReceive = I2C_CMD_NULL;
		break;
		case I2C_CMD_SET_ADDR:
			deviceI2CAddress = receiveBuffer[1];
		break;
        
        case I2C_CMD_TEST_TX_RX_ON:
            testFlag = true;
			commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_TEST_TX_RX_OFF:
            testFlag = false;
			commandReceive = I2C_CMD_NULL;
        break;
		
		case I2C_CMD_JUMP_TO_BOOT:
			commandReceive = I2C_CMD_NULL;
			jumpToBootloader();
		break;
		
		default:
		break;
	}
}

void requestEvent(void)
{	
	// autoSleepPreviousMillis = millis();
	
#ifdef BLE_SUPPORT
	Core_mode = CORE_ATMEL_MODE;
#endif
	switch(commandReceive)
	{
		case I2C_CMD_GET_DEV_ID:
			Wire.write(ptr1, 4);
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_GET_DEV_EVENT:
			Wire.write(ptr1 + 4, 4);
			commandReceive = I2C_CMD_NULL;
			packet_01_data.data.deviceEvent = BUTTON_NO_EVENT;
#ifdef BLE_SUPPORT
			preEvent = BUTTON_NO_EVENT;
#endif
		break;
        
        case I2C_CMD_TEST_GET_VER:
            Wire.write(versions, 3);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_GET_DEVICE_UID:
            Wire.write(chipId, 12);
            commandReceive = I2C_CMD_NULL;
        break;
		
		default:
		break;
	}
}

/***************************************************************

 ***************************************************************/
uint8_t buttonDriver(void) 
{   
	static uint8_t keyState = 0;
	static uint16_t keyTime = 0;
	uint8_t keyPress, keyReturn; 

	keyReturn = N_KEY;
	
	if(digitalRead(BUTTON_A_PIN_NUM) == false && digitalRead(BUTTON_B_PIN_NUM) == false)
	{
		buttonAddValue = 6;
		keyPress = false;
	}
	else 
	{
		keyPress = true;	
	}
	
	if(buttonAddValue < 6)
	{
		if(digitalRead(BUTTON_A_PIN_NUM) == true && digitalRead(BUTTON_B_PIN_NUM) == false)
		{
			buttonAddValue = 3;
			keyPress = false;
		}
		else if(digitalRead(BUTTON_A_PIN_NUM) == false && digitalRead(BUTTON_B_PIN_NUM) == true)
		{
			buttonAddValue = 0;
			keyPress = false;
		}
		else 
		{
			keyPress = true;	
		}
	}

	switch(keyState)     
	{       
		case KEY_STATE_0:
			if(!keyPress)
			{
				keyTime = 0;
				keyState = KEY_STATE_1;
			}        
		break;

		case KEY_STATE_1:
			if(!keyPress)                     
			{
				keyTime ++;
				if(keyTime >= SINGLE_KEY_TIME)
				{
					keyState = KEY_STATE_2;
				}
			}         
			else  keyState = KEY_STATE_0;
		break; 

		case KEY_STATE_2:
			if(keyPress)
			{ 
				keyReturn = S_KEY;
				keyState = KEY_STATE_0;
			} 
			else
			{
				keyTime++;                     
				if(keyTime >= LONG_KEY_TIME)
				{
					keyReturn = L_KEY + buttonAddValue;
					keyState = KEY_STATE_3;
				}
			}
		break;

		case KEY_STATE_3:
			if(keyPress) 
			{
				keyState = KEY_STATE_0;
				buttonAddValue = 0;
			}        
		break; 

		default:
			keyState = KEY_STATE_0;
		break;
	}

	return  keyReturn;
}

uint8_t buttonRead(void)                            
{ 
	static uint8_t keyState1 = 0, keyTime1 = 0;
	uint8_t keyReturn, keyTemp;

	keyReturn = N_KEY;
	keyTemp = buttonDriver();

#ifdef DOUBLE_KEY_SUPPORT
	switch(keyState1) 
	{         
		case KEY_STATE_0:
			if(keyTemp == S_KEY)
			{ 
				keyTime1 = 0;
				keyState1 = KEY_STATE_1;
			}             
			else
			{
				keyReturn = keyTemp;
			}
		break;
		case KEY_STATE_1:
			if(keyTemp == S_KEY)
			{              
				keyReturn = D_KEY + buttonAddValue;
				keyState1 = KEY_STATE_0;
				buttonAddValue = 0;
			} 
			else
			{
				keyTime1 ++;
				if(keyTime1 >= KEY_INTERVAL)
				{ 
					keyReturn = S_KEY + buttonAddValue;
					keyState1 = KEY_STATE_0;
					buttonAddValue = 0;
				}              
			}              
		break; 
		default:
			keyState1 = KEY_STATE_0;
		break;
	}
#else
	if(keyTemp == S_KEY)
	{
		keyReturn = S_KEY + buttonAddValue;
	}else 
		keyReturn = keyTemp;
	if(keyTemp != N_KEY)
		buttonAddValue = 0;
#endif

	return keyReturn;
}
