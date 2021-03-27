
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>

#define GROVE_TX_PIN_NUM	PA2
#define GROVE_RX_PIN_NUM	PA3

#define GROVE_LED_PIN_NUM		PA1

/***************************************************************
 Board defines
 ***************************************************************/
#define ENCODER_A_PIN     PA4
#define ENCODER_B_PIN     PA6
#define ENCODER_BTN_PIN   PA0

#ifdef BUTTON_V_3
#define BUTTON_ADDRESS_A		PF0
#define BUTTON_ADDRESS_B		PF1
#endif

bool readyMsg = false;
bool rotateFlag = false;
bool rotateDirect = false;
bool buttonFlag = false;

/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x38
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x000b

#define I2C_DEF_ADDR_FLASH_LOC	0x00
#define I2C_CUR_ADDR_FLASH_LOC	0x01

#define I2C_CMD_GET_DEV_ID		0x00 // 
#define I2C_CMD_GET_DEV_EVENT	0x01 //
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
#define I2C_CMD_NULL			0xff // 

uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS;
uint8_t commandReceive = I2C_CMD_NULL;
uint8_t	ledFlashTimes = 0;

#ifdef BLE_SUPPORT

uint8_t Core_mode = 0;
uint32_t StartMillis = 0, PreMillis = 0;

typedef struct
{
	uint8_t Datalen;
	uint8_t type;
	uint8_t Address;
	uint8_t Option;
}packet_header_t;

typedef struct
{
	uint8_t threshold0[2];
	uint8_t threshold1[2];
	uint8_t flashSave;
}packet_thlsd_t;


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
		packet_raw_t	raw;
		packet_thlsd_t	thread;
		packet_got_atr	atr;
		uint8_t Option[MAINBOARD_BLE_I2C_DATALEN-sizeof(packet_header_t)];
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
Basic defines
 ***************************************************************/
LowPower nrgSave;

#define CLICK_CHECK_TIMEOUT	1000
#define AUTO_SLEEP_TIMEOUT	2000

uint32_t clickCheckPreviousMillis = 0;
uint32_t autoSleepPreviousMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

#define LED_FLASH_TIME	250

bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;

bool testFlag = false;
char *versions = "V20";
uint16_t NodeVersion = 0x6104;
// 6104: Add INPUT_PULLUP for button pin(PA0).
uint32_t intStart = 0;
uint32_t intEnd = 0;
uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/***************************************************************
 Device initialization
 ***************************************************************/
void setup()
{
	uint8_t i2cDefaultAddr = Flash.read8(I2C_DEF_ADDR_FLASH_LOC); 
	uint8_t i2cCurrentAddr = Flash.read8(I2C_CUR_ADDR_FLASH_LOC);
	
	if(i2cDefaultAddr == 0xff)Flash.write8(I2C_DEF_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	if(i2cCurrentAddr == 0xff)Flash.write8(I2C_CUR_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	else deviceI2CAddress = i2cCurrentAddr;
	
	packet_01_data.data.deviceVID = DEVICE_VID;
	packet_01_data.data.devicePID = DEVICE_PID;
	packet_01_data.data.deviceEvent = 0;
	
	nrgSave.begin(GROVE_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance

	pinMode(GROVE_LED_PIN_NUM, OUTPUT);
	digitalWrite(GROVE_LED_PIN_NUM, HIGH);
#ifdef BUTTON_V_3
	pinMode(BUTTON_ADDRESS_A, INPUT);
	pinMode(BUTTON_ADDRESS_B, INPUT);
	delay(100);

// 0x10/0x30/0x38/0x18.
	deviceI2CAddress -= digitalRead(BUTTON_ADDRESS_A)<<3;
	deviceI2CAddress -= digitalRead(BUTTON_ADDRESS_B)<<5;
#endif

#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif
	    
    attachInterrupt(ENCODER_A_PIN, encoderAEvent, FALLING, INPUT);
	attachInterrupt(ENCODER_B_PIN, encoderBEvent, FALLING, INPUT);
    attachInterrupt(ENCODER_BTN_PIN, encoderButtonEvent, FALLING, INPUT_PULLUP);
	
	Wire.begin(deviceI2CAddress);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);
  
	wwdg.begin();
}

void loop()
{
	uint32_t CurrentMillis = millis();
			
#ifdef BLE_SUPPORT
	if(Wire.isbusidle())PreMillis = CurrentMillis;
	if ((CurrentMillis - PreMillis) > 20)
	{
		Wire.reset();
		Wire.begin(deviceI2CAddress);
		Wire.onReceive(receiveEvent);
		Wire.onRequest(requestEvent);
		PreMillis = CurrentMillis;
	}
#endif

    if((digitalRead(ENCODER_A_PIN) & digitalRead(ENCODER_B_PIN)) == HIGH)
        readyMsg = true;
    
    if(rotateFlag)
    {
        rotateFlag = false;
        if(rotateDirect)
            packet_01_data.data.deviceEvent = 1;
        else packet_01_data.data.deviceEvent = 2;
        
#ifdef BLE_SUPPORT
		if(Core_mode == CORE_BLE_MODE)
		{
			InstructionOption.event.Header.type = I2C_CMD_NOTIFY_EVENT;
			InstructionOption.event.Header.Address = deviceI2CAddress;
			InstructionOption.event.Event  = packet_01_data.data.deviceEvent;
			InstructionOption.event.Header.Datalen	 = sizeof(packet_event);
			Wire.MasterGPIOTransmission(ptr2, sizeof(packet_event));
		}
#endif
	}
    
    if(buttonFlag)
    {
        buttonFlag = false;
        packet_01_data.data.deviceEvent = 3;
        
#ifdef BLE_SUPPORT
		if(Core_mode == CORE_BLE_MODE)
		{
			InstructionOption.event.Header.type = I2C_CMD_NOTIFY_EVENT;
			InstructionOption.event.Header.Address = deviceI2CAddress;
			InstructionOption.event.Event  = packet_01_data.data.deviceEvent;
			InstructionOption.event.Header.Datalen	 = sizeof(packet_event);
			Wire.MasterGPIOTransmission(ptr2, sizeof(packet_event));
		}
#endif
    }

	if(ledFlashCommand)
	{
		uint32_t ledFlashCurrentMillis = millis();
		if(ledFlashCurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
		{
			ledFlashPreviousMillis = ledFlashCurrentMillis;
			digitalWrite(GROVE_LED_PIN_NUM, ledFlashStatus);
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
			commandReceive = I2C_CMD_NOTIFY_ATR;
			StartMillis = CurrentMillis;
#endif
		}
	}
	if(Core_mode == CORE_BLE_MODE)
	{// Message process.

		switch(commandReceive)
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
			digitalWrite(GROVE_LED_PIN_NUM, HIGH);
			
			wwdg.end();
			Wire.end();
			pinMode(PA9, INPUT_PULLUP);
			pinMode(PA10, INPUT_PULLUP);
			
			nrgSave.standby();
			
			Wire.begin(deviceI2CAddress);
			Wire.onReceive(receiveEvent);
			Wire.onRequest(requestEvent);
			wwdg.begin();
		}
	}
    
    if(testFlag)
    {
        wwdg.end();
        pinMode(GROVE_TX_PIN_NUM, OUTPUT);
        pinMode(GROVE_RX_PIN_NUM, OUTPUT);
        
        while(1)
        {           
            digitalWrite(GROVE_TX_PIN_NUM, HIGH);
            digitalWrite(GROVE_RX_PIN_NUM, HIGH);
            delay(1);
            digitalWrite(GROVE_TX_PIN_NUM, LOW);
            delay(1);
            
            digitalWrite(GROVE_TX_PIN_NUM, HIGH);
            digitalWrite(GROVE_RX_PIN_NUM, LOW);
            delay(1);
            digitalWrite(GROVE_TX_PIN_NUM, LOW);
            delay(1);
            
            if(testFlag == false)break;
        }
        
        wwdg.begin();
        attachInterrupt(GROVE_RX_PIN_NUM, dummy, CHANGE, INPUT_PULLUP);	
    }

	wwdg.reset();
}

void dummy(void)
{
	autoSleepPreviousMillis = millis();    
    
    if(digitalRead(GROVE_RX_PIN_NUM) == LOW)intStart = millis();
    else 
    {
        intEnd = millis();
        if((intEnd - intStart) > 20)delay(500);
        else intStart = intEnd;
    }
}

void encoderAEvent(void)
{
    autoSleepPreviousMillis = millis();
    
    if(readyMsg)
    {
        if(digitalRead(ENCODER_A_PIN) > digitalRead(ENCODER_B_PIN))
	    {
            rotateDirect = true;
            rotateFlag = true;
            readyMsg = false;
        }
        else if(digitalRead(ENCODER_A_PIN) < digitalRead(ENCODER_B_PIN))
        {
            rotateDirect = false;
            rotateFlag = true;
            readyMsg = false;
        }
    }
}

void encoderBEvent(void)
{
    autoSleepPreviousMillis = millis();
    
    if(readyMsg)
    {
        if(digitalRead(ENCODER_A_PIN) > digitalRead(ENCODER_B_PIN))
	    {
            rotateDirect = true;
            rotateFlag = true;
            readyMsg = false;
        }
        else if(digitalRead(ENCODER_A_PIN) < digitalRead(ENCODER_B_PIN))
        {
            rotateDirect = false;
            rotateFlag = true;
            readyMsg = false;
        }
    }
}

void encoderButtonEvent(void)
{
    autoSleepPreviousMillis = millis();
    
    buttonFlag = true;
}

void receiveEvent(int howMany)
{
	uint8_t i = 0, receiveBuffer[MAINBOARD_BLE_I2C_DATALEN] = {0,};
	// autoSleepPreviousMillis = millis();
	
	while(Wire.available())
	{	
		receiveBuffer[i ++] = Wire.read();
		if(i >= MAINBOARD_BLE_I2C_DATALEN)i--;
	}
#ifdef BLE_SUPPORT
	if((receiveBuffer[0] >= MAINBOARD_BLE_COMMAND_LOW) && (receiveBuffer[0] <  MAINBOARD_BLE_COMMAND_HIGH))
	{// BLE command: len,cmd,opt,...
		Core_mode = CORE_BLE_MODE;
		memcpy(commandOption.bytes, receiveBuffer, i);
		commandOption.data.Header.Datalen -= MAINBOARD_BLE_COMMAND_LOW;
		if (i != commandOption.data.Header.Datalen)
		{// Bus error!!!
			return;
		}
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
		case I2C_CMD_LED_ON:
			ledFlashCommand = true;
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_LED_OFF:
			ledFlashCommand = false;
			ledFlashStatus = false;
			digitalWrite(GROVE_LED_PIN_NUM, HIGH);
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
			packet_01_data.data.deviceEvent = 0;
		break;
        
        case I2C_CMD_TEST_GET_VER:
            Wire.write(versions, 3);
            commandReceive = I2C_CMD_NULL;
        break;
		
		default:
		break;
	}
}

/***************************************************************
 Device driver
 ***************************************************************/
