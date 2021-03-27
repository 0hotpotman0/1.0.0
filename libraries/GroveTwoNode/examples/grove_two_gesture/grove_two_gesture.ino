
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>


#define GROVE_TWO_TX_PIN_NUM	PA2
#define GROVE_TWO_RX_PIN_NUM	PA3

#define GROVE_LED_PIN_NUM		PA1

/***************************************************************
 Board defines
 ***************************************************************/
#define GESTURE_INT_PIN_NUM		PA0
#define GESTURE_VLED_PIN_NUM    PA4
#define SOFT_SCL_PIN	PA7
#define SOFT_SDA_PIN	PB1

#define I2C_DELAY		10
#define I2C_WAIT()		delayMicroseconds(I2C_DELAY)
#define I2C_SDA_IN()	pinMode(SOFT_SDA_PIN, INPUT)
#define I2C_SDA_OUT()	pinMode(SOFT_SDA_PIN, OUTPUT)
#define I2C_SDA_READ()	digitalRead(SOFT_SDA_PIN)
#define I2C_SCL_LOW()	digitalWrite(SOFT_SCL_PIN, LOW)
#define I2C_SCL_HIGH()	digitalWrite(SOFT_SCL_PIN, HIGH)
#define I2C_SDA_LOW()	digitalWrite(SOFT_SDA_PIN, LOW)
#define I2C_SDA_HIGH()	digitalWrite(SOFT_SDA_PIN, HIGH)

void i2cStart(void);
void i2cStop(void);
bool i2cWaitAck(void);
void i2cAck(void);
void i2cNAck(void);
void i2cSendByte(uint8_t txd);
uint8_t i2cReadByte(void);

#define PAJ7620_I2C_ADDR    0x73

enum GESTURE_TYPE {
    None = 0,
    Right = 1,
    Left = 2,
    Up = 3,
    Down = 4,
    Forward = 5,
    Backward = 6,
    Clockwise = 7,
    Anticlockwise = 8,
    Wave = 9
};

const uint8_t initRegisterArray[] = {
    0xEF, 0x00, 0x32, 0x29, 0x33, 0x01, 0x34, 0x00, 0x35, 0x01, 0x36, 0x00, 0x37, 0x07, 0x38, 0x17,
    0x39, 0x06, 0x3A, 0x12, 0x3F, 0x00, 0x40, 0x02, 0x41, 0xFF, 0x42, 0x01, 0x46, 0x2D, 0x47, 0x0F,
    0x48, 0x3C, 0x49, 0x00, 0x4A, 0x1E, 0x4B, 0x00, 0x4C, 0x20, 0x4D, 0x00, 0x4E, 0x1A, 0x4F, 0x14,
    0x50, 0x00, 0x51, 0x10, 0x52, 0x00, 0x5C, 0x02, 0x5D, 0x00, 0x5E, 0x10, 0x5F, 0x3F, 0x60, 0x27,
    0x61, 0x28, 0x62, 0x00, 0x63, 0x03, 0x64, 0xF7, 0x65, 0x03, 0x66, 0xD9, 0x67, 0x03, 0x68, 0x01,
    0x69, 0xC8, 0x6A, 0x40, 0x6D, 0x04, 0x6E, 0x00, 0x6F, 0x00, 0x70, 0x80, 0x71, 0x00, 0x72, 0x00,
    0x73, 0x00, 0x74, 0xF0, 0x75, 0x00, 0x80, 0x42, 0x81, 0x44, 0x82, 0x04, 0x83, 0x20, 0x84, 0x20,
    0x85, 0x00, 0x86, 0x10, 0x87, 0x00, 0x88, 0x05, 0x89, 0x18, 0x8A, 0x10, 0x8B, 0x01, 0x8C, 0x37,
    0x8D, 0x00, 0x8E, 0xF0, 0x8F, 0x81, 0x90, 0x06, 0x91, 0x06, 0x92, 0x1E, 0x93, 0x0D, 0x94, 0x0A,
    0x95, 0x0A, 0x96, 0x0C, 0x97, 0x05, 0x98, 0x0A, 0x99, 0x41, 0x9A, 0x14, 0x9B, 0x0A, 0x9C, 0x3F,
    0x9D, 0x33, 0x9E, 0xAE, 0x9F, 0xF9, 0xA0, 0x48, 0xA1, 0x13, 0xA2, 0x10, 0xA3, 0x08, 0xA4, 0x30,
    0xA5, 0x19, 0xA6, 0x10, 0xA7, 0x08, 0xA8, 0x24, 0xA9, 0x04, 0xAA, 0x1E, 0xAB, 0x1E, 0xCC, 0x19,
    0xCD, 0x0B, 0xCE, 0x13, 0xCF, 0x64, 0xD0, 0x21, 0xD1, 0x0F, 0xD2, 0x88, 0xE0, 0x01, 0xE1, 0x04,
    0xE2, 0x41, 0xE3, 0xD6, 0xE4, 0x00, 0xE5, 0x0C, 0xE6, 0x0A, 0xE7, 0x00, 0xE8, 0x00, 0xE9, 0x00,
    0xEE, 0x07, 0xEF, 0x01, 0x00, 0x1E, 0x01, 0x1E, 0x02, 0x0F, 0x03, 0x10, 0x04, 0x02, 0x05, 0x00,
    0x06, 0xB0, 0x07, 0x04, 0x08, 0x0D, 0x09, 0x0E, 0x0A, 0x9C, 0x0B, 0x04, 0x0C, 0x05, 0x0D, 0x0F,
    0x0E, 0x02, 0x0F, 0x12, 0x10, 0x02, 0x11, 0x02, 0x12, 0x00, 0x13, 0x01, 0x14, 0x05, 0x15, 0x07,
    0x16, 0x05, 0x17, 0x07, 0x18, 0x01, 0x19, 0x04, 0x1A, 0x05, 0x1B, 0x0C, 0x1C, 0x2A, 0x1D, 0x01,
    0x1E, 0x00, 0x21, 0x00, 0x22, 0x00, 0x23, 0x00, 0x25, 0x01, 0x26, 0x00, 0x27, 0x39, 0x28, 0x7F,
    0x29, 0x08, 0x30, 0x03, 0x31, 0x00, 0x32, 0x1A, 0x33, 0x1A, 0x34, 0x07, 0x35, 0x07, 0x36, 0x01,
    0x37, 0xFF, 0x38, 0x36, 0x39, 0x07, 0x3A, 0x00, 0x3E, 0xFF, 0x3F, 0x00, 0x40, 0x77, 0x41, 0x40,
    0x42, 0x00, 0x43, 0x30, 0x44, 0xA0, 0x45, 0x5C, 0x46, 0x00, 0x47, 0x00, 0x48, 0x58, 0x4A, 0x1E,
    0x4B, 0x1E, 0x4C, 0x00, 0x4D, 0x00, 0x4E, 0xA0, 0x4F, 0x80, 0x50, 0x00, 0x51, 0x00, 0x52, 0x00,
    0x53, 0x00, 0x54, 0x00, 0x57, 0x80, 0x59, 0x10, 0x5A, 0x08, 0x5B, 0x94, 0x5C, 0xE8, 0x5D, 0x08,
    0x5E, 0x3D, 0x5F, 0x99, 0x60, 0x45, 0x61, 0x40, 0x63, 0x2D, 0x64, 0x02, 0x65, 0x96, 0x66, 0x00,
    0x67, 0x97, 0x68, 0x01, 0x69, 0xCD, 0x6A, 0x01, 0x6B, 0xB0, 0x6C, 0x04, 0x6D, 0x2C, 0x6E, 0x01,
    0x6F, 0x32, 0x71, 0x00, 0x72, 0x01, 0x73, 0x35, 0x74, 0x00, 0x75, 0x33, 0x76, 0x31, 0x77, 0x01,
    0x7C, 0x84, 0x7D, 0x03, 0x7E, 0x01
};

void paj7620WriteReg(uint8_t addr, uint8_t cmd);
uint8_t paj7620ReadReg(uint8_t addr);
void paj7620ReadRegBuf(uint8_t addr, uint8_t *buffer, uint8_t len);
void paj7620SelectBank(uint8_t bank);
void paj7620Init(void);
uint8_t gestureRead(void);
void gestureSuspend(void);
void gestureResume(void);

#define GESTURE_SAMPLE_INTERVAL    100

uint32_t sampleTimerPreviousMillis = 0;

/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x0c
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x0009

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
	uint8_t 	Raw_data_type;
	uint8_t		delay[2];
}packet_raw_t;

typedef struct
{
	uint8_t threshold0[2];
	uint8_t threshold1[2];
	uint8_t flashSave;
}packet_thlsd_t;

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
char *versions = "V21";
uint16_t NodeVersion = 0x6102;
// Decrease BAUD rate for PAJ's I2C.
uint32_t intStart = 0;
uint32_t intEnd = 0;

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
	
	nrgSave.begin(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance

	pinMode(GROVE_LED_PIN_NUM, OUTPUT);
	digitalWrite(GROVE_LED_PIN_NUM, HIGH);
    
    pinMode(GESTURE_INT_PIN_NUM, INPUT_PULLUP);
    pinMode(GESTURE_VLED_PIN_NUM, OUTPUT);
    digitalWrite(GESTURE_VLED_PIN_NUM, HIGH);
    
    pinMode(SOFT_SCL_PIN, OUTPUT);
	pinMode(SOFT_SDA_PIN, OUTPUT);
	digitalWrite(SOFT_SCL_PIN, HIGH);
	digitalWrite(SOFT_SDA_PIN, HIGH);
#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif

    paj7620Init();
	
	Wire.begin(deviceI2CAddress);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);
  
	wwdg.begin();
    
    // Serial.begin(115200);
}

void loop()
{
    uint32_t sampleTimerCurrentMillis = millis();
			
#ifdef BLE_SUPPORT
	if(Wire.isbusidle())PreMillis = sampleTimerCurrentMillis;
	if ((sampleTimerCurrentMillis - PreMillis) > 20)
	{
		Wire.reset();
		Wire.begin(deviceI2CAddress);
		Wire.onReceive(receiveEvent);
		Wire.onRequest(requestEvent);
		PreMillis = sampleTimerCurrentMillis;
	}
#endif
    if(sampleTimerCurrentMillis - sampleTimerPreviousMillis >= GESTURE_SAMPLE_INTERVAL)
    {
        sampleTimerPreviousMillis = sampleTimerCurrentMillis;
        packet_01_data.data.deviceEvent = gestureRead();
        // if(packet_01_data.data.deviceEvent)Serial.println(packet_01_data.data.deviceEvent);
#ifdef BLE_SUPPORT
			if(Core_mode == CORE_BLE_MODE)
			{
				if(packet_01_data.data.deviceEvent > 0)
				{
					InstructionOption.event.Header.type = I2C_CMD_NOTIFY_EVENT;
					InstructionOption.event.Header.Address = deviceI2CAddress;
					InstructionOption.event.Event  = packet_01_data.data.deviceEvent;
					InstructionOption.event.Header.Datalen	 = sizeof(packet_event);
					Wire.MasterGPIOTransmission(ptr2, sizeof(packet_event));
				}
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
					if (ledFlashTimes >= 3)ledFlashCommand = false;
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
				InstructionOption.atr.chipid		= 0;
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

			default:
				commandReceive = I2C_CMD_NULL;
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
			
            gestureSuspend();
            pinMode(SOFT_SCL_PIN, INPUT_PULLUP);
			pinMode(SOFT_SDA_PIN, INPUT_PULLUP);
            digitalWrite(GESTURE_VLED_PIN_NUM, LOW);
            
			wwdg.end();
			Wire.end();
			pinMode(PA9, INPUT_PULLUP);
			pinMode(PA10, INPUT_PULLUP);
			
			nrgSave.standby();
			
			Wire.begin(deviceI2CAddress);
			Wire.onReceive(receiveEvent);
			Wire.onRequest(requestEvent);
			wwdg.begin();
            
            digitalWrite(GESTURE_VLED_PIN_NUM, HIGH);
            pinMode(SOFT_SCL_PIN, OUTPUT);
            pinMode(SOFT_SDA_PIN, OUTPUT);
            gestureResume();
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
void i2cStart(void)
{	
	I2C_SDA_HIGH();
	I2C_WAIT();
	I2C_SCL_HIGH();  
	I2C_WAIT();
	I2C_SDA_LOW();
	I2C_WAIT();
	I2C_SCL_LOW(); 
	I2C_WAIT();
}

void i2cStop(void)
{	
	I2C_SDA_LOW();
	I2C_WAIT();
	I2C_SCL_HIGH();
	I2C_WAIT();
	I2C_SDA_HIGH();
	I2C_WAIT();
}

bool i2cWaitAck(void)
{
	bool ack, ret;
	
	I2C_SDA_IN();

	ack = I2C_SDA_READ();
	
	I2C_SCL_HIGH();  
	I2C_WAIT();
	I2C_SCL_LOW();
	I2C_WAIT();

	I2C_SDA_OUT();
	
	ret = (ack == LOW)?true:false;
	return ret;
}

void i2cAck(void)
{
	I2C_SDA_LOW();  
	I2C_WAIT();
	I2C_SCL_HIGH();  
	I2C_WAIT();
	I2C_SCL_LOW();  
	I2C_WAIT();
}

void i2cNAck(void)
{	
	I2C_SDA_HIGH();  
	I2C_WAIT();
	I2C_SCL_HIGH();  
	I2C_WAIT(); 
	I2C_SCL_LOW();  
	I2C_WAIT();
}

void i2cSendByte(uint8_t txd)
{
	uint8_t i;
	
	for(i = 0; i < 8; i ++)  
	{                
		if(txd & 0x80)I2C_SDA_HIGH();
		else  I2C_SDA_LOW();
		txd <<= 1;
		
		I2C_SCL_HIGH();
		I2C_WAIT();
		I2C_SCL_LOW();   
		I2C_WAIT();
	}
}

uint8_t i2cReadByte(void)
{
	uint8_t i, res = 0;

	I2C_SDA_IN();
	
	for(i = 0; i < 8; i ++)  
	{
		res <<= 1;  
		if(I2C_SDA_READ())res ++;  
		
		I2C_SCL_HIGH();
		I2C_WAIT();
		I2C_SCL_LOW();
		I2C_WAIT(); 
	}           
	
	I2C_SDA_OUT();
	
	return res;  
}

void paj7620WriteReg(uint8_t addr, uint8_t cmd)
{
    i2cStart(); 
	i2cSendByte((PAJ7620_I2C_ADDR << 1) | 0x00);
	i2cWaitAck();
	i2cSendByte(addr); 
	i2cWaitAck();
    i2cSendByte(cmd); 
	i2cWaitAck();
	i2cStop();
}

uint8_t paj7620ReadReg(uint8_t addr)
{
    uint8_t data = 0; 

	i2cStart(); 
	i2cSendByte((PAJ7620_I2C_ADDR << 1) | 0x00);
	i2cWaitAck();
	i2cSendByte(addr); 
	i2cWaitAck();
	i2cStart(); 
	i2cSendByte((PAJ7620_I2C_ADDR << 1) | 0x01);
	i2cWaitAck();
	data = i2cReadByte(); 
	i2cAck();
	i2cStop(); 

	return data;
}

void paj7620ReadRegBuf(uint8_t addr, uint8_t *buffer, uint8_t len)
{
	uint8_t i;

	i2cStart();
	i2cSendByte((PAJ7620_I2C_ADDR << 1) | 0x00);
	i2cWaitAck();
	i2cSendByte(addr);
	i2cWaitAck();
	i2cStart();
	i2cSendByte((PAJ7620_I2C_ADDR << 1) | 0x01);
	i2cWaitAck();
	for(i = 0; i < (len - 1); i++)
	{
		buffer[i] = i2cReadByte();
		i2cAck();
	}
	buffer[i] = i2cReadByte();
	i2cNAck();
	i2cStop();
}

void paj7620SelectBank(uint8_t bank)
{
    if (bank == 0) paj7620WriteReg(0xef, 0);
    else if (bank == 1) paj7620WriteReg(0xef, 1);
}

void paj7620Init(void)
{
    uint8_t temp = 0;
    
    paj7620SelectBank(0);
    temp = paj7620ReadReg(0);
    if(temp == 0x20)
    {
        for(uint16_t i = 0; i < 438; i += 2)
        {
            paj7620WriteReg(initRegisterArray[i], initRegisterArray[i + 1]);
        }
    }
    paj7620SelectBank(0);
    delay(200);
}

uint8_t gestureRead(void)
{
    uint8_t result = 0, data[2] = {0, 0};

    paj7620ReadRegBuf(0x43, data, 2);
    
    switch(data[0]) {
        case 0x01:
            result = Left;
        break;

        case 0x02:
            result = Right;
        break;

        case 0x04:
            result = Down;
        break;

        case 0x08:
            result = Up;
        break;

        case 0x10:
            result = Forward;
        break;

        case 0x20:
            result = Backward;
        break;

        case 0x40:
            result = Clockwise;
        break;

        case 0x80:
            result = Anticlockwise;
        break;

        default:
            if(data[1] == 0x01)result = Wave;
        break;
    }

    return result;
}

void gestureSuspend(void)
{
    paj7620WriteReg(0xEF, 0x01); // switch to bank 1
	paj7620WriteReg(0x72, 0x00); // disable 7620
	paj7620WriteReg(0xEF, 0x00); // switch to bank 0
	paj7620WriteReg(0x03, 0x01); // 
}

void gestureResume(void)
{
    // Read ID 3 times to make sure I2C is active
	paj7620ReadReg(0x00);
	paj7620ReadReg(0x00);
	paj7620ReadReg(0x00);
	
	paj7620WriteReg(0xEF, 0x01); // switch to bank 1
    paj7620WriteReg(0xEF, 0x00);
    paj7620WriteReg(0xEE, 0x03);
    paj7620WriteReg(0xEE, 0x07);
}
