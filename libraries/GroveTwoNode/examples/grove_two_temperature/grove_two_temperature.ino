
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <RTCTimer.h>
#include <WatchDog.h>


#define GROVE_TWO_TX_PIN_NUM	PA2
#define GROVE_TWO_RX_PIN_NUM	PA3

#define TEMP_LED_PIN_NUM		PA1

#define SOFT_SCL_PIN	PA7
#define SOFT_SDA_PIN	PB1

#define TEMP_TOS_PIN	PA6

#define I2C_DELAY		1
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

#define LM75A_I2C_ADDR	0x44
#define LM75A_TEMP		0x00
#define LM75A_CONF		0x01
#define LM75A_THYST		0x02
#define LM75A_TOS		0x03
#define LM75A_ID		0x07

uint16_t lm75aReadData(uint8_t dataAddr);
void lm75aWriteData(uint8_t dataAddr, uint16_t dataValue);
void lm75aPowerOn(void);
void lm75aPowerOff(void);
int16_t lm75aGetTempData(void);

/***************************************************************

 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x03
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x0003

#define I2C_DEF_ADDR_FLASH_LOC		0x00 // byte address
#define I2C_CUR_ADDR_FLASH_LOC		0x01 // byte address
#define I2C_THD_0_ADDR_FLASH_LOC	0x01 // int address
#define I2C_THD_1_ADDR_FLASH_LOC	0x02 // int address

#define I2C_CMD_GET_DEV_ID		0x00 // 
#define I2C_CMD_GET_DEV_EVENT	0x01 // 
#define I2C_CMD_GET_TEMP		0x02 // 
#define I2C_CMD_SET_THD			0x03 // 
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

#define I2C_CMD_BLE_SET_THD	0x90

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

void requestEvent(void);
void receiveEvent(int howMany);

/***************************************************************

 ***************************************************************/
LowPower nrgSave;

#define AUTO_SLEEP_TIMEOUT	2000

uint32_t autoSleepPreviousMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

/***************************************************************

 ***************************************************************/
#define LED_FLASH_TIME	250

bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0, PreMillis = 0;

/***************************************************************

 ***************************************************************/
#define TEMP_THD_0_NUM	10
#define TEMP_THD_1_NUM	40

int16_t tempThd0 = TEMP_THD_0_NUM;
int16_t tempThd1 = TEMP_THD_1_NUM;

/***************************************************************

 ***************************************************************/
#define TEMP_MIN_CONVERT_TIME	150
#define TEMP_SAMPLE_PERIOD		2000

uint16_t tempSamplePeriod = TEMP_SAMPLE_PERIOD;
bool timeoutFlag = false;
bool sampleFlag = false;
int16_t curTempData = 0;

bool testFlag = false;
char *versions = "V20";
uint16_t NodeVersion = 0x6101;

uint32_t intStart = 0;
uint32_t intEnd = 0;

/***************************************************************

 ***************************************************************/
uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/***************************************************************

 ***************************************************************/
bool flashSave = false;

/***************************************************************

 ***************************************************************/
void setup()
{
	uint8_t i2cDefaultAddr = Flash.read8(I2C_DEF_ADDR_FLASH_LOC); 
	uint8_t i2cCurrentAddr = Flash.read8(I2C_CUR_ADDR_FLASH_LOC);
    
    uint8_t *ptr2 = (uint8_t *)Flash.getChipUniqueID();
    for(uint8_t i = 0; i < 12; i ++)chipId[i] = *(ptr2 + i);
	
	if(i2cDefaultAddr == 0xff)Flash.write8(I2C_DEF_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	if(i2cCurrentAddr == 0xff)Flash.write8(I2C_CUR_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	else deviceI2CAddress = i2cCurrentAddr;
	
	uint16_t tempThd = Flash.read16(I2C_THD_0_ADDR_FLASH_LOC);
	if(tempThd == 0xffff)Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, TEMP_THD_0_NUM);
	else tempThd0 = (int16_t)tempThd;
	
	tempThd = Flash.read16(I2C_THD_1_ADDR_FLASH_LOC);
	if(tempThd == 0xffff)Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, TEMP_THD_1_NUM);
	else tempThd1 = (int16_t)tempThd;
	
	packet_01_data.data.deviceVID = DEVICE_VID;
	packet_01_data.data.devicePID = DEVICE_PID;
	packet_01_data.data.deviceEvent = 0;
	
	nrgSave.begin(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance

	pinMode(TEMP_LED_PIN_NUM, OUTPUT);
	digitalWrite(TEMP_LED_PIN_NUM, HIGH);
	
	pinMode(TEMP_TOS_PIN, INPUT_PULLUP);
	
	pinMode(SOFT_SCL_PIN, OUTPUT);
	pinMode(SOFT_SDA_PIN, OUTPUT);
	digitalWrite(SOFT_SCL_PIN, HIGH);
	digitalWrite(SOFT_SDA_PIN, HIGH);
	
	lm75aPowerOn();
	
	RTCTimer.begin(TEMP_MIN_CONVERT_TIME, dummySample);

#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif
	
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

	if(timeoutFlag)
	{
		timeoutFlag = false;
		
		pinMode(SOFT_SCL_PIN, OUTPUT);
		pinMode(SOFT_SDA_PIN, OUTPUT);
		
		if(sampleFlag)
		{
			curTempData = lm75aGetTempData();
			lm75aPowerOff();
			RTCTimer.setNewPeriod(tempSamplePeriod);
			
			if(curTempData < tempThd0)packet_01_data.data.deviceEvent = 1;
			else if(curTempData >= tempThd0 && curTempData < tempThd1)packet_01_data.data.deviceEvent = 2;
			else if(curTempData >= tempThd1)packet_01_data.data.deviceEvent = 3;
#ifdef BLE_SUPPORT
			if(Core_mode == CORE_BLE_MODE)
			{
				if(preEvent != packet_01_data.data.deviceEvent)
				{
					InstructionOption.event.Header.type = I2C_CMD_NOTIFY_EVENT;
					InstructionOption.event.Header.Address = deviceI2CAddress;
					InstructionOption.event.Event  = packet_01_data.data.deviceEvent;
					InstructionOption.event.Header.Datalen	 = sizeof(packet_event);
					Wire.MasterGPIOTransmission(ptr2, sizeof(packet_event));
					preEvent = packet_01_data.data.deviceEvent;
				}
			}
#endif
		}
		else
		{
			lm75aPowerOn();
			RTCTimer.setNewPeriod(TEMP_MIN_CONVERT_TIME);
		}	
	}

	if(ledFlashCommand)
	{
		uint32_t ledFlashCurrentMillis = millis();
		if(ledFlashCurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
		{
			ledFlashPreviousMillis = ledFlashCurrentMillis;
			digitalWrite(TEMP_LED_PIN_NUM, ledFlashStatus);
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
	{
		uint32_t CurrentMillis = millis();

		if (LightRawData)
		{
			if (CurrentMillis - RawPreviousMillis >= RawDelayMillis)
			{
				InstructionOption.raw_data.Header.type = I2C_CMD_GET_RAW_DATA;
				InstructionOption.raw_data.Header.Address = deviceI2CAddress;
				InstructionOption.raw_data.data[0] = curTempData&0xFF;
				InstructionOption.raw_data.data[1] = (curTempData>>8)&0xFF;
				InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);
				// Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw));
				if(Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw)) == 0)
				{
					ErrorCount++;
					RawPreviousMillis += 3; // Retry in 3 ms later.
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
		switch(commandReceive)
		{
			case I2C_CMD_NOTIFY_ATR:
				InstructionOption.atr.Header.type = I2C_CMD_ATR;
				InstructionOption.atr.pid[0] = DEVICE_PID&0xff;
				InstructionOption.atr.pid[1] = (DEVICE_PID>>8 ) & 0xff;
				InstructionOption.atr.Header.Address = deviceI2CAddress;
				InstructionOption.atr.chipid = chipId[0];
				InstructionOption.atr.hwaddress = DEVICE_I2C_ADDRESS;
				InstructionOption.atr.version[0] = versions[0];
				InstructionOption.atr.version[1] = versions[1];
				InstructionOption.atr.version[2] = versions[2];
				InstructionOption.atr.option[0] = NodeVersion&0xFF;
				InstructionOption.atr.option[1] = (NodeVersion>>8)&0xFF;
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
			case I2C_CMD_BLE_SET_THD:
				{
					tempThd0 = (int16_t)commandOption.data.commands.thread.threshold0[0] + 
										commandOption.data.commands.thread.threshold0[1] * 256;
					tempThd1 = (int16_t)commandOption.data.commands.thread.threshold1[0] + 
										commandOption.data.commands.thread.threshold1[1] * 256;
					if(commandOption.data.commands.thread.flashSave>0)
					{
						Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, tempThd0);
						Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, tempThd1);
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
	else if(commandReceive == I2C_CMD_SET_THD) // set new threshold
	{
		commandReceive = I2C_CMD_NULL;
        if(flashSave)
        {
            Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, tempThd0);
            Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, tempThd1);
        }
	}
#ifdef BLE_SUPPORT
	}
#endif
	
	if(autoSleepFlag)
	{
		uint32_t autoSleepCurrentMillis = millis();
		if((autoSleepCurrentMillis - autoSleepPreviousMillis) > AUTO_SLEEP_TIMEOUT)
		{
			ledFlashCommand = false;
			ledFlashStatus = false;
			digitalWrite(TEMP_LED_PIN_NUM, HIGH);
			
			pinMode(SOFT_SCL_PIN, INPUT_PULLUP);
			pinMode(SOFT_SDA_PIN, INPUT_PULLUP);
			
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

void dummySample(void)
{
	timeoutFlag = true;
	sampleFlag = !sampleFlag;
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
		case I2C_CMD_SET_THD:
			{
				int16_t tempThd = (int16_t)receiveBuffer[2] + receiveBuffer[3] * 256;
				if(receiveBuffer[1] == 0)tempThd0 = tempThd;
				else if(receiveBuffer[1] == 1)tempThd1 = tempThd;
                flashSave = receiveBuffer[4];
			}
		break;
		
		case I2C_CMD_LED_ON:
			ledFlashCommand = true;
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_LED_OFF:
			ledFlashCommand = false;
			ledFlashStatus = false;
			digitalWrite(TEMP_LED_PIN_NUM, HIGH);
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
#ifdef BLE_SUPPORT
			preEvent = 0;
#endif
		break;
		
		case I2C_CMD_GET_TEMP:
        {
			Wire.write((uint8_t)(((uint16_t)curTempData) & 0xff));
			Wire.write((uint8_t)(((uint16_t)curTempData) >> 8));
			commandReceive = I2C_CMD_NULL;
        }   
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

uint16_t lm75aReadData(uint8_t dataAddr)
{
	uint16_t Data = 0; 

	i2cStart(); 
	i2cSendByte((LM75A_I2C_ADDR << 1) | 0x00);
	i2cWaitAck();
	i2cSendByte(dataAddr); 
	i2cWaitAck();
	i2cStart(); 
	i2cSendByte((LM75A_I2C_ADDR << 1) | 0x01);
	i2cWaitAck();
	switch(dataAddr)
	{ 
		case LM75A_TEMP:
		case LM75A_THYST:
		case LM75A_TOS:
			Data = i2cReadByte(); 
			i2cAck();
			Data <<= 8;
			Data |= i2cReadByte();
			i2cNAck();
		break;

		case LM75A_CONF:
			Data = i2cReadByte(); 
			i2cNAck();	
		break;

		default:
		break;
	}
	i2cStop(); 

	return Data;
}

void lm75aWriteData(uint8_t dataAddr, uint16_t dataValue)
{
	i2cStart(); 
	i2cSendByte((LM75A_I2C_ADDR << 1) | 0x00);
	i2cWaitAck();
	i2cSendByte(dataAddr); 
	i2cWaitAck();

	switch(dataAddr)
	{
		case LM75A_THYST:
		case LM75A_TOS:
			i2cSendByte((dataValue >> 1) & 0xff); 
			i2cWaitAck();
			i2cSendByte(((dataValue & 0x01) << 7) & 0xff); 
			i2cWaitAck();
		break;

		case LM75A_CONF:
			i2cSendByte((dataValue) & 0xff); 
			i2cWaitAck();
		break;

		default:
		break;
	}
	i2cStop(); 
}

void lm75aPowerOn(void)
{
	lm75aWriteData(LM75A_CONF,0x00);
}

void lm75aPowerOff(void)
{
	lm75aWriteData(LM75A_CONF,0x01);
}

int16_t lm75aGetTempData(void)
{
	uint16_t data = 0;

	data = lm75aReadData(LM75A_TEMP);
	data >>= 5;
	if(data & 0x0400)data |= 0xf800;
	return ((int16_t)(data)) / 8;
}
