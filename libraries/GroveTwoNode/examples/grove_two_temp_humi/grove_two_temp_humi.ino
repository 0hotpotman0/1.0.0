
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <RTCTimer.h>
#include <WatchDog.h>


#define GROVE_TWO_TX_PIN_NUM	PA2
#define GROVE_TWO_RX_PIN_NUM	PA3

#define TEMP_LED_PIN_NUM		PA1

#define DHT11_DATA_PIN			PA5

void i2cStart(void);
void i2cStop(void);
bool i2cWaitAck(void);
void i2cAck(void);
void i2cNAck(void);
void i2cSendByte(uint8_t txd);
uint8_t i2cReadByte(void);

#define DHT11_DATA_REQUEST		20

#define DHT11_RESPONSE_LENGH	0x2D
#define DHT11_DATA_LOW			0x1C
#define DHT11_DATA_HIGH			0x1B

#define DHT11_ADJUST_VAL		0x05

uint16_t lm75aReadData(uint8_t dataAddr);
void lm75aWriteData(uint8_t dataAddr, uint16_t dataValue);
void lm75aPowerOn(void);
void lm75aPowerOff(void);
int16_t lm75aGetTempData(void);

/***************************************************************

 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x0E
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x0012

#define I2C_DEF_ADDR_FLASH_LOC		0x00 // byte address
#define I2C_CUR_ADDR_FLASH_LOC		0x01 // byte address

#define I2C_CMD_GET_DEV_ID		0x00 //
#define I2C_CMD_GET_DEV_EVENT	0x01 //
#define I2C_CMD_GET_DHT_DATA	0x02 //

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
uint32_t StartMillis = 0, PreMillis = 0, BusPreMillis = 0;
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
		packet_raw_t	raw;
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
	uint8_t		data[4];
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

uint8_t	LightRawData = 0, TempRawData = 0;
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
uint32_t ledFlashPreviousMillis = 0;

/***************************************************************

 ***************************************************************/

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
uint16_t NodeVersion = 0x6102;

uint32_t intStart = 0;
uint32_t intEnd = 0;

uint8_t DhtData[8];
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

	packet_01_data.data.deviceVID = DEVICE_VID;
	packet_01_data.data.devicePID = DEVICE_PID;
	packet_01_data.data.deviceEvent = 0;

	nrgSave.begin(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance

	pinMode(TEMP_LED_PIN_NUM, OUTPUT);
	digitalWrite(TEMP_LED_PIN_NUM, HIGH);

	pinMode(DHT11_DATA_PIN, OUTPUT);
	digitalWrite(DHT11_DATA_PIN, HIGH);

#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif

	Wire.begin(deviceI2CAddress);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);

//	wwdg.begin();
}

void loop()
{
	uint32_t j, CurrentMillis = millis();
			
#ifdef BLE_SUPPORT
	if(Wire.isbusidle())BusPreMillis = CurrentMillis;
	if ((CurrentMillis - BusPreMillis) > 20)
	{
		Wire.reset();
		Wire.begin(deviceI2CAddress);
		Wire.onReceive(receiveEvent);
		Wire.onRequest(requestEvent);
		BusPreMillis = CurrentMillis;
	}
#endif

	if (((CurrentMillis - PreMillis) > 3000) && (LightRawData == 0))
	{
		pinMode(DHT11_DATA_PIN, OUTPUT);
		digitalWrite(DHT11_DATA_PIN, LOW);
		PreMillis = CurrentMillis;
		LightRawData = 1;
	}
	if (((CurrentMillis - PreMillis) > DHT11_DATA_REQUEST) && (LightRawData == 1))
	{
		digitalWrite(DHT11_DATA_PIN, HIGH);
		pinMode(DHT11_DATA_PIN, INPUT);
		while(digitalRead(DHT11_DATA_PIN) == HIGH);
		j = 0;
		while(digitalRead(DHT11_DATA_PIN) == LOW)
		{
			delayMicroseconds(1);
			j++;
		}
		if ((j < (DHT11_RESPONSE_LENGH-DHT11_ADJUST_VAL)) ||
			(j > (DHT11_RESPONSE_LENGH+DHT11_ADJUST_VAL)) )
		{
			pinMode(DHT11_DATA_PIN, OUTPUT);
			digitalWrite(DHT11_DATA_PIN, HIGH);
			LightRawData = 0;
			return;
		}
		j = 0;
		while(digitalRead(DHT11_DATA_PIN) == HIGH)
		{
			delayMicroseconds(1);
			j++;
		}
		if ((j < (DHT11_RESPONSE_LENGH-DHT11_ADJUST_VAL)) ||
			(j > (DHT11_RESPONSE_LENGH+DHT11_ADJUST_VAL)) )
		{
			pinMode(DHT11_DATA_PIN, OUTPUT);
			digitalWrite(DHT11_DATA_PIN, HIGH);
			LightRawData = 0;
			return;
		}

		dht11ReadData();
		
		if (((DhtData[0]+DhtData[1]+DhtData[2]+DhtData[3]-DhtData[4]) & 0xFF) != 0)
		{
			memset(DhtData, 0, 8);
		}else memcpy((uint8_t *) &packet_01_data.data.deviceEvent, DhtData, 4);

		PreMillis = CurrentMillis;
		LightRawData = 0;
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

		if (TempRawData)
		{
			if (CurrentMillis - RawPreviousMillis >= RawDelayMillis)
			{
				
				InstructionOption.raw_data.Header.type = I2C_CMD_GET_RAW_DATA;
				InstructionOption.raw_data.Header.Address = deviceI2CAddress;
				InstructionOption.raw_data.data[0] = DhtData[0];
				InstructionOption.raw_data.data[1] = DhtData[1];
				InstructionOption.raw_data.data[2] = DhtData[2];
				InstructionOption.raw_data.data[3] = DhtData[3];
				InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);
				if(Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw)) == 0)
				{
					ErrorCount++;
					RawPreviousMillis += 10; // Retry in 10 ms later.
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
			ledFlashCommand = false;
			ledFlashStatus = false;
			digitalWrite(TEMP_LED_PIN_NUM, HIGH);

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

//	wwdg.reset();
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
				TempRawData = 1;
				RawDelayMillis = commandOption.data.commands.raw.delay[0]+commandOption.data.commands.raw.delay[1]*256+(chipId[0]&0x03);
				if (RawDelayMillis == 0)
					RawDelayMillis = I2C_CMD_RAW_DATA_TIME;
				RawPreviousMillis = 0;
			}else
				TempRawData = 0;
			commandReceive = I2C_CMD_NULL;
			
			break;
#endif
		case I2C_CMD_GET_DHT_DATA:
			digitalWrite(DHT11_DATA_PIN, LOW);
			PreMillis = millis();
			commandReceive = I2C_CMD_NULL;
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

uint32_t dht11ReadData(void)
{
	uint8_t Data = 0, i, j;

	for(Data=0;Data<5;Data++)
	{
		DhtData[Data] = 0;
		
		for(i=0;i<8;i++)
		{
			j = 0;
			while(digitalRead(DHT11_DATA_PIN) == LOW)
			{
				delayMicroseconds(1);
				j++;
			}
			if ((j < (DHT11_DATA_LOW-DHT11_ADJUST_VAL)) ||
				(j > (DHT11_DATA_LOW+DHT11_ADJUST_VAL)) )
			{
				return 0;
			}

			j = 0;
			while(digitalRead(DHT11_DATA_PIN) == HIGH)
			{
				delayMicroseconds(1);
				j++;
			}
			if (j > (DHT11_DATA_HIGH))
			{
				DhtData[Data] |= (0x80>>i);
			}
		}
	}
	while(digitalRead(DHT11_DATA_PIN) == LOW)
	{
		delayMicroseconds(1);
		j++;
	}
	if ((j < (DHT11_DATA_LOW-DHT11_ADJUST_VAL)) ||
		(j > (DHT11_DATA_LOW+DHT11_ADJUST_VAL)) )
	{
		return 0;
	}
	pinMode(DHT11_DATA_PIN, OUTPUT);
	digitalWrite(DHT11_DATA_PIN, HIGH);
	
	return 1;
}


