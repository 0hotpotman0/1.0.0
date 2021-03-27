#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>

#define GROVE_LED_PIN_NUM		PA1

/***************************************************************
 Board defines
 ***************************************************************/

/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x02
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x000f

#define I2C_DEF_ADDR_FLASH_LOC		0x00 // byte address
#define I2C_CUR_ADDR_FLASH_LOC		0x01 // byte address
#define I2C_THD_0_ADDR_FLASH_LOC	0x01 // int address
#define I2C_THD_1_ADDR_FLASH_LOC	0x02 // int address

#define I2C_CMD_GET_DEV_ID		0x00 //
#define I2C_CMD_GET_DEV_EVENT	0x01 //
#define I2C_CMD_GET_DISTANCE    0x02 //
#define I2C_CMD_SET_THD			0x03 //

#define I2C_CMD_SET_SERVO_ANGLE 0x05 // set angle of servo

#define I2C_CMD_LED_ON			0xb0 //
#define I2C_CMD_LED_OFF			0xb1 //
#define I2C_CMD_AUTO_SLEEP_ON	0xb2 //
#define I2C_CMD_AUTO_SLEEP_OFF	0xb3 //
#define I2C_CMD_SET_ADDR		0xc0 //
#define I2C_CMD_RST_ADDR		0xc1 //
#define I2C_CMD_TEST_TX_RX_ON   0xe0 //
#define I2C_CMD_TEST_TX_RX_OFF  0xe1 //
#define I2C_CMD_TEST_GET_VER    0xe2 //
#define I2C_CMD_GET_VOLT        0xe3 //
#define I2C_CMD_JUMP_TO_BOOT	0xf0 //
#define I2C_CMD_GET_DEVICE_UID  0xf1 //
#define I2C_CMD_NULL			0xff //

uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS;
uint8_t commandReceive = I2C_CMD_NULL;
uint8_t	ledFlashTimes = 0;

#ifdef BLE_SUPPORT

#define I2C_CMD_BLE_SET_SERVO_ANGLE		0x90

uint8_t Core_mode = 0;
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
	uint8_t 	angle;
}packet_angle;

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
		packet_angle	angle;
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
	uint8_t		data[12];
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

/***************************************************************

 ***************************************************************/
#define DISTANCE_THD_0_NUM	100
#define DISTANCE_THD_1_NUM	300

uint16_t distThd0 = DISTANCE_THD_0_NUM;
uint16_t distThd1 = DISTANCE_THD_1_NUM;

/***************************************************************

 ***************************************************************/
#define LED_FLASH_TIME	250

bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;

char *versions = "V20";

uint16_t NodeVersion = 0x6100;

uint32_t PreDataMillis = 0;

/***************************************************************

 ***************************************************************/
uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t rawget = 0, rawput = 0, rawbuf[256] = {0,};

/***************************************************************

 ***************************************************************/
bool flashSave = false;

/***************************************************************
 Device initialization
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

    pinMode(GROVE_LED_PIN_NUM, OUTPUT);
    digitalWrite(GROVE_LED_PIN_NUM, LOW);
	delay(20);

#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif

    Wire.begin(deviceI2CAddress);
    Wire.onReceive(receiveEvent);

    wwdg.begin();
    
	Serial.begin(115200);
}

void loop()
{

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
			Core_mode = CORE_BLE_MODE;
		}

	}
	while (Serial.available())
	{
		rawbuf[rawput++] = Serial.read();
		PreDataMillis = millis();
	}
	if(Core_mode == CORE_BLE_MODE)
	{
		uint32_t CurrentMillis = millis();

		if ((rawget != rawput) && ((CurrentMillis - PreDataMillis) > 5))
		{
			LightRawData = 0;
			InstructionOption.raw_data.Header.type = I2C_CMD_GET_RAW_DATA;
			InstructionOption.raw_data.Header.Address = deviceI2CAddress;
			while((rawget != rawput) && (LightRawData < 12))
			{
				InstructionOption.raw_data.data[LightRawData++] = rawbuf[rawget++];
			}
			InstructionOption.raw_data.Header.Datalen = 4+LightRawData;
			Wire.MasterGPIOTransmission(ptr2, InstructionOption.raw_data.Header.Datalen);
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
				InstructionOption.atr.option[0] = 0;
				InstructionOption.atr.option[1] = 0;
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
	}
#endif

    wwdg.reset();
}

void receiveEvent(int howMany)
{
    uint8_t i = 0, j = 0, receiveBuffer[MAINBOARD_BLE_I2C_DATALEN] = {0,};
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
		for (j=4;j<i;j++)Serial.write(receiveBuffer[j]);
		commandOption.data.Header.Datalen -= MAINBOARD_BLE_COMMAND_LOW;
		if (i != commandOption.data.Header.Datalen)
		{// Bus error!!!
			return;
		}
		commandReceive = commandOption.data.Header.type;
	}
#endif

}


