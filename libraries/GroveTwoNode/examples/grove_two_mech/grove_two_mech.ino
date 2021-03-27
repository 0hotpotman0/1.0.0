
/*****************************************************************************
 *                                                                           *
 * COPYRIGHT (C) 2018 Chaihuo-edu - All Rights Reserved                      *
 *                                                                           *
 * Chaihuo-edu makes no warranty express or implied including but not        *
 * limited to, any warranty of                                               *
 *                                                                           *
 *                                                                           *
 *  (i)  merchantability or fitness for a particular purpose and/or          *
 *                                                                           *
 *  (ii) requirements, for a particular purpose in relation to the LICENSED  *
 *       MATERIALS, which is provided AS IS, WITH ALL FAULTS. Chaihuo-edu    *
 *       does not represent or warrant that the LICENSED MATERIALS provided  *
 *       here under is free of infringement of any third party patents,      *
 *       copyrights, trade secrets or other intellectual property rights.    *
 *       ALL WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE        *
 *       EXCLUDED TO THE FULLEST EXTENT PERMITTED BY LAW                     *
 *                                                                           *
 *****************************************************************************
 * */

/* codecraft_usr.c -*- mode:C; c-file-style: "eay" -*- */
/*
 * @author Jim XU <xu.baojun@chaihuo.org>
 *
 
 6101 : Fix bug 182: Color led can't display when start bug.
 
 */

#include <Wire.h>
#include <SPI.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>

#define GROVE_TX_PIN_NUM	PA2
#define GROVE_RX_PIN_NUM	PA3

#define GROVE_LED_PIN_NUM		PA1

#ifdef BUTTON_V_3
#define BUTTON_ADDRESS_A		PF0
#define BUTTON_ADDRESS_B		PF1
#endif

/***************************************************************
 Board defines
 ***************************************************************/
#define RGB_POWER_EN_PIN        PB1
#define RGB_POWER_SAMPLE_PIN    PA6
#define RGB_SDI_PIN             PA7
#define RGB_CKI_PIN             PA5
#define MECH_KEY_PIN            PA0

uint32_t powerVoltage = 0;

/***************************************************************

 ***************************************************************/
#define SINGLE_KEY_TIME     50
#define KEY_INTERVAL        30      //  KEY_INTERVAL*10MS    = 300MS
#define LONG_KEY_TIME       2000     //  LONG_KEY_TIME   = 2S

#define KEY_STATE_0         0
#define KEY_STATE_1         1
#define KEY_STATE_2         2
#define KEY_STATE_3         3
#define KEY_STATE_4         4
#define KEY_STATE_5         5

#define N_KEY               0       //  no click
#define S_KEY               1       //  single click
#define L_KEY               2       //   long press

enum mech_key_type_t
{
	MECH_KEY_NO_EVENT = 0,
	MECH_KEY_CLICK = 1,
	MECH_KEY_LONG_PRESS = 2,

	MECH_KEY_MAX = 3
};

uint32_t clickPreviousMillis = 0;
uint8_t buttonDriver(void);
uint8_t buttonRead(void);

uint32_t rgbPreviousMillis = 0;
uint32_t RGB_Millis = 0;

/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x3f
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x000c

#define I2C_DEF_ADDR_FLASH_LOC	0x00
#define I2C_CUR_ADDR_FLASH_LOC	0x01

#define I2C_CMD_GET_DEV_ID		0x00 //
#define I2C_CMD_GET_DEV_EVENT	0x01 //
#define I2C_CMD_COLOR_SET		0x02 //
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

uint8_t Core_mode = 0, ErrorCount=0;
uint32_t StartMillis = 0;

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
	uint8_t		ColorValue[4];
	uint8_t		Time[2];
}packet_color_t;

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
		packet_color_t	color;
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
uint32_t RawPreviousMillis = 0, PreMillis = 0;
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
uint8_t keyPress=0, keyReturn=0;

#define LED_FLASH_TIME	250

bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0, ButtonPressMillis = 0;

bool testFlag = false;
char *versions = "V22";
uint16_t NodeVersion = 0x6104;

uint32_t intStart = 0;
uint32_t intEnd = 0;

#define VOLTAGE_CHECK_TIMEOUT 1000

uint32_t voltagePreviousMillis = 0;

/***************************************************************

 ***************************************************************/
uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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

    nrgSave.begin(GROVE_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance
	
#ifdef BUTTON_V_3
	pinMode(BUTTON_ADDRESS_A, INPUT);
	pinMode(BUTTON_ADDRESS_B, INPUT);
	delay(100);
#endif

    pinMode(GROVE_LED_PIN_NUM, OUTPUT);
    digitalWrite(GROVE_LED_PIN_NUM, HIGH);
	// digitalWrite(GROVE_LED_PIN_NUM, LOW);

    SPI.begin();

    pinMode(RGB_POWER_EN_PIN, OUTPUT);
    digitalWrite(RGB_POWER_EN_PIN, HIGH);

    attachInterrupt(MECH_KEY_PIN, butttonEvent, CHANGE, INPUT);
#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif

#ifdef BUTTON_V_3
// 0x0f/0x1f/0x2f/0x3f.
	deviceI2CAddress -= digitalRead(BUTTON_ADDRESS_A)<<5;
	deviceI2CAddress -= digitalRead(BUTTON_ADDRESS_B)<<4;
#endif

    Wire.begin(deviceI2CAddress);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    wwdg.begin();
    // SerialUSB.begin(115200);
    // SerialUSB.println("start");
	pixelColor(0, 0, 0, 31);
}

void loop()
{
	uint32_t currentMillis = millis();
    mech_key_type_t button_status = MECH_KEY_NO_EVENT;
		
#ifdef BLE_SUPPORT
	if(Wire.isbusidle())PreMillis = currentMillis;
	if ((currentMillis - PreMillis) > 20)
	{
		Wire.reset();
		Wire.begin(deviceI2CAddress);
		Wire.onReceive(receiveEvent);
		Wire.onRequest(requestEvent);
		PreMillis = currentMillis;
	}
#endif
	if(keyPress == 2)
	{
		if ((currentMillis - ButtonPressMillis) >= LONG_KEY_TIME)
		{
			button_status = MECH_KEY_LONG_PRESS;
		}else if ((currentMillis - ButtonPressMillis) >= SINGLE_KEY_TIME)
		{
			button_status = MECH_KEY_CLICK;
		}
		keyPress = 0;
	}else if ((keyPress == 1) && ((currentMillis - ButtonPressMillis) >= LONG_KEY_TIME))
	{// Long pressed, release message, needn't wait for key release.
		button_status = MECH_KEY_LONG_PRESS;
		keyPress = 0;
	}

	if ((button_status != MECH_KEY_NO_EVENT) && (button_status < MECH_KEY_MAX))
	{
		packet_01_data.data.deviceEvent = button_status;
		clickCheckPreviousMillis = millis();

#ifdef BLE_SUPPORT
		if(Core_mode == CORE_BLE_MODE)
		{
			InstructionOption.event.Header.type = I2C_CMD_NOTIFY_EVENT;
			InstructionOption.event.Header.Address = deviceI2CAddress;
			InstructionOption.event.Event  = packet_01_data.data.deviceEvent;
			InstructionOption.event.Header.Datalen	 = sizeof(packet_event);
			Wire.MasterGPIOTransmission(ptr2, sizeof(packet_event));
			packet_01_data.data.deviceEvent = 0;
		}
#endif
	}
	else if(packet_01_data.data.deviceEvent)
	{
		uint32_t clickCheckCurrentMillis = millis();
		if(clickCheckCurrentMillis - clickCheckPreviousMillis >= CLICK_CHECK_TIMEOUT)
		{
			clickCheckPreviousMillis = clickCheckCurrentMillis;
			packet_01_data.data.deviceEvent = MECH_KEY_NO_EVENT;
		}
	}

    if(RGB_Millis > 0)
    {
        uint32_t rgbCurrentMillis = millis();
        if (((rgbCurrentMillis - rgbPreviousMillis) >= RGB_Millis))
        {
            RGB_Millis = 0;
            pixelColor(0, 0, 0, 0);
        }
    }

    uint32_t voltageCurrentMillis = millis();
    if(voltageCurrentMillis - voltagePreviousMillis >= VOLTAGE_CHECK_TIMEOUT)
    {
        voltagePreviousMillis = voltageCurrentMillis;
        powerVoltage = analogRead(RGB_POWER_SAMPLE_PIN) * 3300 * 2 / 1024;
    }

	if(ledFlashCommand)
	{
		uint32_t ledFlashCurrentMillis = millis();
		if(ledFlashCurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
		{
			ledFlashPreviousMillis = ledFlashCurrentMillis;
			digitalWrite(GROVE_LED_PIN_NUM, ledFlashStatus);
			ledFlashStatus = !ledFlashStatus;
			pixelColor(((ledFlashTimes == 0)? 255:0), ((ledFlashTimes == 1)? 255:0), ((ledFlashTimes == 2)? 255:0), 31*ledFlashStatus);
			if(ledFlashTimes < LED_FLASH_COUNT)
			{
				if (!ledFlashStatus)
				{
					ledFlashTimes++;
					if (ledFlashTimes >= LED_FLASH_COUNT)
					{
						ledFlashCommand = false;
						pixelColor(0, 0, 0, 31);
					}
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

		if (LightRawData)
		{
			if (currentMillis - RawPreviousMillis >= RawDelayMillis)
			{
				
				InstructionOption.raw_data.Header.type = I2C_CMD_GET_RAW_DATA;
				InstructionOption.raw_data.Header.Address = deviceI2CAddress;
				InstructionOption.raw_data.data[0] = digitalRead(MECH_KEY_PIN);
				InstructionOption.raw_data.data[1] = 0;
				InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);
				if(Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw)) == 0)
				{
					ErrorCount++;
					RawPreviousMillis += 10; // Retry in 10 ms later.
				}else
				{
					ErrorCount = 0;
					RawPreviousMillis = currentMillis;
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
			case I2C_CMD_COLOR_SET:
				commandReceive = I2C_CMD_NULL;
				rgbPreviousMillis = currentMillis;
				RGB_Millis = commandOption.data.commands.color.Time[0]+commandOption.data.commands.color.Time[1]*256;
				pixelColor (commandOption.data.commands.color.ColorValue[0], 
							commandOption.data.commands.color.ColorValue[1],
							commandOption.data.commands.color.ColorValue[2],
							commandOption.data.commands.color.ColorValue[3]);
			break;

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

            pixelColor(0, 0, 0, 0);

            SPI.end();

			// SPI pin
            // pinMode(RGB_SDI_PIN, INPUT_PULLUP);
            // pinMode(RGB_CKI_PIN, INPUT_PULLUP);
            pinMode(RGB_SDI_PIN, INPUT);
            pinMode(RGB_CKI_PIN, INPUT);

            digitalWrite(RGB_POWER_EN_PIN, LOW);

			wwdg.end();
			Wire.end();
			pinMode(PA9, INPUT_PULLUP);
			pinMode(PA10, INPUT_PULLUP);

			nrgSave.standby();

			Wire.begin(deviceI2CAddress);
			Wire.onReceive(receiveEvent);
			Wire.onRequest(requestEvent);
			wwdg.begin();

            digitalWrite(RGB_POWER_EN_PIN, HIGH);

			// Restart SPI
            SPI.begin();
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

mech_key_type_t buttonEventdetect(void)
{
	mech_key_type_t button_status = MECH_KEY_NO_EVENT;
	uint32_t clickCurrentMillis = millis();

    if(clickCurrentMillis - clickPreviousMillis >= 10)
	{
		clickPreviousMillis = clickCurrentMillis;
		button_status = (mech_key_type_t)buttonRead();
	}

	return button_status;
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

void butttonEvent()
{
	autoSleepPreviousMillis = millis();
	
	if(digitalRead(MECH_KEY_PIN) == HIGH)
	{
		keyPress = 1;
		ButtonPressMillis = autoSleepPreviousMillis;
	}else
	{
		if (keyPress == 1)keyPress = 2;
		// else, it's long key pressed, message was already released.
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
	if (ledFlashTimes < LED_FLASH_COUNT)
	{
		ledFlashTimes = LED_FLASH_COUNT+5;
		ledFlashCommand = false;
	}
	
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

        case I2C_CMD_GET_VOLT:
            Wire.write(powerVoltage & 0xff);
            Wire.write((powerVoltage >> 8) & 0xff);
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
 Device driver
 ***************************************************************/
void pixelColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t level)
{
    if(level > 31)level = 31;

    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);

    SPI.transfer(0xe0 | level);

    SPI.transfer(blue);
    SPI.transfer(green);
    SPI.transfer(red);

    SPI.transfer(0xff);
    SPI.transfer(0xff);
    SPI.transfer(0xff);
    SPI.transfer(0xff);
}

