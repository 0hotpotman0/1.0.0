
#include <Wire.h>
#include <SPI.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>

#define DOUBLE_KEY_SUPPORT

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
#define D_KEY               2       //  double click
#define L_KEY               3      //   long press
#define T_KEY				4     	// three click

enum mech_key_type_t
{
	MECH_KEY_NO_EVENT = 0,
	MECH_KEY_CLICK = 1,
	MECH_KEY_DOUBLE_CLICK = 2,
	MECH_KEY_LONG_PRESS = 3,
	MECH_KEY_THREE_CLICK = 4,
};


uint32_t clickPreviousMillis = 0;
uint8_t buttonDriver(void);
uint8_t buttonRead(void);

#define RGB_TIMEOUT	500

uint32_t rgbPreviousMillis = 0;
bool rgbFlag = false;

/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x0f
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x000c

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
#define I2C_CMD_GET_VOLT        0xe3 //
#define I2C_CMD_JUMP_TO_BOOT	0xf0 //
#define I2C_CMD_GET_DEVICE_UID  0xf1 //
#define I2C_CMD_NULL			0xff //

uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS;
uint8_t commandReceive = I2C_CMD_NULL;
uint8_t	ledFlashTimes = 0;

#ifdef BLE_SUPPORT

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

bool ledFlashCommand = false;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;

bool testFlag = false;
char *versions = "V20";

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

    pinMode(GROVE_LED_PIN_NUM, OUTPUT);
    digitalWrite(GROVE_LED_PIN_NUM, HIGH);
	// digitalWrite(GROVE_LED_PIN_NUM, LOW);

    SPI.begin();

    pinMode(RGB_POWER_EN_PIN, OUTPUT);
    digitalWrite(RGB_POWER_EN_PIN, HIGH);

    attachInterrupt(MECH_KEY_PIN, butttonEvent, RISING, INPUT);
#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif

    Wire.begin(deviceI2CAddress);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    wwdg.begin();
    // SerialUSB.begin(115200);
    // SerialUSB.println("start");
	pixelColor(0, 0, 0, 32);
}

void loop()
{
    mech_key_type_t button_status = buttonEventdetect();

	if(button_status != MECH_KEY_NO_EVENT)
	{
		packet_01_data.data.deviceEvent = button_status;
		clickCheckPreviousMillis = millis();

        rgbPreviousMillis = clickCheckPreviousMillis;
        rgbFlag = true;

        switch(button_status)
        {
            case MECH_KEY_CLICK:
                pixelColor(0, 255, 0, 32);
            break;

            case MECH_KEY_DOUBLE_CLICK:
                pixelColor(0, 0, 255, 32);
            break;

            case MECH_KEY_LONG_PRESS:
                pixelColor(255, 0, 255, 32);
			break;

			case MECH_KEY_THREE_CLICK:
				pixelColor(255, 255, 0, 32);
			break;

            default:
            break;
        }
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
	else if(packet_01_data.data.deviceEvent)
	{
		uint32_t clickCheckCurrentMillis = millis();
		if(clickCheckCurrentMillis - clickCheckPreviousMillis >= CLICK_CHECK_TIMEOUT)
		{
			clickCheckPreviousMillis = clickCheckCurrentMillis;
			packet_01_data.data.deviceEvent = MECH_KEY_NO_EVENT;
			preEvent = packet_01_data.data.deviceEvent;
		}
	}

    if(rgbFlag)
    {
        uint32_t rgbCurrentMillis = millis();
        if(rgbCurrentMillis - rgbPreviousMillis >= RGB_TIMEOUT)
        {
            rgbFlag = false;
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
			if(ledFlashTimes < LED_FLASH_COUNT)
			{
				if(ledFlashStatus)
				{
					ledFlashTimes++;
					pixelColor(((ledFlashTimes == 1)? 255:0), ((ledFlashTimes == 2)? 255:0), ((ledFlashTimes == 3)? 255:0), 31);
				}else{
					if(ledFlashTimes >= LED_FLASH_COUNT)
					{
						ledFlashCommand = false;
					}
					pixelColor(0, 0, 0, 31);
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
				InstructionOption.atr.option[0]		= 0;
				InstructionOption.atr.option[1]		= 0;
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

uint8_t buttonDriver(void)
{
	static uint8_t keyState = 0;
	static uint16_t keyTime = 0;
	uint8_t keyPress, keyReturn;

	keyReturn = N_KEY;

	if(digitalRead(MECH_KEY_PIN))keyPress = false;
	else keyPress = true;

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
					keyReturn = L_KEY;
					keyState = KEY_STATE_3;
				}
			}
		break;

		case KEY_STATE_3:
			if(keyPress)
			{
				keyState = KEY_STATE_0;
			}
			else
			{
				keyReturn = L_KEY;
			}
		break;

		default:
			keyState = KEY_STATE_0;
		break;
	}
	// SerialUSB.println(keyReturn);
	return  keyReturn;
}

uint8_t buttonRead(void)
{
	static uint8_t keyState1 = 0, keyTime1 = 0, keyTime2 = 0;
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
				keyTime2 = 0;
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
				// keyReturn = D_KEY;
				keyState1 = KEY_STATE_2;
			}
			else
			{
				keyTime1 ++;
				if(keyTime1 >= KEY_INTERVAL)
				{
					keyReturn = S_KEY;
					keyState1 = KEY_STATE_0;
				}
			}
		break;

		case KEY_STATE_2:
			if (keyTemp == S_KEY)
			{
				keyReturn = T_KEY;
				keyState1 = KEY_STATE_0;
			}
			else
			{
				keyTime2 ++;
				if(keyTime2 >= KEY_INTERVAL)
				{
					keyReturn = D_KEY;
					keyState1 = KEY_STATE_0;
				}
			}
		break;

		default:
			keyState1 = KEY_STATE_0;
		break;
	}
#else
    keyReturn = keyTemp;
#endif

	return keyReturn;
}
