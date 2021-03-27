
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>
#include "Adafruit_NeoPixel.h"


#define GROVE_TWO_TX_PIN_NUM	PA2
#define GROVE_TWO_RX_PIN_NUM	PA3

#define GROVE_LED_PIN_NUM		PA1

/***************************************************************
 Board defines
 ***************************************************************/
#define PIXELS_POWER_PIN    PB1
#define PIXELS_PIN  PA0
#define PIXELS_NUM  1
#define PIXELS_POWER_SAMPLE_PIN    PA6

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(PIXELS_NUM, PIXELS_PIN, NEO_GRB + NEO_KHZ800);

uint32_t displayTimer = 0;
uint32_t displayTimerPreviousMillis = 0;
uint8_t displayMode = 0;
uint8_t displayCount = 0;
int8_t displayCountInt = 0;
bool displayFlashFlag = false;
uint32_t displayIntervalTimerPreviousMillis = 0;
uint16_t red = 0, green = 0, blue = 0;
uint32_t now_colors = 0;


/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x0b
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x8004

#define I2C_DEF_ADDR_FLASH_LOC	0x00
#define I2C_CUR_ADDR_FLASH_LOC	0x01

#define I2C_CMD_GET_DEV_ID		0x00
#define I2C_CMD_DISPLAY_ALWAY       0x01
#define I2C_CMD_DISPLAY_BLINK       0x02
#define I2C_CMD_DISPLAY_BREATHE     0x03
#define I2C_CMD_DISPLAY_RAINNBOW    0x04
#define I2C_CMD_DISPLAY_FADE        0x05
#define I2C_CMD_DISPLAY_RANDOM      0x06
#define I2C_CMD_SET_BRIGHTNESS      0x07
#define I2C_CMD_DISPLAY_STOP        0x08

#define I2C_CMD_LED_ON			0xb0 //
#define I2C_CMD_LED_OFF			0xb1 //
#define I2C_CMD_AUTO_SLEEP_ON	0xb2 //
#define I2C_CMD_AUTO_SLEEP_OFF	0xb3 //
#define I2C_CMD_SET_ADDR		0xc0 //
#define I2C_CMD_RST_ADDR		0xc1 //
#define I2C_CMD_TEST_TX_RX_ON   0xe0 //
#define I2C_CMD_TEST_TX_RX_OFF  0xe1 //
#define I2C_CMD_TEST_GET_VER    0xe2 //
#define I2C_CMD_GET_VOLT        0xe3 
#define I2C_CMD_JUMP_TO_BOOT	0xf0 //
#define I2C_CMD_GET_DEVICE_UID  0xf1 // 
#define I2C_CMD_NULL			0xff //

uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS;
uint8_t commandReceive = I2C_CMD_NULL;
uint8_t	ledFlashTimes = 0;


#ifdef BLE_SUPPORT

#define I2C_CMD_BLE_DISPLAY_ALWAY		0x90
#define I2C_CMD_BLE_DISPLAY_BLINK		0x91
#define I2C_CMD_BLE_DISPLAY_BREATHE		0x92
#define I2C_CMD_BLE_DISPLAY_RAINNBOW	0x93
#define I2C_CMD_BLE_DISPLAY_FADE		0x94
#define I2C_CMD_BLE_DISPLAY_RANDOM      0x95
#define I2C_CMD_BLE_SET_BRIGHTNESS      0x96
#define I2C_CMD_BLE_DISPLAY_STOP        0x97

#define I2C_CMD_LOW_PRIORITY			0x90

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
	packet_header_t	Header;
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
		packet_got_atr			atr;
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



uint32_t clickCheckPreviousMillis = 0, PreMillis=0;
uint32_t autoSleepPreviousMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

#define LED_FLASH_TIME	250

bool ledFlashCommand = true;
// bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;

bool testFlag = false;
char *versions = "V20";
uint16_t NodeVersion = 0x6100;

uint32_t intStart = 0;
uint32_t intEnd = 0;

uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define VOLTAGE_CHECK_TIMEOUT 1000

uint32_t voltagePreviousMillis = 0;
uint32_t powerVoltage = 0;

uint32_t test_millis = 0;
bool displayForeverFlag = false;
bool displayPriorityFlag = false;

uint16_t blinkInterval_ms = 250;
uint16_t breatheInterval_ms = 20;
uint16_t fadeInterval_ms = 20;
uint16_t rainbowInterval_ms = 20;
int16_t fade_green_slope, fade_red_slope, fade_blue_slope;
static bool show_flag = false;

/***************************************************************
 Device initialization
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
	packet_01_data.data.deviceEvent = 0;

    // Attention: mainboard send wakeup() will cause the rgb led sequence chaos
	nrgSave.begin(GROVE_TWO_RX_PIN_NUM, NULL, CHANGE); // The pin need pull up by a resistance
#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif

    pixelInit();

	Wire.begin(deviceI2CAddress);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);

    wwdg.begin();

	pixelDisplayRainbow(20, 1500);
}

void loop()
{
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
    if (show_flag) 
    {
        // EXTI_ClearITPendingBit(1<<3);
        // NVIC_DisableIRQ(EXTI2_3_IRQn);
        pixel.show();
        show_flag = false;

        // NVIC_EnableIRQ(EXTI2_3_IRQn);
    }

    if(displayMode == 1){
        // for safity
        delay(1);
        pixelDisplaySingle((uint8_t)(now_colors>>16), (uint8_t)(now_colors>>8), (uint8_t)(now_colors>>0));
        displayMode = 0;     
    } // pixel display alway
    else if(displayMode == 2) // pixel display flash
    {
        uint32_t displayIntervalTimerCurrentMillis = millis();
        if(displayIntervalTimerCurrentMillis - displayIntervalTimerPreviousMillis >= blinkInterval_ms)
        {
            displayIntervalTimerPreviousMillis = displayIntervalTimerCurrentMillis;
            displayFlashFlag = !displayFlashFlag;
            if(displayFlashFlag)pixelDisplaySingle(red, green, blue);
            else pixelDisplaySingle(0, 0, 0);
        }
    }
    else if(displayMode == 3) // pixel display breathe
    {
        uint32_t displayIntervalTimerCurrentMillis = millis();
        if(displayIntervalTimerCurrentMillis - displayIntervalTimerPreviousMillis >= breatheInterval_ms)
        {
            displayIntervalTimerPreviousMillis = displayIntervalTimerCurrentMillis;

            pixelDisplaySingle((red * displayCount) / 100, (green * displayCount) / 100, (blue * displayCount) / 100);
            if(displayFlashFlag)
            {
                displayCount ++;
                if (displayCount >= 100) displayFlashFlag = false;
            }
            else
            {
                displayCount --;
                if (displayCount == 0) displayFlashFlag = true;
            }
        }
    }
    else if(displayMode == 4) // pixel display rainbow
    {
        uint32_t displayIntervalTimerCurrentMillis = millis();
        if(displayIntervalTimerCurrentMillis - displayIntervalTimerPreviousMillis >= rainbowInterval_ms)
        {
            displayIntervalTimerPreviousMillis = displayIntervalTimerCurrentMillis;
            uint32_t c = wheel((displayCount ++) & 255);
            pixelDisplaySingle((uint8_t)(c>>16), (uint8_t)(c>>8), (uint8_t)(c>>0));
            if(displayCount >= 256)displayCount = 0;
        }
    }
    else if(displayMode == 5) // pixel display fade
    {
        uint32_t displayIntervalTimerCurrentMillis = millis();
        uint8_t r,g,b;
        if(displayIntervalTimerCurrentMillis - displayIntervalTimerPreviousMillis >= fadeInterval_ms)
        {
            displayIntervalTimerPreviousMillis = displayIntervalTimerCurrentMillis;
            r = (red*100+fade_red_slope * limiter(displayCountInt))/100;
            g = (green*100+fade_green_slope * limiter(displayCountInt))/100;
            b = (blue*100+fade_blue_slope * limiter(displayCountInt))/100;
            pixelDisplaySingle(r, g, b);
            displayCountInt ++;
            if(displayCountInt >= 110){
                displayCountInt = -10;
                // displayMode = 0;
                if (displayTimer == 0) displayForeverFlag = true;
                else 
                {
                    displayForeverFlag = false;
                    displayMode = 0;
                }
            }
        }
    }
    if ((displayTimer != 0) && (displayForeverFlag == false))
    {
        uint32_t displayTimerCurrentMillis = millis();
        if(displayTimerCurrentMillis - displayTimerPreviousMillis >= displayTimer)
        {
            displayMode = 0;
            displayTimer = 0;
			displayPriorityFlag = false;
            pixelDisplaySingle(0, 0, 0);
            // to save power
            digitalWrite(PIXELS_POWER_PIN, LOW);
        }
    }


    uint32_t voltageCurrentMillis = millis();
    if(voltageCurrentMillis - voltagePreviousMillis >= VOLTAGE_CHECK_TIMEOUT)
    {
        voltagePreviousMillis = voltageCurrentMillis;
        powerVoltage = analogRead(PIXELS_POWER_SAMPLE_PIN) * 3300 * 2 / 1024;
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
			case  I2C_CMD_GET_RAW_DATA:
			// Raw data request, send powerVoltage.
				InstructionOption.raw_data.Header.type		= I2C_CMD_GET_RAW_DATA;
				InstructionOption.raw_data.Header.Address	= deviceI2CAddress;
				InstructionOption.raw_data.data[0]	= powerVoltage&0xFF;
				InstructionOption.raw_data.data[1]	= (powerVoltage>>8)&0xFF;
				InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);
				Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw));
				commandReceive = I2C_CMD_NULL;
				
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

            digitalWrite(PIXELS_POWER_PIN, HIGH);
            pixelDisplaySingle(0, 0, 0);
            digitalWrite(PIXELS_POWER_PIN, LOW);
            digitalWrite(PIXELS_PIN, LOW);

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
	uint8_t i = 0, j, receiveBuffer[MAINBOARD_BLE_I2C_DATALEN] = {0,};
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
		if((commandReceive & 0xF0) == I2C_CMD_LOW_PRIORITY)
		{// Low priority.
			if (displayPriorityFlag)
			{	// High priority task is working.
				commandReceive = I2C_CMD_NULL;
				return;
			}
			commandReceive &= 0x0F;
			commandReceive++;
		}else if(commandReceive < 0x0A)
		{
			commandReceive++;
			displayPriorityFlag = true;			// High priority
		}
		for(j=0;j<(i-sizeof(packet_header_t));j++)
		{
			receiveBuffer[j+1] = receiveBuffer[j+sizeof(packet_header_t)];
		}
		i -= (sizeof(packet_header_t)-1);
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
        case I2C_CMD_DISPLAY_ALWAY:
            pixelDisplayOneColor(receiveBuffer[1], receiveBuffer[2], receiveBuffer[3], receiveBuffer[4] + receiveBuffer[5] * 256);
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_DISPLAY_BLINK:
            pixelDisplayBlink(receiveBuffer[1], receiveBuffer[2], receiveBuffer[3], receiveBuffer[4] + receiveBuffer[5] * 256, receiveBuffer[6] + receiveBuffer[7] * 256);
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_DISPLAY_BREATHE:
            pixelDisplayBreathe(receiveBuffer[1], receiveBuffer[2], receiveBuffer[3], receiveBuffer[4] + receiveBuffer[5] * 256, receiveBuffer[6] + receiveBuffer[7] * 256);
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_DISPLAY_RAINNBOW:
            pixelDisplayRainbow(receiveBuffer[1] + receiveBuffer[2] * 256, receiveBuffer[3] + receiveBuffer[4] * 256);
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_DISPLAY_FADE:
            pixelDisplayFade(receiveBuffer[1], receiveBuffer[2], receiveBuffer[3], receiveBuffer[4], receiveBuffer[5], receiveBuffer[6], receiveBuffer[7]+receiveBuffer[8]*256, receiveBuffer[9]+receiveBuffer[10]*256);
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_DISPLAY_STOP:
            displayMode = 0;
            displayTimer = 0;
			displayPriorityFlag = false;
            digitalWrite(PIXELS_POWER_PIN, HIGH);
            pixelDisplaySingle(0, 0, 0);
            digitalWrite(PIXELS_POWER_PIN, LOW);
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_DISPLAY_RANDOM:
            pixelDisplayRandomColor(receiveBuffer[1]+receiveBuffer[2]*256);
            commandReceive = I2C_CMD_NULL;
        break;            

        case I2C_CMD_SET_BRIGHTNESS:
            pixelSetBrightness(receiveBuffer[1]);
			displayPriorityFlag = false;
            commandReceive = I2C_CMD_NULL;
        break;

		case I2C_CMD_LED_ON:
			ledFlashCommand = true;
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_LED_OFF:
			ledFlashCommand = false;
			ledFlashStatus = false;
			displayPriorityFlag = false;
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

        case I2C_CMD_TEST_GET_VER:
            Wire.write(versions, 3);
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_GET_DEVICE_UID:
            Wire.write(chipId, 12);
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_GET_VOLT:
            Wire.write(powerVoltage & 0xff);
            Wire.write((powerVoltage >> 8) & 0xff);
            commandReceive = I2C_CMD_NULL;
        break;

		default:
		break;
	}
}

/***************************************************************
 Device driver
 ***************************************************************/

void pixelInit(void)
{
    pinMode(GROVE_LED_PIN_NUM, OUTPUT);
	digitalWrite(GROVE_LED_PIN_NUM, HIGH);

    pinMode(PIXELS_POWER_PIN, OUTPUT);
    digitalWrite(PIXELS_POWER_PIN, HIGH);

    pixel.begin();
    pixelSetBrightness(100);
}


void pixelDisplaySingle(uint8_t r, uint8_t g, uint8_t b)
{
    pixel.setPixelColor(0, pixel.Color(r, g, b));
    show_flag = true;
    // pixel.show();
}

void pixelDisplaySingle(uint32_t rgb)
{
    pixel.setPixelColor(0, rgb);
    show_flag = true;
    // pixel.show();
}

void pixelDisplayOneColor(uint8_t r, uint8_t g, uint8_t b, uint16_t duration)
{
    digitalWrite(PIXELS_POWER_PIN, HIGH);

    displayMode = 1;
    displayTimer = duration;
    if (duration == 0) displayForeverFlag = true;
    else displayForeverFlag = false;
    now_colors = pixel.Color(r, g, b);
    displayTimerPreviousMillis = millis();
}

void pixelDisplayRandomColor(uint16_t duration)
{
    digitalWrite(PIXELS_POWER_PIN, HIGH);

    displayMode = 1;
    displayTimer = duration;
    if (duration == 0) displayForeverFlag = true;
    else displayForeverFlag = false;
    now_colors = wheel(random(0, 256));
    displayTimerPreviousMillis = millis();    
}

void pixelDisplayBlink(uint8_t r, uint8_t g, uint8_t b, uint16_t interval, uint16_t duration)
{
    digitalWrite(PIXELS_POWER_PIN, HIGH);

    displayMode = 2;
    displayTimer = duration;
    if (duration == 0) displayForeverFlag = true;
    else displayForeverFlag = false;
    red = r;
    green = g;
    blue = b;
    blinkInterval_ms = interval;
    displayTimerPreviousMillis = millis();
    displayIntervalTimerPreviousMillis = displayTimerPreviousMillis;
}

void pixelDisplayBreathe(uint8_t r, uint8_t g, uint8_t b, uint16_t interval, uint16_t duration)
{
    digitalWrite(PIXELS_POWER_PIN, HIGH);

    displayMode = 3;
    displayTimer = duration;
    if (duration == 0) displayForeverFlag = true;
    else displayForeverFlag = false;
    red = r;
    green = g;
    blue = b;
    breatheInterval_ms = interval;
    displayCount = 0;
    displayFlashFlag = true;
    displayTimerPreviousMillis = millis();
    displayIntervalTimerPreviousMillis = displayTimerPreviousMillis;
}

void pixelDisplayRainbow(uint16_t interval, uint16_t duration)
{
    digitalWrite(PIXELS_POWER_PIN, HIGH);

    displayMode = 4;
    displayTimer = duration;
    if (duration == 0) displayForeverFlag = true;
    else displayForeverFlag = false;
    rainbowInterval_ms = interval;
    displayCount = 0;
    displayTimerPreviousMillis = millis();
    displayIntervalTimerPreviousMillis = displayTimerPreviousMillis;
}

void pixelDisplayFade(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2, uint16_t interval, uint16_t duration)
{
    digitalWrite(PIXELS_POWER_PIN, HIGH);
    
    displayMode = 5;
    displayForeverFlag = true;
    displayTimer = duration;
    fadeInterval_ms = interval;

    red = r1;
    green = g1;
    blue = b1;
    fade_red_slope = (int16_t)(r2 - r1);
    fade_green_slope = (int16_t)(g2 - g1);
    fade_blue_slope = (int16_t)(b2 -b1);

    displayCountInt = -10;
    displayTimerPreviousMillis = millis();
    displayIntervalTimerPreviousMillis = displayTimerPreviousMillis;
}

void pixelSetBrightness(uint8_t brightness)
{
    pixel.setBrightness(brightness);
}




uint32_t wheel(uint8_t wheelPos)
{
    wheelPos = 255 - wheelPos;
    if(wheelPos < 85)
    {
        return pixel.Color(255 - wheelPos * 3, 0, wheelPos * 3);
    }
    if(wheelPos < 170)
    {
        wheelPos -= 85;
        return pixel.Color(0, wheelPos * 3, 255 - wheelPos * 3);
    }
    wheelPos -= 170;
    return pixel.Color(wheelPos * 3, 255 - wheelPos * 3, 0);
}

uint8_t limiter(int8_t n)
{
    if (n >= 100) return 100;
    else if (n <= 0) return 0;
    else return n;
}
