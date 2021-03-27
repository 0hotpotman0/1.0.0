
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>
#include <Timer3.h>

#define TM1668

#define GROVE_TX_PIN_NUM	PA2
#define GROVE_RX_PIN_NUM	PA3

#define GROVE_LED_PIN_NUM		PA1

/***************************************************************
 Board defines
 ***************************************************************/
#define POWER_EN_PIN    	PB1
#define POWER_SAMPLE_PIN	PF1

#define TM1668_CLK      	PA5
#define TM1668_DIO      	PA6
#define TM1668_STB      	PA7

#define ADDR_AUTO  0x40
#define ADDR_FIXED 0x44

#ifdef	TM1668
static int8_t tubeTab[20] = 
{
    0x3f, 0x30, 0x5b, 0x79, 0x74, 0x6d, 0x6f, 0x38, 0x7f, 0x7d, // 0 - 9
    0x7e, 0x75, 0x0f, 0x73, 0x5b, 0x4b, 0x00,// A,b,C,d,E,F
}; 
#else
static int8_t tubeTab[20] = 
{
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, // 0 - 9
    0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x00,// A,b,C,d,E,F
}; 
#endif

int8_t dispBuf[8] = {0,};
uint8_t cmdSetData = 0x40;
uint8_t cmdSetAddr = 0xc0;
uint8_t cmdDispCtrl = 0x8f;
bool ColonFlag = false;

#define DISPLAY_NUMBER    0 // display number mode
#define DISPLAY_TIMER     1 // timer mode
#define DISPLAY_STOPWATCH 2 // stopwatch mode
#define DISPLAY_NULL      3 // nothing
#define DISPLAY_BLINK     4 // blink mode
#define DISPLAY_STRING    5 // strings mode

uint8_t displayMode = DISPLAY_NULL;
uint16_t timerDiv = 0, timerData = 0, timerDataPre = 0;
uint16_t stopwatchData = 0, stopwatchDataPre = 0;

uint32_t powerVoltage = 0;
    
/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x22
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x8006

#define I2C_DEF_ADDR_FLASH_LOC	0x00
#define I2C_CUR_ADDR_FLASH_LOC	0x01

#define I2C_CMD_GET_DEV_ID		0x00 // 
#define I2C_CMD_GET_DEV_EVENT	0x01 //
#define I2C_CMD_DISPLAY_NUM     0x02 // 
#define I2C_CMD_START_TIMER     0x03 // 
#define I2C_CMD_START_STOPWATCH 0x04 // 
#define I2C_CMD_PAUSE_STOPWATCH   0x05 // 
#define I2C_CMD_RESUME_STOPWATCH  0x06 // 
#define I2C_CMD_RESET_STOPWATCH 0x07 // 
#define I2C_CMD_GET_STOPWATCH   0x08 // 
#define I2C_CMD_CLEAR_DISPLAY   0x09 // 
#define I2C_CMD_BLINK           0x0a 
#define I2C_CMD_SET_BRIGHTNESS  0x0b

#define I2C_CMD_DISPLAY_STR     0xe5

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
uint8_t	ledFlashTimes = 0, ledDisplayTimes = 0;

#ifdef BLE_SUPPORT

#define I2C_CMD_LOW_PRIORITY		0x90

bool displayPriorityFlag = false;

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
	uint8_t		num[2];
}packet_diplay_num;

typedef struct
{
	uint8_t		timer[2];
}packet_diplay_timer;

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
		packet_diplay_timer		timer;
		packet_diplay_num		num;
		packet_raw_t			raw;
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

uint8_t	WatchRawData = 0;
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
// #define AUTO_SLEEP_TIMEOUT	5000

uint32_t clickCheckPreviousMillis = 0;
uint32_t autoSleepPreviousMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

#define LED_FLASH_TIME	250

bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;
uint32_t ledDisplayPreviousMillis = 0;

bool testFlag = false;
char *versions = "V20";
uint16_t NodeVersion = 0x6102;

uint32_t intStart = 0;
uint32_t intEnd = 0;

#define VOLTAGE_CHECK_TIMEOUT 1000

uint32_t voltagePreviousMillis = 0;

/***************************************************************

 ***************************************************************/
uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const static uint8_t display_interval = 100; //100ms
uint32_t display_number_previous_time = 0;
uint32_t display_blink_preivious_time = 0;
int16_t number_to_display = 0;
bool display_blink_state = false;

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
    
    pinMode(POWER_EN_PIN, OUTPUT);
    digitalWrite(POWER_EN_PIN, HIGH);
    
    pinMode(TM1668_CLK, OUTPUT);
    pinMode(TM1668_DIO, OUTPUT);
    pinMode(TM1668_STB, OUTPUT);
    digitalWrite(TM1668_CLK, HIGH);
    digitalWrite(TM1668_DIO, HIGH);
    digitalWrite(TM1668_STB, HIGH);

    // This delay is important, it is to avoid communication errors caused by the hardware reset bug.
    delay(100);
	
	Wire.begin(deviceI2CAddress);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);
    
	wwdg.begin();
    
    clearDisplay();
#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif
	    
    Timer3.init(1000); // 1 ms
    Timer3.attachInterrupt(timerIsr);
    Timer3.stop();

    // startStopwatch();  
    // startTimer(60);
    // number_to_display = -999;
    // displayMode = DISPLAY_BLINK;
	
}

void loop()
{
	uint32_t data, i, j;
	uint32_t CurrentMillis = millis();
			
#ifdef BLE_SUPPORT
	if(Wire.isbusidle())PreMillis = CurrentMillis;
	if ((CurrentMillis - PreMillis) > 20)
	{
		digitalWrite(GROVE_LED_PIN_NUM, LOW);
		Wire.reset();
		Wire.begin(deviceI2CAddress);
		Wire.onReceive(receiveEvent);
		Wire.onRequest(requestEvent);
		PreMillis = CurrentMillis;
		digitalWrite(GROVE_LED_PIN_NUM, HIGH);
	}
#endif
	
    if(displayMode == DISPLAY_TIMER)
    {
        if(timerDataPre != timerData)
        {
            timerDataPre = timerData;
            displayNumber(timerData);
        }
    }
    if(displayMode == DISPLAY_STOPWATCH)
    {
        if(stopwatchDataPre != stopwatchData)
        {
            stopwatchDataPre = stopwatchData;
            displayStopWatch(stopwatchDataPre);
        }
    }
    if (displayMode == DISPLAY_NUMBER)
    {
        if (millis() - display_number_previous_time >= display_interval)
        {
            displayNumber(number_to_display);
            display_number_previous_time = millis();
        }
    }
    if (displayMode == DISPLAY_STRING)
    {
        if (millis() - display_number_previous_time >= display_interval)
        {
            display(dispBuf);
            display_number_previous_time = millis();
        }
    }
    if (displayMode == DISPLAY_BLINK)
    {
        if (millis() - display_blink_preivious_time >= 500)
        {
            if (display_blink_state) displayFull();
            else clearDisplay();
            display_blink_state = !display_blink_state;
            display_blink_preivious_time = millis();
        }
    }
    
    uint32_t voltageCurrentMillis = millis();
    if(voltageCurrentMillis - voltagePreviousMillis >= VOLTAGE_CHECK_TIMEOUT)
    {
        voltagePreviousMillis = voltageCurrentMillis;
        powerVoltage = analogRead(POWER_SAMPLE_PIN) * 3300 * 2 / 1024;
    }
	
	if(ledDisplayTimes <= 11)
	{
		uint32_t ledFlashCurrentMillis = millis();
		
		if(ledFlashCurrentMillis - ledDisplayPreviousMillis >= 500)
		{
			if(ledDisplayTimes == 11)
			{
				clearDisplay();
			}else if(ledDisplayTimes == 10)
			{
				ColonFlag = true;
				dispBuf[0] = 0x7F;
				dispBuf[1] = 0x7F;
				dispBuf[2] = 0x7F;
				dispBuf[3] = 0x7F;
				display(dispBuf);
			}else
			{
				for(i=0;i<4;i++)
				{
					dispBuf[i] = ledDisplayTimes;
				}
				display(dispBuf);
			}
			ledDisplayPreviousMillis = ledFlashCurrentMillis;
			ledDisplayTimes++;
		}
	}else if (displayMode == DISPLAY_NULL)
	{
		if (millis() - display_number_previous_time >= display_interval)
		{
			clearDisplay();
			ColonFlag = false;
			display_number_previous_time = millis();
		}
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
			StartMillis = CurrentMillis;
			commandReceive = I2C_CMD_NOTIFY_ATR;
#endif
		}
	}
	if(Core_mode == CORE_BLE_MODE)
	{// Message process.
		uint32_t currentMillis = millis();
#if 1
		if(packet_01_data.data.deviceEvent > 0)
		{
			InstructionOption.event.Header.type = I2C_CMD_NOTIFY_EVENT;
			InstructionOption.event.Header.Address = deviceI2CAddress;
			InstructionOption.event.Event  = (uint8_t)packet_01_data.data.deviceEvent;
			InstructionOption.event.Header.Datalen	 = sizeof(packet_event);
			Wire.MasterGPIOTransmission(ptr2, sizeof(packet_event));
			packet_01_data.data.deviceEvent = 0;
			displayPriorityFlag = false;
		}
#endif
		if (WatchRawData)
		{
			if ((currentMillis - RawPreviousMillis) >= I2C_CMD_RAW_DATA_TIME)
			{
				data = getStopwatch();
				InstructionOption.raw_data.Header.type = I2C_CMD_GET_RAW_DATA;
				InstructionOption.raw_data.Header.Address = deviceI2CAddress;
				InstructionOption.raw_data.data[0]	= data&0xFF;
				InstructionOption.raw_data.data[1]	= (data>>8)&0xFF;
				InstructionOption.raw_data.data[2]	= (data>>16)&0xFF;
				InstructionOption.raw_data.data[3]	= (data>>24)&0xFF;
				InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);
				Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw));
				RawPreviousMillis = currentMillis;
			}
		}

		switch(commandReceive)
		{
			case  I2C_CMD_GET_VOLT:
			// Raw data request, send powerVoltage.
				InstructionOption.raw_data.Header.type		= I2C_CMD_GET_VOLT;
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
            
            Timer3.stop();

            digitalWrite(POWER_EN_PIN, LOW);
            pinMode(TM1668_CLK, INPUT);
            pinMode(TM1668_DIO, INPUT);

			wwdg.end();
			Wire.end();
			pinMode(PA9, INPUT_PULLUP);
			pinMode(PA10, INPUT_PULLUP);
			
			nrgSave.standby();
			
			Wire.begin(deviceI2CAddress);
			Wire.onReceive(receiveEvent);
			Wire.onRequest(requestEvent);
			wwdg.begin();
            
            digitalWrite(POWER_EN_PIN, HIGH);
            pinMode(TM1668_CLK, OUTPUT);
            pinMode(TM1668_DIO, OUTPUT);
            digitalWrite(TM1668_CLK, HIGH);
            digitalWrite(TM1668_DIO, HIGH);
            
            Timer3.start();
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
	if ((receiveBuffer[0] >= MAINBOARD_BLE_COMMAND_LOW) && 
		(receiveBuffer[0] <  MAINBOARD_BLE_COMMAND_HIGH))
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
			commandReceive += I2C_CMD_DISPLAY_NUM;
		}else if (commandReceive < 10)
		{
			commandReceive += I2C_CMD_DISPLAY_NUM;
			displayPriorityFlag = true;	// High priority
		}else
			displayPriorityFlag = false;
		
		for(j=0;j<(i-sizeof(packet_header_t));j++)
		{
			receiveBuffer[j+1] = receiveBuffer[j+sizeof(packet_header_t)];
		}
		i -= (sizeof(packet_header_t)-1);
		if (commandReceive == I2C_CMD_START_STOPWATCH)
		{
			commandReceive = I2C_CMD_START_STOPWATCH+commandOption.data.commands.num.num[0];
		}
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
				WatchRawData = 1;
				RawPreviousMillis = 0;
			}else
				WatchRawData = 0;
			commandReceive = I2C_CMD_NULL;
			
			break;
#endif
		case  I2C_CMD_GET_STOPWATCH:
			WatchRawData = 1;
			RawPreviousMillis = 0;
			commandReceive = I2C_CMD_NULL;

			break;
		case I2C_CMD_DISPLAY_NUM:
            displayMode = DISPLAY_NUMBER;
			stopwatchData = 0;
			ledDisplayTimes = 100;
			WatchRawData = 0;
            number_to_display = (int16_t)(receiveBuffer[1] + receiveBuffer[2] * 256);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_DISPLAY_STR:
            commandReceive = I2C_CMD_NULL;
			if (i < 6)
			{
				break;
			}
			displayMode = DISPLAY_STRING;
			for(j=0;j<4;j++)
			{
				dispBuf[j] = receiveBuffer[5-j];
			}
			ColonFlag = receiveBuffer[1];
			ledDisplayTimes = 100;
			WatchRawData = 0;
			stopwatchData = 0;
        break;
        
        case I2C_CMD_START_TIMER:
            startTimer((uint16_t)(receiveBuffer[1] + receiveBuffer[2] * 256));
			ledDisplayTimes = 100;
			stopwatchData = 0;
			WatchRawData = 0;
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_SET_BRIGHTNESS:
            cmdDispCtrl = 0x88 | (receiveBuffer[1] & 0x07);
            commandReceive = I2C_CMD_NULL;
        break;
		
        case I2C_CMD_START_STOPWATCH:
            startStopwatch();
			ledDisplayTimes = 100;
			WatchRawData = 1;
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_PAUSE_STOPWATCH:
            pauseStopwatch();
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_RESUME_STOPWATCH:
            resumeStopwatch();
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_RESET_STOPWATCH:
            resetStopwatch();
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_CLEAR_DISPLAY:
            clearDisplay();
			stopwatchData = 0;
            displayMode = DISPLAY_NULL;
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_BLINK:
            displayMode = DISPLAY_BLINK;
			stopwatchData = 0;
			WatchRawData = 0;
            commandReceive = I2C_CMD_NULL;            
        break;
        
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
        
        case I2C_CMD_GET_STOPWATCH:
            {
                uint32_t data = getStopwatch();
                Wire.write((uint8_t *)(&data), 4);
                commandReceive = I2C_CMD_NULL;
            }
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
void start(void)
{
    digitalWrite(TM1668_CLK, HIGH);
    digitalWrite(TM1668_DIO, HIGH);
    delayMicroseconds(2);
    digitalWrite(TM1668_DIO, LOW);
}

void stop(void)
{
    digitalWrite(TM1668_CLK, LOW);
    delayMicroseconds(2);
    digitalWrite(TM1668_DIO, LOW);
    delayMicroseconds(2);
    digitalWrite(TM1668_CLK, HIGH);
    delayMicroseconds(2);
    digitalWrite(TM1668_DIO, HIGH);
}

void writeByte(uint8_t data)
{
    uint8_t i;
    
    for(i = 0;i < 8;i ++)        //sent 8bit data
    {
        digitalWrite(TM1668_CLK, LOW);
        if(data & 0x01)digitalWrite(TM1668_DIO, HIGH);//LSB first
        else digitalWrite(TM1668_DIO, LOW);
        delayMicroseconds(3);
        data >>= 1;
        digitalWrite(TM1668_CLK, HIGH);
        delayMicroseconds(3);
    }
}

void set(uint8_t brightness, uint8_t setData, uint8_t setAddr)
{
    cmdSetData = setData;
    cmdSetAddr = setAddr;
    cmdDispCtrl = 0x88 + brightness;//Set the brightness and it takes effect the next time it displays.
}

void coding(uint8_t dispData[])
{    
    for(uint8_t i = 0; i < 4; i ++)
    {
        if(dispData[i] == 0x7f)dispData[i] = 0x00;
        else 
        {
            dispData[i] = tubeTab[dispData[i]];
        }
    }
}

uint8_t coding(uint8_t dispData)
{    
    if(dispData == 0x7f) dispData = 0x00;
    else if (dispData == 0x40) dispData = 0x40;
    else 
    {
        dispData = tubeTab[dispData];
    }
    return dispData;
}

void display(int8_t dispData[])
{
    uint8_t segData[4], i;
    
    for(i = 0;i < 4;i ++)segData[i] = dispData[i];
    coding(segData);
	if (ColonFlag)segData[1] |= 0x80;
	
	digitalWrite(TM1668_STB, HIGH);
    delayMicroseconds(5);
	digitalWrite(TM1668_STB, LOW);
    delayMicroseconds(5);
    start();
    writeByte(ADDR_AUTO);
    stop();
	digitalWrite(TM1668_STB, HIGH);
    delayMicroseconds(5);
	digitalWrite(TM1668_STB, LOW);
    delayMicroseconds(5);
    start();
    writeByte(cmdSetAddr);
    for(i=0;i < 7;i ++)
    {
        writeByte(segData[i]);
		delayMicroseconds(2);
        writeByte(0);
		delayMicroseconds(2);
    }
    stop();
	digitalWrite(TM1668_STB, HIGH);
    delayMicroseconds(5);
	digitalWrite(TM1668_STB, LOW);
    delayMicroseconds(5);
    start();
    writeByte(cmdDispCtrl);
    stop();
	digitalWrite(TM1668_STB, HIGH);
}

void display(uint8_t bitAddr, uint8_t dispData)
{
    uint8_t segData;
    
	bitAddr = 3-bitAddr;
    segData = coding(dispData);
	if (ColonFlag && (bitAddr == 1))segData |= 0x80;
	digitalWrite(TM1668_STB, LOW);
    delayMicroseconds(5);
    start();
    writeByte((bitAddr*2) | 0xc0);
    delayMicroseconds(1);
    writeByte(segData);
    stop();
	digitalWrite(TM1668_STB, HIGH);
    delayMicroseconds(5);
}

void clearDisplay(void)
{
	ColonFlag = false;
	memset(dispBuf, 0x7F, 4);
	display(dispBuf);
}

void displayFull(void)
{
	ColonFlag = true;
	digitalWrite(TM1668_STB, LOW);
    delayMicroseconds(5);
    start();
    writeByte(ADDR_FIXED);
    stop();
	digitalWrite(TM1668_STB, HIGH);
    delayMicroseconds(5);
    display(0x00,8);
    display(0x01,8);
    display(0x02,8);
    display(0x03,8);
	digitalWrite(TM1668_STB, LOW);
    delayMicroseconds(5);
    start();
    writeByte(cmdDispCtrl);
    stop();
	digitalWrite(TM1668_STB, HIGH);
	ColonFlag = false;
}

// -999 ~ 9999
void displayNumber(int num)
{
    if(num > 9999 || num < -999)return;
    
    // static int numBuf = 0;
    // if(num == numBuf)return;
    // numBuf = num;
	ColonFlag = false;
    bool negative_flag = false;
    if (num < 0) {
        negative_flag = true;
        num = 0 - num;
    }
	digitalWrite(TM1668_STB, LOW);
    delayMicroseconds(5);
    start();
    writeByte(ADDR_FIXED);
    stop();
	digitalWrite(TM1668_STB, HIGH);
    delayMicroseconds(5);

    if(num < 10)
    {
        display(3, num);
        if (negative_flag) display(2, 0x40);
        else display(2, 0x7f);
        display(1, 0x7f);
        display(0, 0x7f);
    }
    else if(num < 100)
    {
        display(3, num%10);
        display(2, (num/10)%10);
        if (negative_flag) display(1, 0x40);
        else display(1, 0x7f);
        display(0, 0x7f);
    }
    else if(num < 1000)
    {
        display(3, num % 10);
        display(2, (num / 10) % 10);
        display(1, (num / 100) % 10);
        if (negative_flag) display(0, 0x40);
        else display(0, 0x7f);
    }
    else
    {
        display(3, num % 10);
        display(2, (num / 10) % 10);
        display(1, (num / 100) % 10);
        display(0, (num / 1000) % 10);
    }
	digitalWrite(TM1668_STB, LOW);
    delayMicroseconds(5);
    start();
    writeByte(cmdDispCtrl);
    stop();
	digitalWrite(TM1668_STB, HIGH);

}

void startTimer(uint16_t data)
{
    if(data > 9999)data = 9999;
    
    Timer3.stop();
    packet_01_data.data.deviceEvent = 0;
    
    displayMode = DISPLAY_TIMER;
    timerData = data;
    stopwatchData = 0;
    Timer3.start();
}

void startStopwatch(void)
{
    Timer3.stop();
    packet_01_data.data.deviceEvent = 0;
    
    displayMode = DISPLAY_STOPWATCH;
    timerData = 0;
    stopwatchData = 0;
    Timer3.start(); 
}

void pauseStopwatch(void)
{
    Timer3.stop();
    displayMode = DISPLAY_STOPWATCH;
}

void resumeStopwatch(void)
{
    Timer3.start();
    displayMode = DISPLAY_STOPWATCH;
}

void resetStopwatch(void)
{
    Timer3.stop();
    stopwatchData = 0;
    displayMode = DISPLAY_STOPWATCH;
}

uint32_t getStopwatch(void)
{
    uint32_t data = stopwatchData * 10;
    return data;
}

void timerIsr(void)
{ 
    if(displayMode == DISPLAY_TIMER)
    {
        timerDiv ++;
        if(timerDiv >= 1000)
        {
            timerDiv = 0;
            if(timerData > 0)
            {
                timerData --;
                if(timerData == 0)
                {
                    Timer3.stop();
                    packet_01_data.data.deviceEvent = 1; // Generate a timer out event
                }
            }
        }
    }
    else if(displayMode == DISPLAY_STOPWATCH)
    {
        timerDiv ++;
        if(timerDiv >= 10)
        {
            timerDiv = 0;
            if(stopwatchData < 9999)stopwatchData ++;
        }
    }
}

void displayStopWatch(int num)
{
    if(num > 9999 || num < 0)return;
    
    // static int numBuf = 0;
    // if(num == numBuf)return;
    // numBuf = num;
	digitalWrite(TM1668_STB, LOW);
    delayMicroseconds(5);
    start();
    writeByte(ADDR_FIXED);
    stop();
	digitalWrite(TM1668_STB, HIGH);
    delayMicroseconds(5);
    ColonFlag = true;
    if(num < 10)
    {
        display(3, num);
        display(2, 0);
        display(1, 0);
        display(0, 0);
    }
    else if(num < 100)
    {
        display(3, num%10);
        display(2, (num/10)%10);
        display(1, 0);
        display(0, 0);
    }
    else if(num < 1000)
    {
        display(3, num % 10);
        display(2, (num / 10) % 10);
        display(1, (num / 100) % 10);
        display(0, 0);
    }
    else
    {
        display(3, num % 10);
        display(2, (num / 10) % 10);
        display(1, (num / 100) % 10);
        display(0, (num / 1000) % 10);
    }
	
	digitalWrite(TM1668_STB, LOW);
    delayMicroseconds(5);
    start();
    writeByte(cmdDispCtrl);
    stop();
	digitalWrite(TM1668_STB, HIGH);
}
