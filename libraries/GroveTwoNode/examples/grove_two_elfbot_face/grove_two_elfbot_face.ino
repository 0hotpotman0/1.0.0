
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>
#include <Timer3.h>

#define TM1640

#define GROVE_TX_PIN_NUM	PA2
#define GROVE_RX_PIN_NUM	PA3

#define GROVE_LED_PIN_NUM		PA1

/***************************************************************
 Board defines
 ***************************************************************/
#define POWER_EN_PIN    	PB1
#define POWER_SAMPLE_PIN	PA0

#define TM1640_CLK      	SCL
#define TM1640_DIO      	SDA
#define TM1640_STB      	PA7

#define ADDR_AUTO  0x40
#define ADDR_FIXED 0x44

uint8_t dispBuf[16] = {0,};
uint8_t cmdSetData = 0x40;
uint8_t cmdSetAddr = 0xc0;
uint8_t cmdDispCtrl = 0x8f;
bool ColonFlag = false;

#define DISPLAY_NUMBER    0 // display number mode
#define DISPLAY_TIMER     1 // timer mode
#define DISPLAY_STOPWATCH 2 // stopwatch mode
#define DISPLAY_NULL      3 // nothing
#define DISPLAY_BLINK     4 // blink mode

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
    
    pinMode(TM1640_CLK, OUTPUT);
    pinMode(TM1640_DIO, OUTPUT);
    digitalWrite(TM1640_CLK, HIGH);
    digitalWrite(TM1640_DIO, HIGH);

    // This delay is important, it is to avoid communication errors caused by the hardware reset bug.
    delay(100);
	
	wwdg.begin();
    
    clearDisplay();
#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif

}

void loop()
{
	uint32_t data, i, j;
	
    if (displayMode == DISPLAY_NUMBER)
    {
        if (millis() - display_number_previous_time >= display_interval)
        {
            displayNumber(number_to_display);
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
	
	if(ledDisplayTimes <= 128)
	{
		uint32_t ledFlashCurrentMillis = millis();
		
		if(ledFlashCurrentMillis - ledDisplayPreviousMillis >= 50)
		{
			if(ledDisplayTimes == 128)
			{
				Wire.begin(deviceI2CAddress);
				Wire.onReceive(receiveEvent);
    
				clearDisplay();
			}else
			{
				dispBuf[ledDisplayTimes/8] |= (1 << (ledDisplayTimes%8));
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
            pinMode(TM1640_CLK, INPUT);
            pinMode(TM1640_DIO, INPUT);

			wwdg.end();
			Wire.end();
			pinMode(PA9, INPUT_PULLUP);
			pinMode(PA10, INPUT_PULLUP);
			
			nrgSave.standby();
			
			Wire.begin(deviceI2CAddress);
			Wire.onReceive(receiveEvent);
			wwdg.begin();
            
            digitalWrite(POWER_EN_PIN, HIGH);
            pinMode(TM1640_CLK, OUTPUT);
            pinMode(TM1640_DIO, OUTPUT);
            digitalWrite(TM1640_CLK, HIGH);
            digitalWrite(TM1640_DIO, HIGH);
            
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
        case I2C_CMD_DISPLAY_NUM:
            displayMode = DISPLAY_NUMBER;
			ledDisplayTimes = 160;
            number_to_display = (int16_t)(receiveBuffer[1] + receiveBuffer[2] * 256);
            commandReceive = I2C_CMD_NULL;
        break;

		case I2C_CMD_LED_ON:
			ledFlashCommand = true;
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_LED_OFF:
			ledFlashCommand = false;
			ledFlashStatus = false;
			clearDisplay();
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


/***************************************************************
 Device driver
 ***************************************************************/
void start(void)
{
    digitalWrite(TM1640_CLK, HIGH);
    digitalWrite(TM1640_DIO, HIGH);
    delayMicroseconds(2);
    digitalWrite(TM1640_DIO, LOW);
}

void stop(void)
{
    digitalWrite(TM1640_CLK, LOW);
    delayMicroseconds(2);
    digitalWrite(TM1640_DIO, LOW);
    delayMicroseconds(2);
    digitalWrite(TM1640_CLK, HIGH);
    delayMicroseconds(2);
    digitalWrite(TM1640_DIO, HIGH);
}

void writeByte(uint8_t data)
{
    uint8_t i;
    
    for(i = 0;i < 8;i ++)        //sent 8bit data
    {
        digitalWrite(TM1640_CLK, LOW);
        if(data & 0x01)digitalWrite(TM1640_DIO, HIGH);//LSB first
        else digitalWrite(TM1640_DIO, LOW);
        delayMicroseconds(3);
        data >>= 1;
        digitalWrite(TM1640_CLK, HIGH);
        delayMicroseconds(3);
    }
}

void display(uint8_t dispData[])
{
    uint8_t i;
    
    start();
    writeByte(ADDR_AUTO);
    stop();
    delayMicroseconds(5);
    start();
    writeByte(cmdSetAddr);
    for(i=0;i < 16;i ++)
    {
        writeByte(dispData[i]);
		delayMicroseconds(1);
    }
    stop();
    delayMicroseconds(5);
    start();
    writeByte(cmdDispCtrl);
    stop();
}

void display(uint8_t bitAddr, uint8_t dispData)
{
    delayMicroseconds(5);
    start();
    writeByte(bitAddr | 0xc0);
    delayMicroseconds(1);
    writeByte(dispData);
    stop();
}

void clearDisplay(void)
{
	ColonFlag = false;
	memset(dispBuf, 0, 16);
	display(dispBuf);
}

void displayFull(void)
{
	int i;
	
    start();
    writeByte(ADDR_FIXED);
    stop();
    delayMicroseconds(5);
	for(i=0;i<16;i++)
	{
		display(i,0xFF);
	}
    delayMicroseconds(5);
    start();
    writeByte(cmdDispCtrl);
    stop();
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
    start();
    writeByte(ADDR_FIXED);
    stop();

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
    delayMicroseconds(5);
    start();
    writeByte(cmdDispCtrl);
    stop();

}

