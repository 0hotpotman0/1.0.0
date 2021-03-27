
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>
#include <Timer3.h>


#define GROVE_TX_PIN_NUM	PA2
#define GROVE_RX_PIN_NUM	PA3

#define GROVE_LED_PIN_NUM		PA1

/***************************************************************
 Board defines
 ***************************************************************/
#define ULTRASONIC_BOOT_EN_PIN      PB1
#define ULTRASONIC_PWM_OUT_PIN      PA4
#define ULTRASONIC_TX_EN_PIN        PA0
#define ULTRASONIC_SIGNAL_OUT_PIN   PA6
#define ULTRASONIC_WAVE_RAW_PIN     PA7
#define ULTRASONIC_POWER_PIN        PA5

uint16_t ultraTimeCount = 0;
bool RXcomplete = false;
bool RXcount = false;
uint16_t waveCountTimer[256];
uint8_t waveNum = 0;

bool thldFlag = false;
uint32_t timeoutPreviousMillis = 0;

uint32_t curDistData = 0, distancePre = 0;
bool distanceFlag = false;

uint32_t powerVoltage = 0;

/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x23
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

    uint16_t distThd = Flash.read16(I2C_THD_0_ADDR_FLASH_LOC);
    if(distThd == 0xffff)Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, DISTANCE_THD_0_NUM);
    else distThd0 = distThd;

    distThd = Flash.read16(I2C_THD_1_ADDR_FLASH_LOC);
    if(distThd == 0xffff)Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, DISTANCE_THD_1_NUM);
    else distThd1 = distThd;

    packet_01_data.data.deviceVID = DEVICE_VID;
    packet_01_data.data.devicePID = DEVICE_PID;
    packet_01_data.data.deviceEvent = 0;

    nrgSave.begin(GROVE_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance

    pinMode(GROVE_LED_PIN_NUM, OUTPUT);
    digitalWrite(GROVE_LED_PIN_NUM, HIGH);

    pinMode(ULTRASONIC_BOOT_EN_PIN, OUTPUT);
    pinMode(ULTRASONIC_TX_EN_PIN, OUTPUT);
    pinMode(ULTRASONIC_WAVE_RAW_PIN, INPUT);

    digitalWrite(ULTRASONIC_BOOT_EN_PIN, HIGH);
    digitalWrite(ULTRASONIC_TX_EN_PIN, LOW);
    analogWrite(ULTRASONIC_PWM_OUT_PIN, 0);

    attachInterrupt(ULTRASONIC_SIGNAL_OUT_PIN, waveCount, FALLING, INPUT_PULLUP);

#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif

    Timer3.init(10); // 10us
    Timer3.attachInterrupt(timerIsr);
    TIM_Cmd(TIM3, DISABLE);

    Wire.begin(deviceI2CAddress);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    wwdg.begin();

    // Serial.begin(115200);
}

void loop()
{
    ultrasonicMeasureStartType();
    if(curDistData < distThd0)packet_01_data.data.deviceEvent = 1;
    else if(curDistData >= distThd0 && curDistData < distThd1)packet_01_data.data.deviceEvent = 2;
    else if(curDistData >= distThd1)packet_01_data.data.deviceEvent = 3;

    uint32_t voltageCurrentMillis = millis();
    if(voltageCurrentMillis - voltagePreviousMillis >= VOLTAGE_CHECK_TIMEOUT)
    {
        voltagePreviousMillis = voltageCurrentMillis;
        powerVoltage = analogRead(ULTRASONIC_POWER_PIN) * 3300 * 2 / 1024;
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
	{
		uint32_t CurrentMillis = millis();
#if 1
		if(preEvent != packet_01_data.data.deviceEvent)
		{
			InstructionOption.event.Header.type = I2C_CMD_NOTIFY_EVENT;
			InstructionOption.event.Header.Address = deviceI2CAddress;
			InstructionOption.event.Event  = packet_01_data.data.deviceEvent;
			InstructionOption.event.Header.Datalen	 = sizeof(packet_event);
			Wire.MasterGPIOTransmission(ptr2, sizeof(packet_event));
			preEvent = packet_01_data.data.deviceEvent;
		}
#endif

		if (LightRawData)
		{
			if (CurrentMillis - RawPreviousMillis >= RawDelayMillis)
			{
				InstructionOption.raw_data.Header.type = I2C_CMD_GET_RAW_DATA;
				InstructionOption.raw_data.Header.Address = deviceI2CAddress;
				InstructionOption.raw_data.data[0] = curDistData&0xFF;
				InstructionOption.raw_data.data[1] = (curDistData>>8)&0xFF;
				InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);
				Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw));

				RawPreviousMillis = CurrentMillis;
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
				break;
			case I2C_CMD_BLE_SET_THD:
				{
					distThd0 = (int16_t)commandOption.data.commands.thread.threshold0[0] +
										commandOption.data.commands.thread.threshold0[1] * 256;
					distThd1 = (int16_t)commandOption.data.commands.thread.threshold1[0] +
										commandOption.data.commands.thread.threshold1[1] * 256;
					if(commandOption.data.commands.thread.flashSave>0)
					{
						Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, distThd0);
						Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, distThd1);
					}
				}
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
            Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, distThd0);
            Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, distThd1);
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
            autoSleepPreviousMillis = autoSleepCurrentMillis;

            ledFlashCommand = false;
            ledFlashStatus = false;
            digitalWrite(GROVE_LED_PIN_NUM, HIGH);

            digitalWrite(ULTRASONIC_BOOT_EN_PIN, LOW);
            digitalWrite(ULTRASONIC_TX_EN_PIN, LOW);
            analogWrite(ULTRASONIC_PWM_OUT_PIN, 0);

            pinMode(ULTRASONIC_SIGNAL_OUT_PIN, OUTPUT);
            digitalWrite(ULTRASONIC_SIGNAL_OUT_PIN, LOW);
            // detachInterrupt(ULTRASONIC_SIGNAL_OUT_PIN);

            wwdg.end();
            Wire.end();
            pinMode(PA9, INPUT_PULLUP);
            pinMode(PA10, INPUT_PULLUP);

            nrgSave.standby(true);

            Wire.begin(deviceI2CAddress);
            Wire.onReceive(receiveEvent);
            Wire.onRequest(requestEvent);
            wwdg.begin();

            digitalWrite(ULTRASONIC_BOOT_EN_PIN, HIGH);
            attachInterrupt(ULTRASONIC_SIGNAL_OUT_PIN, waveCount, FALLING, INPUT_PULLUP);
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
            int16_t distThd = (int16_t)receiveBuffer[2] + receiveBuffer[3] * 256;
            if(receiveBuffer[1] == 0)distThd0 = distThd;
            else if(receiveBuffer[1] == 1)distThd1 = distThd;
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
#ifdef BLE_SUPPORT
			preEvent = 0;
#endif
        break;

        case I2C_CMD_GET_DISTANCE:
            Wire.write((uint8_t)((curDistData) & 0xff));
			Wire.write((uint8_t)((curDistData) >> 8));
            commandReceive = I2C_CMD_NULL;
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
void ultrasonicMeasureStartType(void)
{
    uint32_t distance = 0;

    if(RXcount)
    {
        if((millis() - timeoutPreviousMillis) > 30)
        {
            RXcomplete = false;
            RXcount = false;

            TIM_Cmd(TIM3, DISABLE);

            if(waveNum)
            {
                distance = ((waveCountTimer[0] * 10 + 1970 + 300) * 172 + 5000); // unit: centimeter * 10000

                if(distance < 100000)distance += 15000; //
                else if(distance >= 100000 && distance < 150000)distance += 10000; //
                else if(distance >= 150000 && distance < 200000)distance += 5000; //
                else if(distance >= 200000 && distance < 250000)distance += 1000; //
                else if((distanceFlag == false) && (distance > 400000))
                {
                    distance = (distancePre + distance) / 2;
                }

                curDistData = distance / 1000; // uint: millimeter

                // Serial.println(curDistData);

                distanceFlag = false;
            }
            curDistData = 0;
        }
    }
    else
    {
        TIM_SetCompare1(TIM14, 13);

        delayMicroseconds(250);
        // analogWrite(ULTRASONIC_PWM_OUT_PIN, 255);   // desable PWM output
        TIM_SetCompare1(TIM14, 25);

        delay(2); // real delay is 1.97 ms
        // digitalWrite(GROVE_LED_PIN_NUM, HIGH);
        ultrasonicRxStart();
    }
}

void ultrasonicRxStart(void)
{
    ultraTimeCount = 0;
    RXcomplete = false;
    RXcount = true;
    waveNum = 0;
    memset(waveCountTimer, 0, sizeof(waveCountTimer));

    // TIM_SetCounter(TIM3, 0); // clean the timer counter
    TIM3->CNT = 0; // clean the timer counter

    // TIM_Cmd(TIM3, ENABLE); // enable timer
    TIM3->CR1 |= TIM_CR1_CEN; // enable timer

    timeoutPreviousMillis = millis();
}

void timerIsr()
{
    ultraTimeCount ++;
}

void waveCount()
{
    if(waveNum < 256)waveCountTimer[waveNum ++] = ultraTimeCount;
}
