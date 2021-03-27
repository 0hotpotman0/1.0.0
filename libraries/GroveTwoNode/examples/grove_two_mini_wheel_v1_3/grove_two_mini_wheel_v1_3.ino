
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>


#define GROVE_TX_PIN_NUM	PA2
#define GROVE_RX_PIN_NUM	PA3

#define GROVE_LED_PIN_NUM		PF0
// #define GROVE_LED_PIN_NUM		PA14

/***************************************************************
 Board defines
 ***************************************************************/
#define MOTOR_PWM_FREQUENCY 50000 // 50KHz
#define MIN_ABS_SPEED   16

#define CH1_PWMB PA4 // TIM14 - CH1
#define CH1_PWMA PA6 // TIM3 - CH1
#define CH2_PWMB PA7 // TIM3 - CH2
#define CH2_PWMA PB1 // TIM3 - CH4

#define MOTOR_FAULT     PA5
#define SLEEP_ENABLE    PA1
// change at 2018/03/21
// #define MOTOR_ENABLE    PA0 // HIGH: enable, LOW: disable
#define MOTOR_ENABLE    PF1 // HIGH: enable, LOW: disable

#define POWER_SAMPLE_PIN   PA0

uint16_t timerArp;
int16_t leftDuty = 0, rightDuty = 0;

#define Straight  1
#define Back  2
#define Left  3
#define Right 4
#define Clockwise 5
#define Anticlockwise  6

#define Slow 1
#define Medium 2
#define Fast 3

uint16_t motionTimer = 0;
uint32_t motionPreviousMillis = 0;
uint8_t	ledFlashTimes = 0;

#define MAX_SPEED_VAL    255 // duty cycle
#define ONCE_SPEED_UPDATE 40 // microsecond

bool motorSpeedUpdate = false;
int16_t motorLeftSpeedGoal = 0, motorRightSpeedGoal = 0;
int16_t motorLeftSpeedCurr = 0, motorRightSpeedCurr = 0;
uint32_t motorSpeedPreviousMicros = 0;

/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x28
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x8009

#define I2C_DEF_ADDR_FLASH_LOC	0x00
#define I2C_CUR_ADDR_FLASH_LOC	0x01

#define I2C_CMD_GET_DEV_ID		0x00 //
#define I2C_CMD_WHEEL_RUN    	0x01 //
#define I2C_CMD_WHEEL_MOTION    0x02 //
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

#ifdef BLE_SUPPORT

#define I2C_CMD_BLE_WHEEL_RUN		0x90
#define I2C_CMD_BLE_WHEEL_MOTION	0x91

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
	uint8_t		speed;
	uint8_t		direction;
	uint8_t 	time[2];
}packet_wheel_motion;

typedef struct
{
	uint8_t 	left[2];
	uint8_t		right[2];
}packet_wheel_run;

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
		packet_wheel_motion	motion;
		packet_wheel_run	run;
		packet_got_atr		atr;
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
	uint8_t 	pid[2];
	uint8_t 	chipid;
	uint8_t 	hwaddress;
	uint8_t 	version[3];
	uint8_t 	option[2];
}packet_atr;

union
{
	packet_atr		atr;
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

uint32_t powerVoltage = 4200;

#define VOLTAGE_CHECK_TIMEOUT 1000

uint32_t voltagePreviousMillis = 0;

//#define DEBUG
/***************************************************************
Basic defines
 ***************************************************************/
LowPower nrgSave;

#define CLICK_CHECK_TIMEOUT	1000
#define AUTO_SLEEP_TIMEOUT	2000

uint32_t clickCheckPreviousMillis = 0;
uint32_t autoSleepPreviousMillis = 0, PreMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

#define LED_FLASH_TIME	250

bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;

bool testFlag = false;
char *versions = "V20";
uint16_t NodeVersion = 0x6101;

uint32_t intStart = 0;
uint32_t intEnd = 0;

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
	packet_01_data.data.deviceEvent = 4200;

	nrgSave.begin(GROVE_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance
#ifdef DEBUG
    Serial.begin(115200);
#endif
#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif

	pinMode(GROVE_LED_PIN_NUM, OUTPUT);
	digitalWrite(GROVE_LED_PIN_NUM, HIGH);

    pinMode(MOTOR_ENABLE, OUTPUT);
    pinMode(SLEEP_ENABLE, OUTPUT);
    pinMode(MOTOR_FAULT, INPUT);
    digitalWrite(MOTOR_ENABLE, HIGH); // HIGH: enable, LOW: disable
    digitalWrite(SLEEP_ENABLE, HIGH); // HIGH: disable, LOW: enable

    pwmInit();
	setDuty(0, 0);

	Wire.begin(deviceI2CAddress);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);

	wwdg.begin();
}

void loop()
{
	uint32_t CurrentMillis = millis();

	if(Wire.isbusidle())PreMillis = CurrentMillis;
	if ((CurrentMillis - PreMillis) > 20)
	{
		Wire.end();
		Wire.begin(deviceI2CAddress);
		Wire.onReceive(receiveEvent);
		Wire.onRequest(requestEvent);
		PreMillis = CurrentMillis;
	}

	if(motorSpeedUpdate)
	{
		uint32_t motorSpeedCurrentMicros = micros();
		if(motorSpeedCurrentMicros - motorSpeedPreviousMicros >= ONCE_SPEED_UPDATE)
        {
            motorSpeedPreviousMicros = motorSpeedCurrentMicros;

			if(abs(motorLeftSpeedCurr) <= 105)
			{
				if(motorLeftSpeedCurr < motorLeftSpeedGoal)motorLeftSpeedCurr += 7;
				else motorLeftSpeedCurr -= 7;
			}
			else if(abs(motorLeftSpeedCurr) <= 180)
			{
				if(motorLeftSpeedCurr < motorLeftSpeedGoal)motorLeftSpeedCurr += 5;
				else motorLeftSpeedCurr -= 5;
			}
			else
			{
				if(motorLeftSpeedCurr < motorLeftSpeedGoal)motorLeftSpeedCurr += 3;
				else motorLeftSpeedCurr -= 3;
			}

			if(abs(motorRightSpeedCurr) <= 105)
			{
				if(motorRightSpeedCurr < motorRightSpeedGoal)motorRightSpeedCurr += 7;
				else motorRightSpeedCurr -= 7;
			}
			else if(abs(motorRightSpeedCurr) <= 180)
		    {
				if(motorRightSpeedCurr < motorRightSpeedGoal)motorRightSpeedCurr += 5;
				else motorRightSpeedCurr -= 5;
			}
			else
			{
				if(motorRightSpeedCurr < motorRightSpeedGoal)motorRightSpeedCurr += 3;
				else motorRightSpeedCurr -= 3;
			}

			setDuty(motorLeftSpeedCurr, motorRightSpeedCurr);
			if((abs(motorLeftSpeedCurr-motorLeftSpeedGoal) < 7)
				&& (abs(motorRightSpeedCurr-motorRightSpeedGoal) < 7))
				{
					motorLeftSpeedCurr = motorLeftSpeedGoal;
					motorRightSpeedCurr= motorRightSpeedGoal;
					setDuty(motorLeftSpeedCurr, motorRightSpeedCurr);
					motorSpeedUpdate = false;
				}

        }

	}

    if(motionTimer > 0)
    {
        uint32_t motionCurrentMillis = millis();
        if(motionCurrentMillis - motionPreviousMillis >= motionTimer)
        {
            motionPreviousMillis = motionCurrentMillis;
            setMotor(0, 0);
        }
    }
#if 1
    uint32_t voltageCurrentMillis = millis();
    if(voltageCurrentMillis - voltagePreviousMillis >= VOLTAGE_CHECK_TIMEOUT)
    {
        voltagePreviousMillis = voltageCurrentMillis;
        powerVoltage = analogRead(POWER_SAMPLE_PIN) * 3300 * 2 / 1024;

		if(powerVoltage < 3600)
		{
#if 1
			digitalWrite(GROVE_LED_PIN_NUM, LOW);
#else
			digitalWrite(MOTOR_ENABLE, LOW); // HIGH: enable, LOW: disable
			setMotor(0, 0);
#endif
		}else
		{
			digitalWrite(GROVE_LED_PIN_NUM, HIGH);
		}
		packet_01_data.data.deviceEvent = powerVoltage;

#ifdef DEBUG
    Serial.println(powerVoltage);
#endif
    }
#endif

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

			wwdg.end();
			Wire.end();
			pinMode(PA9, INPUT_PULLUP);
			pinMode(PA10, INPUT_PULLUP);

			digitalWrite(SLEEP_ENABLE, LOW); // HIGH: disable, LOW: enable
			digitalWrite(MOTOR_ENABLE, LOW); // HIGH: enable, LOW: disable

			nrgSave.standby();

			Wire.begin(deviceI2CAddress);
			Wire.onReceive(receiveEvent);
			Wire.onRequest(requestEvent);
			wwdg.begin();

			digitalWrite(MOTOR_ENABLE, HIGH); // HIGH: enable, LOW: disable
			digitalWrite(SLEEP_ENABLE, HIGH); // HIGH: disable, LOW: enable
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
		receiveBuffer[0] = commandOption.data.Header.type;
		if(receiveBuffer[0] < 10)
			receiveBuffer[0]++;
		for(j=0;j<(i-sizeof(packet_header_t));j++)
		{
			receiveBuffer[j+1] = receiveBuffer[j+sizeof(packet_header_t)];
		}
		i -= (sizeof(packet_header_t)-1);
	}else
	{
		Core_mode = CORE_ATMEL_MODE;
	}
#endif
	commandReceive = receiveBuffer[0];
	if(powerVoltage < 3550)
	{
//		commandReceive = I2C_CMD_NULL;
	}

	switch(commandReceive)
	{
        case I2C_CMD_WHEEL_RUN:
        case I2C_CMD_BLE_WHEEL_RUN:
            leftDuty = (int16_t)(receiveBuffer[1] + receiveBuffer[2] * 256);
            rightDuty = (int16_t)(receiveBuffer[3] + receiveBuffer[4] * 256);
            setMotor(leftDuty, rightDuty);
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_WHEEL_MOTION:
        case I2C_CMD_BLE_WHEEL_MOTION:
            setMotion(receiveBuffer[1], receiveBuffer[2], receiveBuffer[3] + receiveBuffer[4] * 256);
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
			Wire.write(ptr1, 4);
			commandReceive = I2C_CMD_NULL;
		break;
	}
}

/***************************************************************
 Device driver
 ***************************************************************/
void pwmInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOA, 4, GPIO_AF_4); // PA4 -> TIM14 -> CH1
    GPIO_PinAFConfig(GPIOA, 6, GPIO_AF_1); // PA6 -> TIM3 -> CH1
    GPIO_PinAFConfig(GPIOA, 7, GPIO_AF_1); // PA7 -> TIM3 -> CH2
    GPIO_PinAFConfig(GPIOB, 1, GPIO_AF_1); // PB1 -> TIM3 -> CH4

    GPIO_InitStructure.GPIO_Pin = (1 << 4) | (1 << 6) | (1 << 7);
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = (1 << 1);
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

    timerArp = (uint16_t)(2000000 / MOTOR_PWM_FREQUENCY) - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 2000000 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = timerArp;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    // PA4 - TIM14 - CH1
    TIM_OC1Init(TIM14, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);

    // PA6 - TIM3 - CH1
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // PA7 - TIM3 - CH2
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // PB1 - TIM3 - CH4
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM14, ENABLE);

	TIM_Cmd(TIM14, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    TIM_CtrlPWMOutputs(TIM14, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
}

void setDutyCycLeftB(uint32_t duty) //
{
    uint16_t channelPulse = (uint16_t)((duty * (timerArp + 1)) / 255);

    TIM_SetCompare1(TIM14, channelPulse);
}

void setDutyCycLeftA(uint32_t duty) //
{
    uint16_t channelPulse = (uint16_t)((duty * (timerArp + 1)) / 255);

    TIM_SetCompare1(TIM3, channelPulse);
}

void setDutyCycRightB(uint32_t duty) //
{
    uint16_t channelPulse = (uint16_t)((duty * (timerArp + 1)) / 255);

    TIM_SetCompare2(TIM3, channelPulse);
}

void setDutyCycRightA(uint32_t duty) //
{
    uint16_t channelPulse = (uint16_t)((duty * (timerArp + 1)) / 255);

    TIM_SetCompare4(TIM3, channelPulse);
}

void setDuty(int left, int right)
{
    if(left > 0 && left < MIN_ABS_SPEED)left = MIN_ABS_SPEED;
    else if(left < 0 && left > -MIN_ABS_SPEED)left = -MIN_ABS_SPEED;

    if(right > 0 && right < MIN_ABS_SPEED)right = MIN_ABS_SPEED;
    else if(right < 0 && right > -MIN_ABS_SPEED)right = -MIN_ABS_SPEED;

    if(left > 255)left = 255;
    else if(left < -255)left = -255;

    if(right > 255)right = 255;
    else if(right < -255)right = -255;

    if(left < 0) {
        setDutyCycLeftA(255);
        setDutyCycLeftB(255 - abs(left));
    }
    else {
        setDutyCycLeftA(255 - abs(left));
        setDutyCycLeftB(255);
    }

    if(right < 0) {
        setDutyCycRightA(255);
        setDutyCycRightB(255 - abs(right));
    }
    else {
        setDutyCycRightA(255 - abs(right));
        setDutyCycRightB(255);
    }
}

void setMotor(int left, int right)
{
	motorLeftSpeedGoal = left;
	motorRightSpeedGoal = right;
	motorSpeedUpdate = true;
//	setDuty(motorLeftSpeedGoal, motorRightSpeedGoal);
}
#if 1
void setMotion(uint8_t speed, uint8_t direction, uint16_t time)
{
    int left = 0, right = 0;
    
    if(speed == Slow){left = 85; right = 85;}
    else if(speed == Medium){left = 170; right = 170;}
    else if(speed == Fast){left = 255; right = 255;}
    
    if(direction == Straight){}
    else if(direction == Back){left = (-1) * left; right = (-1) * right;}
    else if(direction == Left){left = 255 - left / 2; right = 255;}
    else if(direction == Right){right = 255 - right / 2; left = 255;}
    else if(direction == Clockwise){right = -right;}
    else if(direction == Anticlockwise){left = -left;}
    
    motionTimer = time;
    motionPreviousMillis = millis();
    
    setMotor(left, right);
}
#else
void setMotion(uint8_t speed, uint8_t direction, uint16_t time)
{
    int left = 0, right = 0;

    if(speed == Slow){left = 85; right = 85;}
    else if(speed == Medium){left = 170; right = 170;}
    else if(speed == Fast){left = 255; right = 255;}

    if(direction == Straight){}
    else if(direction == Back){left = (-1) * left; right = (-1) * right;}
    else if(direction == Left){left = 0;}
    else if(direction == Right){right = 0;}
    else if(direction == Clockwise){right = -right;}
    else if(direction == Anticlockwise){left = -left;}

    motionTimer = time;
    motionPreviousMillis = millis();

    setMotor(left, right);
}

void setMotion(uint8_t speed, uint8_t direction, uint16_t time)
{
    int left = 0, right = 0;

    if(speed == Slow){left = 96; right = 96;}
    else if(speed == Medium){left = 160; right = 160;}
    else if(speed == Fast){left = 224; right = 224;}

    if(direction == Straight){}
    else if(direction == Back){left = (-1) * left; right = (-1) * right;}
    else if((direction == Left)&&(speed == Medium)){left = 64; right = 224;}
    else if((direction == Left)&&(speed == Fast)){left = 128; right = 255;}
    else if((direction == Right)&&(speed == Medium)){right = 64; left = 224;}
    else if((direction == Right)&&(speed == Fast)){right = 128; left = 255;}
    else if(direction == Clockwise){right = -right;}
    else if(direction == Anticlockwise){left = -left;}

    motionTimer = time;
    motionPreviousMillis = millis();

    setMotor(left, right);
}
#endif
