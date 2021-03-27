#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>
#include <Timer3.h>
#include <Timer14.h>

#define GROVE_CH1_PWMA      PB1 // TIM3 - CH4
#define GROVE_CH1_PWMB      PA7 // TIM3 - CH2
#define GROVE_CH2_PWMA      PA4 // TIM14 - CH1
#define GROVE_CH2_PWMB      PA6 // TIM3 - CH1

#define SERVO_PWM1_PIN      PA0
#define SERVO_PWM2_PIN      PA1
#define SERVO_PWM3_PIN      PA2
#define SERVO_PWM4_PIN      PA3

#define POWER_SAMPLE_PIN    PA5

#define SERVO_STEP_PERIOLD  1000 // 1s
#define MAX_ABS_SPEED       100
#define SERVO_NUMBER        4

#define SERVO_MAX_USER_ANGLE     180

#define SERVO_MAX_ANGLE     226
#define SERVO_PERIOLD       22       // us on per degree, 2000 us / 180.
#define SERVO_MIN_US        46       // servo in 0 degree.
#define SERVO_MAX_US        1818     // servo in 20ms.

#define MOTOR_MIN_PULSE     10       // 100 KHz, 10 us, 1 ticks.
#define MOTOR_MAX_PULSE     20000    // 50 Hz, 20000 us, 2000 ticks.

#define MOTOR_PINS          4

/***************************************************************
 Board defines
 ***************************************************************/

#define Straight  1
#define Back  2
#define Left  3
#define Right 4
#define Clockwise 5
#define Anticlockwise  6

#define Slow 1
#define Medium 2
#define Fast 3

#define DC_NONE   0
#define DC_MOTOR  1
#define DC_DRIVE  2

/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS      0x58
#define DEVICE_VID              0x2886
#define DEVICE_PID              0x8007

#define I2C_DEF_ADDR_FLASH_LOC  0x00
#define I2C_CUR_ADDR_FLASH_LOC  0x01

#define I2C_CMD_GET_DEV_ID      0x00 //
#define I2C_CMD_GET_DEV_EVENT   0x01 //

#define I2C_CMD_SET_SERVO_ANGLE 0x02 // set angle of servo

#define I2C_CMD_WHEEL_RUN       0x03 //
#define I2C_CMD_WHEEL_MOTION    0x04 //

#define I2C_CMD_MOTOR_LEFT_RUN  0x05 //
#define I2C_CMD_MOTOR_RIGHT_RUN 0x06 //

#define I2C_CMD_MODE_CHG        0x08 //
#define I2C_CMD_DRIVE_RUN       0x09 //

#define I2C_CMD_SET_MOTOR_PWM   0x0A //
#define I2C_CMD_SET_DRIVE_STEP  0x0B //
#define I2C_CMD_SET_DRIVE_RPM   0x0C //

#define I2C_CMD_SET_ADDR        0xc0 //
#define I2C_CMD_RST_ADDR        0xc1 //
#define I2C_CMD_TEST_GET_VER    0xe2 //
#define I2C_CMD_GET_VOLT        0xe3 //

#define I2C_CMD_JUMP_TO_BOOT    0xf0 //
#define I2C_CMD_GET_DEVICE_UID  0xf1 //
#define I2C_CMD_NULL            0xff //

uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS;
uint8_t commandReceive = I2C_CMD_NULL;
uint8_t commandReceive1 = I2C_CMD_NULL;
uint8_t MotorMode = DC_NONE, PreMode = DC_NONE;

#define DRIVE_STEP_8            8 // 0.9
#define DRIVE_STEP_4            4 // 1.8

uint8_t DriveStep = DRIVE_STEP_4;

uint8_t DV_STEP[DRIVE_STEP_4+DRIVE_STEP_8][MOTOR_PINS] = { 
	{HIGH, LOW, LOW, LOW}, 
	{ LOW, LOW,HIGH, LOW}, 
	{ LOW,HIGH, LOW, LOW}, 
	{ LOW, LOW, LOW,HIGH},
	
	{HIGH, LOW, LOW, LOW}, 
	{HIGH, LOW,HIGH, LOW}, 
	{ LOW, LOW,HIGH, LOW}, 
	{ LOW,HIGH,HIGH, LOW}, 
	{ LOW,HIGH, LOW, LOW}, 
	{ LOW,HIGH, LOW,HIGH}, 
	{ LOW, LOW, LOW,HIGH}, 
	{HIGH, LOW, LOW,HIGH},
};
/* Step C0 C1 C2 C3
 *    1  1  0  1  0
 *    2  0  1  1  0
 *    3  0  1  0  1
 *    4  1  0  0  1
	
	{HIGH, LOW,HIGH, LOW}, 
	{ LOW,HIGH,HIGH, LOW}, 
	{ LOW,HIGH, LOW,HIGH}, 
	{HIGH, LOW, LOW,HIGH},
 */


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

union
{
    uint32_t whole_speed;
    uint8_t speed[4];
}motor_speed_data;

uint8_t *ptr1 = (uint8_t *)&packet_01_data;

void requestEvent();
void receiveEvent(int howMany);
void ServotimerIsr(void);
void MotortimerIsr(void);
void DrivetimerIsr(void);
void PWM_Write(uint8_t ulPin, uint8_t ulValue, uint16_t pwmFreq);
void DV_Step(void);
void setMotion(uint8_t speed, uint8_t direction);

/***************************************************************
Basic defines
 ***************************************************************/
uint32_t timerDiv = 0, MotortimerDiv = 0, ServoMillis = 0, PreMillis = 0;
uint32_t MotorLeftStopTime = 0, MotorRightStopTime = 0;
char *versions = "V20";
uint16_t NodeVersion = 0x6105;

uint32_t powerVoltage = 4200;

#define VOLTAGE_CHECK_TIMEOUT 1000

uint32_t voltagePreviousMillis = 0;

/***************************************************************

 ***************************************************************/
uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t motor_pins[MOTOR_PINS] = {GROVE_CH1_PWMB, GROVE_CH1_PWMA, GROVE_CH2_PWMA, GROVE_CH2_PWMB};
uint8_t servo_pins[SERVO_NUMBER] = {SERVO_PWM1_PIN, SERVO_PWM2_PIN, SERVO_PWM3_PIN, SERVO_PWM4_PIN};
uint8_t current_angle[SERVO_NUMBER] = {90,90,90,90};
uint8_t DV_dir=1, ServoStep = 0, MotorSpeed=0;
uint16_t DV_run=0, DV_rpm = 2000; // 20ms, (360/1.8)*20 = 4000ms, 60000/2000=30 rpm.
uint32_t pwm = MOTOR_MAX_PULSE;    // 50 Hz. 1000/50 = 20 ms = 20000 us = 2000 ticks per cycle.
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

 /*   GPIO_PinAFConfig(GPIOA, 4, GPIO_AF_4); // PA4 -> TIM14 -> CH1
    GPIO_PinAFConfig(GPIOA, 6, GPIO_AF_1); // PA6 -> TIM3 -> CH1
    GPIO_PinAFConfig(GPIOA, 7, GPIO_AF_1); // PA7 -> TIM3 -> CH2
    GPIO_PinAFConfig(GPIOB, 1, GPIO_AF_1); // PB1 -> TIM3 -> CH4
*/
#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif

	pinMode(GROVE_CH1_PWMB, OUTPUT);
	pinMode(GROVE_CH1_PWMA, OUTPUT);
	pinMode(GROVE_CH2_PWMB, OUTPUT);
	pinMode(GROVE_CH2_PWMA, OUTPUT);
	
	pinMode(SERVO_PWM1_PIN, OUTPUT);
	pinMode(SERVO_PWM2_PIN, OUTPUT);
	pinMode(SERVO_PWM3_PIN, OUTPUT);
	pinMode(SERVO_PWM4_PIN, OUTPUT);
/*
    digitalWrite(GROVE_CH1_PWMA, DV_STEP[0][0]);
    digitalWrite(GROVE_CH1_PWMB, DV_STEP[0][1]);
    digitalWrite(GROVE_CH2_PWMA, DV_STEP[0][2]);
    digitalWrite(GROVE_CH2_PWMB, DV_STEP[0][3]);
*/
    //Timer3.attachInterrupt(MotortimerIsr);
    Timer3.init(SERVO_PERIOLD);             // 11 us per tick. 2000/360
    Timer3.attachInterrupt(ServotimerIsr);

    Wire.begin(deviceI2CAddress);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    wwdg.begin();
	
}
uint8_t angle_servo = 90;
uint8_t add_flag = 1;

void loop()
{
    uint32_t CurrentMillis = millis(), i;
	if(CurrentMillis - voltagePreviousMillis >= 1000)
    {
		if(add_flag==1){angle_servo = angle_servo + 1;}
		if(add_flag==0){angle_servo = angle_servo - 1;}
		if (angle_servo>=135)
		{
            angle_servo = 135;
			add_flag = 0;			
		}
		if (angle_servo<=90)
		{
            angle_servo = 90;
			add_flag = 1;			
		}
		voltagePreviousMillis = CurrentMillis;
		current_angle[0] = angle_servo;
		current_angle[1] = angle_servo;
	}
	
	digitalWrite(GROVE_CH1_PWMB, LOW);
	digitalWrite(GROVE_CH1_PWMA, HIGH);
	digitalWrite(GROVE_CH2_PWMB, LOW);
	digitalWrite(GROVE_CH2_PWMA, HIGH);
	
	if ((PreMode == DC_DRIVE) && (MotortimerDiv >= DV_run))
	{
		MotorMode = DC_NONE;
        DV_run = 0;
	}
	if (PreMode == DC_MOTOR)
	{
		if ((CurrentMillis >= MotorLeftStopTime) && (MotorLeftStopTime > 0))
		{
			MotorLeftStopTime = 0;
			motor_speed_data.speed[0] = 0;
			motor_speed_data.speed[1] = 0;
		}
		if ((CurrentMillis >= MotorRightStopTime) && (MotorRightStopTime > 0))
		{
			MotorRightStopTime = 0;
			motor_speed_data.speed[2] = 0;
			motor_speed_data.speed[3] = 0;
		}
		if (motor_speed_data.whole_speed == 0)
		{
			MotorMode = DC_NONE;
		}
	}
    if(CurrentMillis - voltagePreviousMillis >= VOLTAGE_CHECK_TIMEOUT)
    {
        voltagePreviousMillis = CurrentMillis;
        powerVoltage = analogRead(POWER_SAMPLE_PIN) * 3300 * 4 / 1024;

		packet_01_data.data.deviceEvent = powerVoltage;
    }

	if (PreMode != MotorMode)
	{// Mode starting or stop.
		if (MotorMode == DC_MOTOR)
		{// Motor start.
			MotortimerDiv = 0;
			Timer14.init((MOTOR_MAX_PULSE/MAX_ABS_SPEED)-1);    // 1 tick, 10 us, 100 KHz
			Timer14.attachInterrupt(MotortimerIsr);
		}
		if (MotorMode == DC_DRIVE)
		{// Drive start. Timer 14 prescaler in 10us.
            TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
            
            TIM_DeInit(TIM14);
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
            TIM_TimeBaseStructure.TIM_Period = DV_rpm-1;
            TIM_TimeBaseStructure.TIM_Prescaler = 0;
            TIM_TimeBaseStructure.TIM_ClockDivision = 0;
            TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
            TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
            TIM_PrescalerConfig(TIM14, 479, TIM_PSCReloadMode_Immediate);

			Timer14.attachInterrupt(DrivetimerIsr);
		}
		if (MotorMode == DC_NONE)
		{// stop
			Timer14.detachInterrupt();
        /*    digitalWrite(GROVE_CH1_PWMB, LOW);
            digitalWrite(GROVE_CH1_PWMA, LOW);
            digitalWrite(GROVE_CH2_PWMB, LOW);
            digitalWrite(GROVE_CH2_PWMA, LOW);*/
		}
		PreMode = MotorMode;
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

    switch(commandReceive)
    {
        case I2C_CMD_WHEEL_RUN:
            commandReceive = I2C_CMD_NULL;
			if (i<5)break;
			MotorMode = DC_MOTOR;
			MotorLeftStopTime  = (receiveBuffer[3] + receiveBuffer[4] * 256);
			if (MotorLeftStopTime > 0)
			{
				MotorLeftStopTime += millis();
			}
			MotorRightStopTime = MotorLeftStopTime;
            setDuty((int8_t)receiveBuffer[1], (int8_t)receiveBuffer[2]);

        break;

        case I2C_CMD_WHEEL_MOTION:
            commandReceive = I2C_CMD_NULL;
			if (i<5)break;
			MotorMode = DC_MOTOR;
			MotorLeftStopTime  = (receiveBuffer[3] + receiveBuffer[4] * 256);
			if (MotorLeftStopTime > 0)
			{
				MotorLeftStopTime += millis();
			}
			MotorRightStopTime = MotorLeftStopTime;
            setMotion(receiveBuffer[1], receiveBuffer[2]);
        break;

        case I2C_CMD_MOTOR_LEFT_RUN:
            commandReceive = I2C_CMD_NULL;
			if (i<4)break;
			MotorMode = DC_MOTOR;
			MotorLeftStopTime = (receiveBuffer[2] + receiveBuffer[3] * 256);
			if (MotorLeftStopTime > 0)
			{
				MotorLeftStopTime += millis();
			}
            setDutyLeft((int8_t)receiveBuffer[1]);

        break;

        case I2C_CMD_MOTOR_RIGHT_RUN:
            commandReceive = I2C_CMD_NULL;
			if (i<4)break;
			MotorMode = DC_MOTOR;
			MotorRightStopTime = (receiveBuffer[2] + receiveBuffer[3] * 256);
			if (MotorRightStopTime > 0)
			{
				MotorRightStopTime += millis();
			}
            setDutyRight((int8_t)receiveBuffer[1]);

        break;

        case I2C_CMD_SET_SERVO_ANGLE:
            commandReceive = I2C_CMD_NULL;
			if (i<3)break;
            if (receiveBuffer[1] >= SERVO_NUMBER)break;
            if (receiveBuffer[2] > SERVO_MAX_USER_ANGLE)receiveBuffer[2] = SERVO_MAX_USER_ANGLE;
            current_angle[receiveBuffer[1]] = receiveBuffer[2];
			
            
        break;

        case I2C_CMD_SET_MOTOR_PWM:
            commandReceive = I2C_CMD_NULL;
			if (i<3)break;
            if (((receiveBuffer[1] + receiveBuffer[2]*256) <= MOTOR_MAX_PULSE) && 
			    ((receiveBuffer[1] + receiveBuffer[2]*256) >= MOTOR_MIN_PULSE) )
			{// Is on ticks, not on us.
				pwm = receiveBuffer[1] + receiveBuffer[2]*256;
			}
            
        break;

        case I2C_CMD_SET_DRIVE_STEP:
            commandReceive = I2C_CMD_NULL;
			if (i<2)break;
			if (receiveBuffer[1] == 1)
			{
				DriveStep = DRIVE_STEP_4;
			}else
            {
                DriveStep = DRIVE_STEP_8;
            }
            digitalWrite(GROVE_CH1_PWMA, DV_STEP[DriveStep-4][0]);
            digitalWrite(GROVE_CH1_PWMB, DV_STEP[DriveStep-4][1]);
            digitalWrite(GROVE_CH2_PWMA, DV_STEP[DriveStep-4][2]);
            digitalWrite(GROVE_CH2_PWMB, DV_STEP[DriveStep-4][3]);
            MotortimerDiv = 1;

        break;

        case I2C_CMD_SET_DRIVE_RPM:
            commandReceive = I2C_CMD_NULL;
			if (i<2)break;
			if (receiveBuffer[1] == 0)break;
            if (DriveStep == DRIVE_STEP_8)
            {// 0.9 per step.
                if (receiveBuffer[1] <= 100)DV_rpm = 30000/receiveBuffer[1];
            }else
            {// 1.8 per step.
                if (receiveBuffer[1] <= 180)DV_rpm = 60000/receiveBuffer[1];
            }
			

        break;

        case I2C_CMD_DRIVE_RUN:
            commandReceive = I2C_CMD_NULL;
			if (i<4)break;
            DV_run = MotortimerDiv+(receiveBuffer[2]+receiveBuffer[3]*256);
			if (receiveBuffer[1] == 1)
            {
                DV_dir = 1;
            }else
            {
                DV_dir = 0;
            }
        //    MotortimerDiv = 0;
			MotorMode = DC_DRIVE;

        break;

        case I2C_CMD_SET_ADDR:
            deviceI2CAddress = receiveBuffer[1];
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

        case I2C_CMD_GET_VOLT:
            Wire.write(powerVoltage & 0xff);
            Wire.write((powerVoltage >> 8) & 0xff);
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_TEST_GET_VER:
            Wire.write((char *)(&NodeVersion), 2);
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

void setDutyLeft(int left)
{
    if (left > MAX_ABS_SPEED)left = MAX_ABS_SPEED;
    else if (left < -MAX_ABS_SPEED)left = -MAX_ABS_SPEED;

    if(left < 0) {
        motor_speed_data.speed[0] = MAX_ABS_SPEED;
        motor_speed_data.speed[1] = MAX_ABS_SPEED + left;
    }
    else {
        motor_speed_data.speed[0] = MAX_ABS_SPEED - left;
        motor_speed_data.speed[1] = MAX_ABS_SPEED;
    }
}

void setDutyRight(int right)
{
    if (right > MAX_ABS_SPEED)right = MAX_ABS_SPEED;
    else if (right < -MAX_ABS_SPEED)right = -MAX_ABS_SPEED;

    if(right < 0) {
        motor_speed_data.speed[2] = MAX_ABS_SPEED;
        motor_speed_data.speed[3] = MAX_ABS_SPEED + right;
    }
    else {
        motor_speed_data.speed[2] = MAX_ABS_SPEED - right;
        motor_speed_data.speed[3] = MAX_ABS_SPEED;
    }
}
void setDuty(int left, int right)
{
    setDutyLeft(left);
    setDutyRight(right);
}
void setMotion(uint8_t speed, uint8_t direction)
{
    int left = 0, right = 0;

    if(speed == Slow){left = 40; right = 40;}
    else if(speed == Medium){left = 70; right = 70;}
    else if(speed == Fast){left = MAX_ABS_SPEED; right = MAX_ABS_SPEED;}

    if(direction == Straight){}
    else if(direction == Back){left = (-1) * left; right = (-1) * right;}
    else if(direction == Left){left = 0;}
    else if(direction == Right){right = 0;}
    else if(direction == Clockwise){right = -right;}
    else if(direction == Anticlockwise){left = -left;}

    setDuty(left, right);
}

/***************************************************************
 Device driver
 ***************************************************************/

void ServotimerIsr(void)
{
	uint8_t i;
	
    timerDiv ++;
	if (timerDiv < SERVO_MIN_US)
	{
		return;
	}
    if(timerDiv >= SERVO_MAX_US)
    {
        timerDiv = 0;
        for(i=0;i<SERVO_NUMBER;i++)digitalWrite(servo_pins[i], HIGH);
    }else if (timerDiv <= (SERVO_MIN_US+SERVO_MAX_ANGLE))
    {
		for(i=0;i<SERVO_NUMBER;i++)
		{
			if ((timerDiv-SERVO_MIN_US) == current_angle[i])
				digitalWrite(servo_pins[i], LOW);
		}
    }

	
}


void MotortimerIsr(void)
{
	uint8_t i;
	
    MotortimerDiv ++;
    if ((MotortimerDiv%MAX_ABS_SPEED) == 0)
    {
		MotortimerDiv = 0;
        for(i=0;i<MOTOR_PINS;i++)digitalWrite(motor_pins[i], HIGH);
    }
	for(i=0;i<MOTOR_PINS;i++)
	{
		if (MotortimerDiv == motor_speed_data.speed[i])
			digitalWrite(motor_pins[i], LOW);
	}
}

void DrivetimerIsr(void)
{
    if (DV_dir)
    {// Clock.
        digitalWrite(GROVE_CH1_PWMA, DV_STEP[(MotortimerDiv&(DriveStep-1))+DriveStep-4][0]);
        digitalWrite(GROVE_CH1_PWMB, DV_STEP[(MotortimerDiv&(DriveStep-1))+DriveStep-4][1]);
        digitalWrite(GROVE_CH2_PWMA, DV_STEP[(MotortimerDiv&(DriveStep-1))+DriveStep-4][2]);
        digitalWrite(GROVE_CH2_PWMB, DV_STEP[(MotortimerDiv&(DriveStep-1))+DriveStep-4][3]);
    }else
    {// Anti-clock.
        digitalWrite(GROVE_CH1_PWMA, DV_STEP[DriveStep-1-(MotortimerDiv&(DriveStep-1))+DriveStep-4][0]);
        digitalWrite(GROVE_CH1_PWMB, DV_STEP[DriveStep-1-(MotortimerDiv&(DriveStep-1))+DriveStep-4][1]);
        digitalWrite(GROVE_CH2_PWMA, DV_STEP[DriveStep-1-(MotortimerDiv&(DriveStep-1))+DriveStep-4][2]);
        digitalWrite(GROVE_CH2_PWMB, DV_STEP[DriveStep-1-(MotortimerDiv&(DriveStep-1))+DriveStep-4][3]);
    }

    MotortimerDiv ++;
}
