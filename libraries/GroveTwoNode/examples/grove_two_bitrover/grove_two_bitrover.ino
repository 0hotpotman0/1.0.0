#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>
#include <Timer14.h>

#define BIT_ROVER_PWM       PA4 // TIM14 - CH1
/***************************************************************
 Board defines
 ***************************************************************/

/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS      0x56
#define DEVICE_VID              0x2886
#define DEVICE_PID              0x8007

#define I2C_DEF_ADDR_FLASH_LOC  0x00
#define I2C_CUR_ADDR_FLASH_LOC  0x01

#define I2C_CMD_GET_DEV_ID      0x00
#define I2C_CMD_GET_DEV_EVENT   0x01

#define I2C_CMD_START           0x02
#define I2C_CMD_STOP        	0x03
#define I2C_CMD_SET_PERIOD      0x04
#define I2C_CMD_SET_LENGTH      0x05
#define I2C_CMD_SET_VALID       0x06

#define I2C_CMD_SET_ADDR        0xc0
#define I2C_CMD_RST_ADDR        0xc1
#define I2C_CMD_TEST_GET_VER    0xe2
#define I2C_CMD_GET_VOLT        0xe3

#define I2C_CMD_JUMP_TO_BOOT    0xf0
#define I2C_CMD_GET_DEVICE_UID  0xf1
#define I2C_CMD_NULL            0xff

uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS;
uint8_t commandReceive = I2C_CMD_NULL;

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
void PulsetimerIsr(void);
void PulsefrEvent(void);
void PulseflEvent(void);
void PulsebrEvent(void);
void PulseblEvent(void);
/***************************************************************
Basic defines
 ***************************************************************/
#define BIT_ROVER_FRONT_R   PA0
#define BIT_ROVER_FRONT_L   PA2
#define BIT_ROVER_BACK_R    PA5
#define BIT_ROVER_BACK_L    PA7

#define BIT_ROVER_PIN_F_R   PA1
#define BIT_ROVER_PIN_F_L   PA3
#define BIT_ROVER_PIN_B_R   PA6
#define BIT_ROVER_PIN_B_L   PB1
#define PIN_NUMBER_COUNT    4
uint8_t pin_input_output[PIN_NUMBER_COUNT*2] = {
    BIT_ROVER_FRONT_R, BIT_ROVER_PIN_F_R,
    BIT_ROVER_FRONT_L, BIT_ROVER_PIN_F_L,
    BIT_ROVER_BACK_R,  BIT_ROVER_PIN_B_R,
    BIT_ROVER_BACK_L,  BIT_ROVER_PIN_B_L
};
uint8_t InputChecked=1;
uint16_t timerDiv = 0;
uint16_t intStartfr = 0, intEndfr = 0, intStartfl = 0, intEndfl = 0;
uint16_t intStartbr = 0, intEndbr = 0, intStartbl = 0, intEndbl = 0;

uint16_t NodeVersion = 0x6101;

/***************************************************************/
#define PULSE_PERIOLD       12       // 13 us for 38.5 KHz. 1000/(38.5*2)=13.
#define PULSE_LENGTH        1000     // 1 ms.
#define PULSE_20MS_LENGTH   20000    // 20 ms.
#define VALID_PULSE_LENGTH  800      // 800 us.
/***************************************************************/
uint16_t PulsePeriod = (PULSE_LENGTH+(PULSE_PERIOLD*3))/(PULSE_PERIOLD+1);
uint16_t PulseLength = PULSE_20MS_LENGTH/(PULSE_PERIOLD+1);
uint16_t PulseValid  = VALID_PULSE_LENGTH/(PULSE_PERIOLD+1);

uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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

	pinMode(BIT_ROVER_PWM, OUTPUT);

	pinMode(BIT_ROVER_FRONT_R, INPUT);
	pinMode(BIT_ROVER_FRONT_L, INPUT);
	pinMode(BIT_ROVER_BACK_R, INPUT);
	pinMode(BIT_ROVER_BACK_L, INPUT);
    
	attachInterrupt(BIT_ROVER_FRONT_R, PulsefrEvent, CHANGE, INPUT);
    attachInterrupt(BIT_ROVER_FRONT_L, PulseflEvent, CHANGE, INPUT);
    attachInterrupt(BIT_ROVER_BACK_R,  PulsebrEvent, CHANGE, INPUT);
    attachInterrupt(BIT_ROVER_BACK_L,  PulseblEvent, CHANGE, INPUT);
    
	pinMode(BIT_ROVER_PIN_F_R, OUTPUT);
	pinMode(BIT_ROVER_PIN_F_L, OUTPUT);
	pinMode(BIT_ROVER_PIN_B_R, OUTPUT);
	pinMode(BIT_ROVER_PIN_B_L, OUTPUT);
	
	digitalWrite(BIT_ROVER_PIN_F_R, HIGH);
	digitalWrite(BIT_ROVER_PIN_F_L, HIGH);
	digitalWrite(BIT_ROVER_PIN_B_R, HIGH);
	digitalWrite(BIT_ROVER_PIN_B_L, HIGH);
#if 0
    Timer14.init(PULSE_PERIOLD);
	Timer14.attachInterrupt(PulsetimerIsr);
#endif
    Wire.begin(deviceI2CAddress);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    wwdg.begin();
}

void loop()
{
    if ((InputChecked == 1) && (timerDiv > (PulsePeriod*2)))
    {
        if ((intEndfr-intStartfr) > PulseValid)
        {
            digitalWrite(BIT_ROVER_PIN_F_R, LOW);
        }else
        {
            digitalWrite(BIT_ROVER_PIN_F_R, HIGH);
        }
        if ((intEndfl-intStartfl) > PulseValid)
        {
            digitalWrite(BIT_ROVER_PIN_F_L, LOW);
        }else
        {
            digitalWrite(BIT_ROVER_PIN_F_L, HIGH);
        }
        if ((intEndbr-intStartbr) > PulseValid)
        {
            digitalWrite(BIT_ROVER_PIN_B_R, LOW);
        }else
        {
            digitalWrite(BIT_ROVER_PIN_B_R, HIGH);
        }
        if ((intEndbl-intStartbl) > PulseValid)
        {
            digitalWrite(BIT_ROVER_PIN_B_L, LOW);
        }else
        {
            digitalWrite(BIT_ROVER_PIN_B_L, HIGH);
        }
        InputChecked = 0;
    }
    if (timerDiv >= PulseLength)
    {
        timerDiv    = 0;
        intStartfr  = 0;
        intEndfr    = 0;
        intStartfl  = 0;
        intEndfl    = 0;
        intStartbr  = 0;
        intEndbr    = 0;
        intStartbl  = 0;
        intEndbl    = 0;
        InputChecked = 1;
    }
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
    else if(commandReceive == I2C_CMD_START) // start pulse send.
    {
        commandReceive = I2C_CMD_NULL;
        Timer14.init(PULSE_PERIOLD);
        Timer14.attachInterrupt(PulsetimerIsr);
    }
    else if(commandReceive == I2C_CMD_STOP) // reset i2c address
    {
        commandReceive = I2C_CMD_NULL;
        TIM_DeInit(TIM14);
    }

    wwdg.reset();
}

void receiveEvent(int howMany)
{
    uint8_t i = 0, receiveBuffer[MAINBOARD_BLE_I2C_DATALEN] = {0,};

    while(Wire.available())
    {
        receiveBuffer[i ++] = Wire.read();
        if(i >= MAINBOARD_BLE_I2C_DATALEN)i = 0;
    }
    commandReceive = receiveBuffer[0];

    switch(commandReceive)
    {
        case I2C_CMD_SET_PERIOD:
            if (i > 2)PulsePeriod = (receiveBuffer[1]+receiveBuffer[2]*256+(PULSE_PERIOLD*3))/(PULSE_PERIOLD+1);
        break;

        case I2C_CMD_SET_LENGTH:
            if (i > 2)PulseLength = (receiveBuffer[1]+receiveBuffer[2]*256)/(PULSE_PERIOLD+1);
        break;

        case I2C_CMD_SET_VALID:
            if (i > 2)PulseValid = (receiveBuffer[1]+receiveBuffer[2]*256)/(PULSE_PERIOLD+1);
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
 Pulse function, interrupt per 13us, output 38.5 KHz pulse for 1 ms.
 ***************************************************************/
void PulsetimerIsr(void)
{
    timerDiv ++;
    if (timerDiv <= PulsePeriod)
    {
        digitalWrite(BIT_ROVER_PWM, timerDiv&1);
    }
}

void PulsefrEvent()
{
    if(digitalRead(BIT_ROVER_FRONT_R) == LOW)intStartfr = timerDiv;
    else intEndfr = timerDiv;
}

void PulseflEvent()
{
    if(digitalRead(BIT_ROVER_FRONT_L) == LOW)intStartfl = timerDiv;
    else intEndfl = timerDiv;
}

void PulsebrEvent()
{
    if(digitalRead(BIT_ROVER_BACK_R) == LOW)intStartbr = timerDiv;
    else intEndbr = timerDiv;
}

void PulseblEvent()
{
    if(digitalRead(BIT_ROVER_BACK_L) == LOW)intStartbl = timerDiv;
    else intEndbl = timerDiv;
}

