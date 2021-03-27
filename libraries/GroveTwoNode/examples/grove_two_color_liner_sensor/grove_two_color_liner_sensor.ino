
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>


#define GROVE_TX_PIN_NUM	PA2
#define GROVE_RX_PIN_NUM	PA3

#define GROVE_LED_PIN_NUM		PA1

/***************************************************************
 Board defines
 ***************************************************************/
#define COLOR_POWER_PIN_NUM     PF0
#define COLOR_LED_PIN_NUM 		PF1
#define SOFT_SCL_PIN	        PA14
#define SOFT_SDA_PIN	        PA13
#define COLOR_BUTTON_PIN_NUM    PB1

#define I2C_DELAY		1
#define I2C_WAIT()		delayMicroseconds(I2C_DELAY)
#define I2C_SDA_IN()	pinMode(SOFT_SDA_PIN, INPUT)
#define I2C_SDA_OUT()	pinMode(SOFT_SDA_PIN, OUTPUT)
#define I2C_SDA_READ()	digitalRead(SOFT_SDA_PIN)
#define I2C_SCL_LOW()	digitalWrite(SOFT_SCL_PIN, LOW)
#define I2C_SCL_HIGH()	digitalWrite(SOFT_SCL_PIN, HIGH)
#define I2C_SDA_LOW()	digitalWrite(SOFT_SDA_PIN, LOW)
#define I2C_SDA_HIGH()	digitalWrite(SOFT_SDA_PIN, HIGH)

void i2cStart(void);
void i2cStop(void);
bool i2cWaitAck(void);
void i2cAck(void);
void i2cNAck(void);
void i2cSendByte(uint8_t txd);
uint8_t i2cReadByte(void);

#define TCS34725_I2C_ADDR    0x29

#define TCS34725_COMMAND_BIT      (0x80)

#define TCS34725_ENABLE           (0x00)
#define TCS34725_ENABLE_AIEN      (0x10)    /* RGBC Interrupt Enable */
#define TCS34725_ENABLE_WEN       (0x08)    /* Wait enable - Writing 1 activates the wait timer */
#define TCS34725_ENABLE_AEN       (0x02)    /* RGBC Enable - Writing 1 actives the ADC, 0 disables it */
#define TCS34725_ENABLE_PON       (0x01)    /* Power on - Writing 1 activates the internal oscillator, 0 disables it */
#define TCS34725_ATIME            (0x01)    /* Integration time */
#define TCS34725_WTIME            (0x03)    /* Wait time (if TCS34725_ENABLE_WEN is asserted) */
#define TCS34725_WTIME_2_4MS      (0xFF)    /* WLONG0 = 2.4ms   WLONG1 = 0.029s */
#define TCS34725_WTIME_204MS      (0xAB)    /* WLONG0 = 204ms   WLONG1 = 2.45s  */
#define TCS34725_WTIME_614MS      (0x00)    /* WLONG0 = 614ms   WLONG1 = 7.4s   */
#define TCS34725_AILTL            (0x04)    /* Clear channel lower interrupt threshold */
#define TCS34725_AILTH            (0x05)
#define TCS34725_AIHTL            (0x06)    /* Clear channel upper interrupt threshold */
#define TCS34725_AIHTH            (0x07)
#define TCS34725_PERS             (0x0C)    /* Persistence register - basic SW filtering mechanism for interrupts */
#define TCS34725_PERS_NONE        (0b0000)  /* Every RGBC cycle generates an interrupt                                */
#define TCS34725_PERS_1_CYCLE     (0b0001)  /* 1 clean channel value outside threshold range generates an interrupt   */
#define TCS34725_PERS_2_CYCLE     (0b0010)  /* 2 clean channel values outside threshold range generates an interrupt  */
#define TCS34725_PERS_3_CYCLE     (0b0011)  /* 3 clean channel values outside threshold range generates an interrupt  */
#define TCS34725_PERS_5_CYCLE     (0b0100)  /* 5 clean channel values outside threshold range generates an interrupt  */
#define TCS34725_PERS_10_CYCLE    (0b0101)  /* 10 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_15_CYCLE    (0b0110)  /* 15 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_20_CYCLE    (0b0111)  /* 20 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_25_CYCLE    (0b1000)  /* 25 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_30_CYCLE    (0b1001)  /* 30 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_35_CYCLE    (0b1010)  /* 35 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_40_CYCLE    (0b1011)  /* 40 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_45_CYCLE    (0b1100)  /* 45 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_50_CYCLE    (0b1101)  /* 50 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_55_CYCLE    (0b1110)  /* 55 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_60_CYCLE    (0b1111)  /* 60 clean channel values outside threshold range generates an interrupt */
#define TCS34725_CONFIG           (0x0D)
#define TCS34725_CONFIG_WLONG     (0x02)    /* Choose between short and long (12x) wait times via TCS34725_WTIME */
#define TCS34725_CONTROL          (0x0F)    /* Set the gain level for the sensor */
#define TCS34725_ID               (0x12)    /* 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_STATUS           (0x13)
#define TCS34725_STATUS_AINT      (0x10)    /* RGBC Clean channel interrupt */
#define TCS34725_STATUS_AVALID    (0x01)    /* Indicates that the RGBC channels have completed an integration cycle */
#define TCS34725_CDATAL           (0x14)    /* Clear channel data */
#define TCS34725_CDATAH           (0x15)
#define TCS34725_RDATAL           (0x16)    /* Red channel data */
#define TCS34725_RDATAH           (0x17)
#define TCS34725_GDATAL           (0x18)    /* Green channel data */
#define TCS34725_GDATAH           (0x19)
#define TCS34725_BDATAL           (0x1A)    /* Blue channel data */
#define TCS34725_BDATAH           (0x1B)

typedef enum
{
  TCS34725_INTEGRATIONTIME_2_4MS  = 0xFF,   /**<  2.4ms - 1 cycle    - Max Count: 1024  */
  TCS34725_INTEGRATIONTIME_24MS   = 0xF6,   /**<  24ms  - 10 cycles  - Max Count: 10240 */
  TCS34725_INTEGRATIONTIME_50MS   = 0xEB,   /**<  50ms  - 20 cycles  - Max Count: 20480 */
  TCS34725_INTEGRATIONTIME_101MS  = 0xD5,   /**<  101ms - 42 cycles  - Max Count: 43008 */
  TCS34725_INTEGRATIONTIME_154MS  = 0xC0,   /**<  154ms - 64 cycles  - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_700MS  = 0x00    /**<  700ms - 256 cycles - Max Count: 65535 */
}
tcs34725IntegrationTime_t;

typedef enum
{
  TCS34725_GAIN_1X                = 0x00,   /**<  No gain  */
  TCS34725_GAIN_4X                = 0x01,   /**<  4x gain  */
  TCS34725_GAIN_16X               = 0x02,   /**<  16x gain */
  TCS34725_GAIN_60X               = 0x03    /**<  60x gain */
}
tcs34725Gain_t;

void write8 (uint8_t reg, uint8_t value);
uint8_t read8 (uint8_t reg);
uint16_t read16 (uint8_t reg);
void readBuf (uint8_t reg, uint8_t *buf, uint8_t len);
void initTCS34725(void);
void enableTCS34725(void);
void disableTCS34725(void);
void setInterrupt(bool flag);
void clearInterrupt(void);
void setIntegrationTime(tcs34725IntegrationTime_t it);
void setGain(tcs34725Gain_t gain);
void getRawData(void);

#define COLOR_SAMPLE_INTERVAL    25 // 25 ms

#if 0
uint16_t blackColorOffset[4] = {0x00A0, 0x0020, 0x0040, 0x0060}; 
uint16_t blackLightOffset[4] = {0x0020, 0x0020, 0x0020, 0x0020}; 
uint16_t whiteColorOffset[4] = {0x0900, 0x0180, 0x0340, 0x04C0}; 
uint16_t whiteLightOffset[4] = {0x0180, 0x0180, 0x0180, 0x0160}; 
#else
uint16_t blackColorOffset[4] = {0x0090, 0x0010, 0x0020, 0x0030}; 
uint16_t blackLightOffset[4] = {0x0010, 0x0010, 0x0010, 0x0010}; 
uint16_t whiteColorOffset[4] = {0x0500, 0x0100, 0x0200, 0x0300}; 
uint16_t whiteLightOffset[4] = {0x0100, 0x0100, 0x0100, 0x00E0}; 
#endif
/*
133	29	6D	A9	31	37	39	2F	998	1B6	37B	4CA	17E	172	194	15E

1. 8A	19	33	49	20	1C	18	23	55E	F0	1F8	2D0	F5	F2	E8	B3	
5. AB	21	3C	55	18	1C	19	13	6E3	139	256	363	D5	100	F4	C6	
8. A9	20	3B	51	17	1C	1D	19	61F	11D	219	2F3	DF	10E	11D	105	
2. 81	19	2D	3E	1F	19	1B	16	573	FC	1DB	29E	112	100	F1	C9	


AD	20	3B	52	15	1A	20	19	5F9	112	206	2DE	10E	110	130	F9	
8F	1B	31	45	1D	19	17	12	50F	EA	1BE	270	100	E9	FA	CF	

A0	20	40	50	20	20	20	20	A03	1BA	39B	50B	17E	180	19C	170	
A0	20	40	50	20	20	20	20	9BE	1A5	36F	501	16D	17A	1A0	167

102	2A	5C	85	25	2B	33	2B	9A4	194	37C	502	176	1AA	1CA	17D	

uint16_t blackColorOffset[4] = {0x00A0, 0x0020, 0x0040, 0x0050}; 
uint16_t blackLightOffset[4] = {0x0020, 0x0020, 0x0020, 0x0020}; 
uint16_t whiteColorOffset[4] = {0x0600, 0x0120, 0x0200, 0x0300}; 
uint16_t whiteLightOffset[4] = {0x0100, 0x0110, 0x0120, 0x0100}; 

A4	1F	37	4D	1A	17	1C	1C	62F	126	210	2EB	11F	DA	F9	10B	
B7	21	3E	58	15	1F	22	15	6C1	132	24B	34D	ED	116	138	BE	
9B	1C	31	48	15	14	1A	18	5C6	106	1D9	2BA	DA	D4	EE	DC	



BD	22	40	5B	1D	1E	23	16	6A5	12D	245	341	E8	11E	132	D6	
95	1B	2F	46	16	14	1A	17	5D4	107	1DC	2C1	DE	D7	ED	DC	
A1	1F	36	4B	19	14	1C	1E	654	129	20F	2F5	125	EA	10B	11A	
A4	1F	38	4C	1B	18	1A	1B	669	12E	227	2FF	DA	124	F1	112	
95	1B	33	4A	18	1D	21	25	5FA	10F	209	2F7	119	118	123	116	
D9	23	47	6E	25	18	26	23	8B0	163	2D5	473	EE	111	14D	157	
AD	20	37	4F	18	19	20	1C	6BB	132	229	322	F0	F9	139	116	
*/

uint32_t sampleTimerPreviousMillis = 0;
uint16_t clearCode = 0, redCode = 0, greenCode = 0, blueCode = 0;
uint16_t clear = 0, red = 0, green = 0, blue = 0, clearCodeTemp = 0;
uint32_t color = 0, colorTemp = 0;

// color sample time 2.4 ms
#define CLEAR_CODE_LIGHT_ON    50
#define CLEAR_CODE_LIGHT_OFF   10

#define LIGHT_LA_PIN    PA0
#define LIGHT_LB_PIN    PA7
#define LIGHT_RB_PIN    PA6
#define LIGHT_RA_PIN    PA5
#define LIGTH_LED_PIN   PA4

#define COLOR_ON_THD_MAN	   50
#define COLOR_ON_THD_MIN	   10
#define COLOR_OFF_THD_MAN	   20
#define COLOR_OFF_THD_MIN	   5

#define LIGHT_ON_THD_MAN	   50
#define LIGHT_ON_THD_MIN	   10
#define LIGHT_OFF_THD_MAN	   20
#define LIGHT_OFF_THD_MIN	   5

#define LINER_STRAIGHT      1
#define LINER_END           2
#define LINER_LEFT_LV1      3
#define LINER_LEFT_LV2      4
#define LINER_RIGHT_LV1     5
#define LINER_RIGHT_LV2     6

bool lightMode = true;
uint8_t lightEvent = 0, lightBits = 0, ErrorCount = 0;
uint16_t lightLA = 0, lightLB = 0, lightRA = 0, lightRB = 0;

bool updateFlag = false, flashFlag = false;
uint8_t lightEventTemp = 0, lightBitsTemp = 0;
uint16_t lightLATemp = 0, lightLBTemp = 0, lightRATemp = 0, lightRBTemp = 0;

#define LIGHT_CHECK_TIMEOUT	1000

uint32_t lightCheckPreviousMillis = 0;

/***************************************************************
 Communication defines
 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x27
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x0010

#define I2C_DEF_ADDR_FLASH_LOC	0x00
#define I2C_CUR_ADDR_FLASH_LOC	0x01

#define I2C_CMD_GET_DEV_ID		0x00 // 
#define I2C_CMD_GET_COLOR_EVENT	0x01 // 
#define I2C_CMD_GET_LINER_EVENT	0x02 // 
#define I2C_CMD_GET_LINER_BITS	0x03 // 
#define I2C_CMD_GET_COLOR_DATA	0x04 // 
#define I2C_CMD_GET_LINER_DATA	0x05 // 
#define I2C_CMD_LIGHT_ON        0x06 // 
#define I2C_CMD_LIGHT_OFF       0x07 // 
#define I2C_CMD_SET_BLACK_OFFSET	0x08 //
#define I2C_CMD_SET_WHITE_OFFSET	0x09 // 
#define I2C_CMD_LED_ON			0xb0 // 
#define I2C_CMD_LED_OFF			0xb1 // 
#define I2C_CMD_AUTO_SLEEP_ON	0xb2 // 
#define I2C_CMD_AUTO_SLEEP_OFF	0xb3 // 
#define I2C_CMD_SET_ADDR		0xc0 //
#define I2C_CMD_RST_ADDR		0xc1 // 
#define I2C_CMD_TEST_TX_RX_ON   0xe0 // 
#define I2C_CMD_TEST_TX_RX_OFF  0xe1 // 
#define I2C_CMD_TEST_GET_VER    0xe2 // 
#define I2C_CMD_JUMP_TO_BOOT	0xf0 // 
#define I2C_CMD_GET_DEVICE_UID  0xf1 // 
#define I2C_CMD_NULL			0xff // 

uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS;
uint8_t commandReceive = I2C_CMD_NULL;
uint8_t	ledFlashTimes = 0;

#ifdef BLE_SUPPORT

#define I2C_CMD_BLE_SET_BLACK_OFFSET	0x90 //
#define I2C_CMD_BLE_SET_WHITE_OFFSET	0x91 // 

uint8_t Core_mode = 0;
uint32_t StartMillis = 1;
uint32_t preEvent;

typedef struct
{
	uint8_t Datalen;
	uint8_t type;
	uint8_t Address;
	uint8_t option;
}packet_header_t;

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
		packet_got_atr	atr;
		uint8_t Option[MAINBOARD_BLE_I2C_DATALEN-4];
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
	uint8_t		color;
	uint8_t		liner;
	uint8_t		red;
	uint8_t		green;
	uint8_t		blue;
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
uint32_t RawDelayMillis = 49;
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

#define AUTO_SLEEP_TIMEOUT	2000

uint32_t autoSleepPreviousMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

#define LED_FLASH_TIME	250

bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0, PreMillis = 0;

bool testFlag = false;
char *versions = "V20";
uint16_t NodeVersion = 0x6100;

uint32_t intStart = 0;
uint32_t intEnd = 0;

bool offset_mode = false;
uint32_t offset_mode_start_time = 0;
typedef enum {
    COLOR_BUTTON_IDOL = 0,
    COLOR_BUTTON_SINGLE_CLICK,
    COLOR_BUTTON_DOUBLE_CLICK,
    COLOR_BUTTON_LONG_CLICK,
} 
button_state_t;

#define COLOR_BUTTON_LONG_CLICK_TIMES  750
uint32_t checkButtonLastMillis = 0;
// #define DEBUG
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
    
    uint8_t *ptr3 = (uint8_t *)Flash.getChipUniqueID();
    for(uint8_t i = 0; i < 12; i ++)chipId[i] = *(ptr3 + i);
	
	if(i2cDefaultAddr == 0xff)Flash.write8(I2C_DEF_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	if(i2cCurrentAddr == 0xff)Flash.write8(I2C_CUR_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	else deviceI2CAddress = i2cCurrentAddr;
	
	packet_01_data.data.deviceVID = DEVICE_VID;
	packet_01_data.data.devicePID = DEVICE_PID;
	packet_01_data.data.deviceEvent = 0;
    
    loadOffsetData();
	
	nrgSave.begin(GROVE_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance
#ifdef DEBUG
    Serial.begin(115200);
#endif

	pinMode(GROVE_LED_PIN_NUM, OUTPUT);
	digitalWrite(GROVE_LED_PIN_NUM, HIGH);
    
    pinMode(COLOR_POWER_PIN_NUM, OUTPUT);
    digitalWrite(COLOR_POWER_PIN_NUM, HIGH);
    
    pinMode(LIGTH_LED_PIN, OUTPUT);
    digitalWrite(LIGTH_LED_PIN, HIGH);
    
	pinMode(COLOR_LED_PIN_NUM, OUTPUT);
    digitalWrite(COLOR_LED_PIN_NUM, HIGH);

    pinMode(COLOR_BUTTON_PIN_NUM, INPUT);

    pinMode(SOFT_SCL_PIN, OUTPUT);
	pinMode(SOFT_SDA_PIN, OUTPUT);
	digitalWrite(SOFT_SCL_PIN, HIGH);
	digitalWrite(SOFT_SDA_PIN, HIGH);
	
#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif
	    
    initTCS34725();
	
	Wire.begin(deviceI2CAddress);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);
  
	wwdg.begin();
}

void loop()
{    
    uint32_t sampleTimerCurrentMillis = millis();
			
	if(Wire.isbusidle())PreMillis = sampleTimerCurrentMillis;
	if ((sampleTimerCurrentMillis - PreMillis) > 20)
	{
		Wire.end();
		Wire.begin(deviceI2CAddress);
		Wire.onReceive(receiveEvent);
		Wire.onRequest(requestEvent);
		PreMillis = sampleTimerCurrentMillis;
	}
    
    if(sampleTimerCurrentMillis - sampleTimerPreviousMillis >= COLOR_SAMPLE_INTERVAL)
    {
        sampleTimerPreviousMillis = sampleTimerCurrentMillis;
        updateFlag = false;
        
        getRawData();
        getRawLight();
        
        if(clearCode > blackColorOffset[0])clearCode = clearCode - blackColorOffset[0];
        else clearCode = 0;
        if(redCode > blackColorOffset[1])redCode = redCode - blackColorOffset[1];
        else redCode = 0;
        if(greenCode > blackColorOffset[2])greenCode = greenCode - blackColorOffset[2];
        else greenCode = 0;
        if(blueCode > blackColorOffset[3])blueCode = blueCode - blackColorOffset[3];
        else blueCode = 0;
        
        if(lightLA > blackLightOffset[0])lightLA = lightLA - blackLightOffset[0];
        else lightLA = 0;
        if(lightLB > blackLightOffset[1])lightLB = lightLB - blackLightOffset[1];
        else lightLB = 0;
        if(lightRB > blackLightOffset[2])lightRB = lightRB - blackLightOffset[2];
        else lightRB = 0;
        if(lightRA > blackLightOffset[3])lightRA = lightRA - blackLightOffset[3];
        else lightRA = 0;
        
        // blance light value
        if(whiteLightOffset[0] > 0)lightLA = lightLA * whiteColorOffset[0] / whiteLightOffset[0];
        if(whiteLightOffset[1] > 0)lightLB = lightLB * whiteColorOffset[0] / whiteLightOffset[1];
        if(whiteLightOffset[2] > 0)lightRB = lightRB * whiteColorOffset[0] / whiteLightOffset[2];
        if(whiteLightOffset[3] > 0)lightRA = lightRA * whiteColorOffset[0] / whiteLightOffset[3];

        if(lightLA > (whiteColorOffset[0] * 7 / 10))lightBits = (lightBits & (~0x10));
        else lightBits = (lightBits | 0x10);
        if(lightLB > (whiteColorOffset[0] * 7 / 10))lightBits = (lightBits & (~0x08));
        else lightBits = (lightBits | 0x08);
        if(clearCode > (whiteColorOffset[0] * 7 / 10))lightBits = (lightBits & (~0x04));
        else lightBits = (lightBits | 0x04);
        if(lightRB > (whiteColorOffset[0] * 7 / 10))lightBits = (lightBits & (~0x02));
        else lightBits = (lightBits | 0x02);
        if(lightRA > (whiteColorOffset[0] * 70 / 100))lightBits = (lightBits & (~0x01));
        else lightBits = (lightBits | 0x01);

        uint8_t left = (lightBits >> 2) & 0x07; // LA, LB, Mid
        uint8_t right = ((lightBits >> 2) & 0x01) | (lightBits & 0x02) | ((lightBits & 0x01) << 2); // RA, RB, Mid
        if((left != 0) || (right != 0 ))
        {
            if((lightBits == 0b00100) || (lightBits == 0b01110) || (lightBits == 0b01100) || (lightBits == 0b00110))lightEvent = LINER_STRAIGHT;
            else if((lightBits == 0b01000) || (lightBits == 0b11110) || (lightBits == 0b11100))lightEvent = LINER_LEFT_LV1;
            else if((lightBits == 0b00010) || (lightBits == 0b01111) || (lightBits == 0b00111))lightEvent = LINER_RIGHT_LV1;
            else if((lightBits == 0b11000) || (lightBits == 0b10000))lightEvent = LINER_LEFT_LV2;
            else if((lightBits == 0b00011) || (lightBits == 0b00001))lightEvent = LINER_RIGHT_LV2;
            else lightEvent = LINER_END;
        }
        else lightEvent = LINER_END;

        uint32_t r, g, b;
        
        // add white blance
        r = redCode; g = greenCode; b = blueCode;
        if(whiteColorOffset[1] > 0)r = r * whiteColorOffset[0] / whiteColorOffset[1];
        if(whiteColorOffset[2] > 0)g = g * whiteColorOffset[0] / whiteColorOffset[2];
        if(whiteColorOffset[3] > 0)b = b * whiteColorOffset[0] / whiteColorOffset[3];
        redCode = r; greenCode = g; blueCode = b;
        
        r = redCode << 14; if(whiteColorOffset[0] > 0)r /= whiteColorOffset[0];
        g = greenCode << 14; if(whiteColorOffset[0] > 0)g /= whiteColorOffset[0];
        b = blueCode << 14; if(whiteColorOffset[0] > 0)b /= whiteColorOffset[0];
        // red = r >> 6; green = g >> 6; blue = b >> 6;
        red = r >> 5; green = g >> 5; blue = b >> 5; // color multiply by 2
        if(red > 255)red = 255;
        if(green > 255)green = 255;
        if(blue > 255)blue = 255;
        color = (red << 16) + (green << 8) + blue;
         
        int32_t rp, gp, bp;
        uint16_t offsetLess = whiteColorOffset[0] / 50;
        uint16_t offsetMiddle = whiteColorOffset[0] / 30;
        uint16_t offsetGreater = whiteColorOffset[0] * 7 / 10;
        rp = (r * 1000) / (r + g + b);
        gp = (g * 1000) / (r + g + b);
        bp = (b * 1000) / (r + g + b);
        if(clearCode < offsetLess)
		{
			packet_01_data.data.deviceEvent = 1; // black
		}else if((clearCode > offsetMiddle) && (clearCode < offsetGreater) && 
			((rp - gp) > 120) && ((rp - bp) > 120) && (lightBits != 0b01100) && (lightBits != 0b00110))
		{
			packet_01_data.data.deviceEvent = 2; // red
        }else if((clearCode > offsetMiddle) && (clearCode < offsetGreater) && ((gp - rp) > 100) && ((gp - bp) > 100))
		{
			packet_01_data.data.deviceEvent = 3; // green
		}else if((clearCode > offsetMiddle) && (clearCode < offsetGreater) && ((bp -gp) > 140) && ((bp - rp) > 140))
		{
			packet_01_data.data.deviceEvent = 4; // blue
        }else
		{
			packet_01_data.data.deviceEvent = 5; // other color
		}
        
        updateFlag = true;
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

		uint32_t CurrentMillis = millis();

		if (LightRawData)
		{
			if ((CurrentMillis - RawPreviousMillis) >= RawDelayMillis)
			{
				// Raw data 
				InstructionOption.raw_data.Header.type		= I2C_CMD_GET_RAW_DATA;
				InstructionOption.raw_data.Header.Address	= deviceI2CAddress;
				InstructionOption.raw_data.color	= packet_01_data.data.deviceEvent;
				InstructionOption.raw_data.liner	= lightEvent;
				InstructionOption.raw_data.red		= red;
				InstructionOption.raw_data.green	= green;
				InstructionOption.raw_data.blue		= blue;
				InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);
//				Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw));
				if(Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw)) == 0)
				{
					ErrorCount++;
					RawPreviousMillis += 3; // Retry in 3 ms later.
				}else
				{
					ErrorCount = 0;
					RawPreviousMillis = CurrentMillis;
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
				Wire.MasterGPIOTransmission(ptr2, InstructionOption.atr.Header.Datalen);
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
			case I2C_CMD_BLE_SET_BLACK_OFFSET:
				commandReceive = I2C_CMD_NULL;
				wwdg.end();
				setBlackOffset(commandOption.data.commands.raw.Raw_data_type);
				wwdg.begin();
			case I2C_CMD_BLE_SET_WHITE_OFFSET:
				commandReceive = I2C_CMD_NULL;
				wwdg.end();
				setWhiteOffset(commandOption.data.commands.raw.Raw_data_type);
				wwdg.begin();
			break;
			
			default:
			break;
		}
	}else 
	{
#endif
    
    if(commandReceive == I2C_CMD_SET_BLACK_OFFSET)
    {
        commandReceive = I2C_CMD_NULL;
        wwdg.end();
        setBlackOffset(flashFlag);
        wwdg.begin();
    }
    else if(commandReceive == I2C_CMD_SET_WHITE_OFFSET)
    {
        commandReceive = I2C_CMD_NULL;
        wwdg.end();
        setWhiteOffset(flashFlag);
        wwdg.begin();
    }
	else if(commandReceive == I2C_CMD_SET_ADDR) // change i2c address
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
	
    if (millis() - checkButtonLastMillis >= 2)
    {
        checkButtonLastMillis = millis();
        if (isButtonLongPress()) 
        {
            offset_mode = true;
            offset_mode_start_time = millis();
        }
    }

    if (offset_mode)
    {
        wwdg.end();
        Wire.end();
        pinMode(PA9, INPUT_PULLUP);
        pinMode(PA10, INPUT_PULLUP);
        // LED turn on 
        digitalWrite(GROVE_LED_PIN_NUM, 0);

        // wait the button release
        uint8_t release_count = 0;
        while (release_count < 30)
        {
            if (digitalRead(COLOR_BUTTON_PIN_NUM)) release_count++;
            delay(3);
        }
        

        while (offset_mode) {
            switch(getButtonState())
            {
                case COLOR_BUTTON_SINGLE_CLICK:
#ifdef DEBUG
    Serial.println("single click");
#endif
                    for (int i=0;i<60;i++)
                    {
                        digitalWrite(GROVE_LED_PIN_NUM, 0);
                        delay(25);
                        digitalWrite(GROVE_LED_PIN_NUM, 1);
                        delay(25);
                    }
                    setBlackOffset(true);
                    // for (int i=0;i<30;i++)
                    // {
                    //     digitalWrite(GROVE_LED_PIN_NUM, 0);
                    //     delay(25);
                    //     digitalWrite(GROVE_LED_PIN_NUM, 1);
                    //     delay(25);
                    // }
                    offset_mode_start_time = millis();
                break;

                case COLOR_BUTTON_DOUBLE_CLICK:
#ifdef DEBUG
    Serial.println("double click");
#endif
                    digitalWrite(GROVE_LED_PIN_NUM, 0);
                    delay(300);
                    digitalWrite(GROVE_LED_PIN_NUM, 1);
                    delay(300);
                    digitalWrite(GROVE_LED_PIN_NUM, 0);
                    delay(300);
                    digitalWrite(GROVE_LED_PIN_NUM, 1);
                    delay(300);
                    digitalWrite(GROVE_LED_PIN_NUM, 0);
                    setWhiteOffset(true);
                    delay(300);
                    digitalWrite(GROVE_LED_PIN_NUM, 1);
                    delay(300);
                    digitalWrite(GROVE_LED_PIN_NUM, 0);
                    delay(300);
                    digitalWrite(GROVE_LED_PIN_NUM, 1);
                    delay(300);
                    digitalWrite(GROVE_LED_PIN_NUM, 0);
                    delay(300);
                    offset_mode_start_time = millis();
                break;

                case COLOR_BUTTON_LONG_CLICK:
#ifdef DEBUG
    Serial.println("long click");
#endif
                    offset_mode = false;
                break;

                case COLOR_BUTTON_IDOL:
                default:
                    digitalWrite(GROVE_LED_PIN_NUM, 0);
                break;
            }
            if (millis() - offset_mode_start_time > 10000)
            {
                offset_mode = false;
#ifdef DEBUG
                Serial.println("leave offset mode");
#endif            
            }
        }
        // LED turn off and last one second
        digitalWrite(GROVE_LED_PIN_NUM, 1);
        delay(200);
        // wait to release
        release_count = 0;
        while (release_count < 3)
        {
            if (digitalRead(COLOR_BUTTON_PIN_NUM)) release_count++;
            delay(1);
        }
        Wire.begin(deviceI2CAddress);
        Wire.onReceive(receiveEvent);
        Wire.onRequest(requestEvent);
        wwdg.begin();
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

	if(autoSleepFlag)
	{
		uint32_t autoSleepCurrentMillis = millis();
		if((autoSleepCurrentMillis - autoSleepPreviousMillis) > AUTO_SLEEP_TIMEOUT)
		{
			autoSleepPreviousMillis = autoSleepCurrentMillis;
			
			ledFlashCommand = false;
			ledFlashStatus = false;
			digitalWrite(GROVE_LED_PIN_NUM, HIGH);
            
            disableTCS34725();
            digitalWrite(COLOR_POWER_PIN_NUM, LOW);
            digitalWrite(COLOR_LED_PIN_NUM, LOW);
            digitalWrite(COLOR_LED_PIN_NUM, LOW);
			digitalWrite(LIGTH_LED_PIN, LOW);
			
			wwdg.end();
			Wire.end();
			pinMode(PA9, INPUT_PULLUP);
			pinMode(PA10, INPUT_PULLUP);
			
			nrgSave.standby();

			Wire.begin(deviceI2CAddress);
			Wire.onReceive(receiveEvent);
			Wire.onRequest(requestEvent);
			wwdg.begin();
            
			digitalWrite(LIGTH_LED_PIN, HIGH);
            digitalWrite(COLOR_POWER_PIN_NUM, HIGH);
            digitalWrite(COLOR_LED_PIN_NUM, HIGH);
            digitalWrite(COLOR_LED_PIN_NUM, HIGH);
            enableTCS34725();
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
		for(j=0;j<(i-sizeof(packet_header_t));j++)
		{
			receiveBuffer[j+1] = receiveBuffer[j+sizeof(packet_header_t)];
		}
		i -= (sizeof(packet_header_t)-1);
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
			digitalWrite(GROVE_LED_PIN_NUM, HIGH);
			commandReceive = I2C_CMD_NULL;
			
			break;
#endif	
        case I2C_CMD_BLE_SET_BLACK_OFFSET:
        case I2C_CMD_BLE_SET_WHITE_OFFSET:
        case I2C_CMD_SET_BLACK_OFFSET:
        case I2C_CMD_SET_WHITE_OFFSET:
            flashFlag = receiveBuffer[1];
        break;
        
        case I2C_CMD_LIGHT_ON:
            digitalWrite(COLOR_LED_PIN_NUM, HIGH);
            digitalWrite(LIGTH_LED_PIN, HIGH);
            lightMode = true;
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_LIGHT_OFF:
            digitalWrite(COLOR_LED_PIN_NUM, LOW);
            digitalWrite(LIGTH_LED_PIN, LOW);
            lightMode = false;
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
	// getRawLight();
	
    if(updateFlag)
    {
        lightEventTemp = lightEvent;
        lightBitsTemp = lightBits;
        colorTemp = color;
        lightLATemp = lightLA;
        lightLBTemp = lightLB;
        clearCodeTemp = clearCode;
        lightRATemp = lightRA;
        lightRBTemp = lightRB;
    }
#ifdef BLE_SUPPORT
	Core_mode = CORE_ATMEL_MODE;
#endif
 
	switch(commandReceive)
	{
		case I2C_CMD_GET_DEV_ID:
			Wire.write(ptr1, 4);
			commandReceive = I2C_CMD_NULL;
		break;
		
		case I2C_CMD_GET_COLOR_EVENT:
			Wire.write(ptr1 + 4, 4);
			commandReceive = I2C_CMD_NULL;
			// packet_01_data.data.deviceEvent = 0;
		break;
        
		case I2C_CMD_GET_LINER_EVENT:
            Wire.write(lightEventTemp);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_GET_LINER_BITS:
            Wire.write(lightBitsTemp);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_GET_COLOR_DATA:
        {
            uint8_t *ptr2 = (uint8_t *)&colorTemp;
            Wire.write(ptr2, 4);
			commandReceive = I2C_CMD_NULL;
        }
        break;
        			/*
uint16_t blackColorOffset[4] = {0x00A0, 0x0020, 0x0040, 0x0060}; 
uint16_t blackLightOffset[4] = {0x0010, 0x0010, 0x0010, 0x0010}; 
uint16_t whiteColorOffset[4] = {0x0900, 0x0180, 0x0340, 0x04C0}; 
uint16_t whiteLightOffset[4] = {0x01C0, 0x01C0, 0x01C0, 0x01A0}; 
*/ 
        case I2C_CMD_GET_LINER_DATA+10:
        {
            Wire.write((uint8_t *)(&blackColorOffset[0]), 2);
            Wire.write((uint8_t *)(&blackColorOffset[1]), 2);
            Wire.write((uint8_t *)(&blackColorOffset[2]), 2);
            Wire.write((uint8_t *)(&blackColorOffset[3]), 2);
            Wire.write((uint8_t *)(&blackLightOffset[0]), 2);
            Wire.write((uint8_t *)(&blackLightOffset[1]), 2);
            Wire.write((uint8_t *)(&blackLightOffset[2]), 2);
            Wire.write((uint8_t *)(&blackLightOffset[3]), 2);

            Wire.write((uint8_t *)(&whiteColorOffset[0]), 2);
            Wire.write((uint8_t *)(&whiteColorOffset[1]), 2);
            Wire.write((uint8_t *)(&whiteColorOffset[2]), 2);
            Wire.write((uint8_t *)(&whiteColorOffset[3]), 2);
            Wire.write((uint8_t *)(&whiteLightOffset[0]), 2);
            Wire.write((uint8_t *)(&whiteLightOffset[1]), 2);
            Wire.write((uint8_t *)(&whiteLightOffset[2]), 2);
            Wire.write((uint8_t *)(&whiteLightOffset[3]), 2);

            commandReceive = I2C_CMD_NULL;
        }
        break;

        case I2C_CMD_GET_LINER_DATA:
        {
            Wire.write((uint8_t *)(&lightLATemp), 2);
            Wire.write((uint8_t *)(&lightLBTemp), 2);
            Wire.write((uint8_t *)(&clearCodeTemp), 2);
            Wire.write((uint8_t *)(&lightRBTemp), 2);
            Wire.write((uint8_t *)(&lightRATemp), 2);
            commandReceive = I2C_CMD_NULL;
        }
        break;
        
        case I2C_CMD_TEST_GET_VER:
            Wire.write(versions, 3);
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
void i2cStart(void)
{	
	I2C_SDA_HIGH();
	I2C_WAIT();
	I2C_SCL_HIGH();  
	I2C_WAIT();
	I2C_SDA_LOW();
	I2C_WAIT();
	I2C_SCL_LOW(); 
	I2C_WAIT();
}

void i2cStop(void)
{	
	I2C_SDA_LOW();
	I2C_WAIT();
	I2C_SCL_HIGH();
	I2C_WAIT();
	I2C_SDA_HIGH();
	I2C_WAIT();
}

bool i2cWaitAck(void)
{
	bool ack, ret;
	
	I2C_SDA_IN();

	ack = I2C_SDA_READ();
	
	I2C_SCL_HIGH();  
	I2C_WAIT();
	I2C_SCL_LOW();
	I2C_WAIT();

	I2C_SDA_OUT();
	
	ret = (ack == LOW)?true:false;
	return ret;
}

void i2cAck(void)
{
	I2C_SDA_LOW();  
	I2C_WAIT();
	I2C_SCL_HIGH();  
	I2C_WAIT();
	I2C_SCL_LOW();  
	I2C_WAIT();
}

void i2cNAck(void)
{	
	I2C_SDA_HIGH();  
	I2C_WAIT();
	I2C_SCL_HIGH();  
	I2C_WAIT(); 
	I2C_SCL_LOW();  
	I2C_WAIT();
}

void i2cSendByte(uint8_t txd)
{
	uint8_t i;
	
	for(i = 0; i < 8; i ++)  
	{                
		if(txd & 0x80)I2C_SDA_HIGH();
		else  I2C_SDA_LOW();
		txd <<= 1;
		
		I2C_SCL_HIGH();
		I2C_WAIT();
		I2C_SCL_LOW();   
		I2C_WAIT();
	}
}

uint8_t i2cReadByte(void)
{
	uint8_t i, res = 0;

	I2C_SDA_IN();
	
	for(i = 0; i < 8; i ++)  
	{
		res <<= 1;  
		if(I2C_SDA_READ())res ++;  
		
		I2C_SCL_HIGH();
		I2C_WAIT();
		I2C_SCL_LOW();
		I2C_WAIT(); 
	}           
	
	I2C_SDA_OUT();
	
	return res;  
}

void write8 (uint8_t reg, uint8_t value)
{
    i2cStart(); 
	i2cSendByte((TCS34725_I2C_ADDR << 1) | 0x00);
	i2cWaitAck();
	i2cSendByte(TCS34725_COMMAND_BIT | reg); 
	i2cWaitAck();
    i2cSendByte(value); 
	i2cWaitAck();
	i2cStop();
}

uint8_t read8 (uint8_t reg)
{
    uint8_t data = 0; 

	i2cStart(); 
	i2cSendByte((TCS34725_I2C_ADDR << 1) | 0x00);
	i2cWaitAck();
	i2cSendByte(TCS34725_COMMAND_BIT | reg); 
	i2cWaitAck();
	i2cStart(); 
	i2cSendByte((TCS34725_I2C_ADDR << 1) | 0x01);
	i2cWaitAck();
	data = i2cReadByte(); 
	i2cNAck();
	i2cStop(); 

	return data;    
}

uint16_t read16 (uint8_t reg)
{
    uint16_t x = 0, t = 0;

	i2cStart(); 
	i2cSendByte((TCS34725_I2C_ADDR << 1) | 0x00);
	i2cWaitAck();
	i2cSendByte(TCS34725_COMMAND_BIT | reg); 
	i2cWaitAck();
	i2cStart(); 
	i2cSendByte((TCS34725_I2C_ADDR << 1) | 0x01);
	i2cWaitAck();
	t = i2cReadByte();
	i2cAck();
    x = i2cReadByte();
	i2cNAck();
	i2cStop(); 

    x <<= 8;
    x |= t;
  
	return x;
}

void readBuf (uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

	i2cStart(); 
	i2cSendByte((TCS34725_I2C_ADDR << 1) | 0x00);
	i2cWaitAck();
	i2cSendByte(TCS34725_COMMAND_BIT | reg); 
	i2cWaitAck();
	i2cStart(); 
	i2cSendByte((TCS34725_I2C_ADDR << 1) | 0x01);
	i2cWaitAck();
    
    for(i = 0; i < (len - 1); i ++)
    {
        buf[i] = i2cReadByte();
        i2cAck();
    }

    buf[i] = i2cReadByte();
    i2cNAck();
        
	i2cStop(); 
}

void enableTCS34725(void)
{
    write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
    delay(3);
    write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);  
}

void disableTCS34725(void)
{
    /* Turn the device off to save power */
    uint8_t reg = 0;
    reg = read8(TCS34725_ENABLE);
    write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

void setInterrupt(bool flag)
{
    uint8_t r = read8(TCS34725_ENABLE);
    
    if(flag)
    {
        r |= TCS34725_ENABLE_AIEN;
    }
    else
    {
        r &= ~TCS34725_ENABLE_AIEN;
    }
    
    write8(TCS34725_ENABLE, r);
}

void clearInterrupt(void)
{
    i2cStart(); 
	i2cSendByte((TCS34725_I2C_ADDR << 1) | 0x00);
	i2cWaitAck();
	i2cSendByte(TCS34725_COMMAND_BIT | 0x66); 
	i2cWaitAck();
	i2cStop();
}

void initTCS34725(void)
{
    setIntegrationTime(TCS34725_INTEGRATIONTIME_24MS);
    setGain(TCS34725_GAIN_4X);
    enableTCS34725();
}

void setIntegrationTime(tcs34725IntegrationTime_t it)
{
    /* Update the timing register */
    write8(TCS34725_ATIME, it);
}

void setGain(tcs34725Gain_t gain)
{
    /* Update the timing register */
    write8(TCS34725_CONTROL, gain);
}

void getRawData(void)
{
    uint8_t data[8] = {0, };
    readBuf(TCS34725_CDATAL, data, 8);
    
    clearCode = data[0] + (data[1] << 8);
    redCode = data[2] + (data[3] << 8);
    greenCode = data[4] + (data[5] << 8);
    blueCode = data[6] + (data[7] << 8);
}

// LA, LB, Mid, RB, RA
void getRawLight(void)
{
    lightLA = analogRead(LIGHT_LA_PIN);
    lightLB = analogRead(LIGHT_LB_PIN);
    lightRB = analogRead(LIGHT_RB_PIN);
    lightRA = analogRead(LIGHT_RA_PIN);
}

void loadOffsetData(void)
{
    uint16_t data = 0;

    data = Flash.read16(1);
    if(data != 0xffff)blackColorOffset[0] = data;
    data = Flash.read16(2);
    if(data != 0xffff)blackColorOffset[1] = data;
    data = Flash.read16(3);
    if(data != 0xffff)blackColorOffset[2] = data;
    data = Flash.read16(4);
    if(data != 0xffff)blackColorOffset[3] = data;
    
    data = Flash.read16(5);
    if(data != 0xffff)blackLightOffset[0] = data;
    data = Flash.read16(6);
    if(data != 0xffff)blackLightOffset[1] = data;
    data = Flash.read16(7);
    if(data != 0xffff)blackLightOffset[2] = data;
    data = Flash.read16(8);
    if(data != 0xffff)blackLightOffset[3] = data;
    
    data = Flash.read16(9);
    if(data != 0xffff)whiteColorOffset[0] = data;
    data = Flash.read16(10);
    if(data != 0xffff)whiteColorOffset[1] = data;
    data = Flash.read16(11);
    if(data != 0xffff)whiteColorOffset[2] = data;
    data = Flash.read16(12);
    if(data != 0xffff)whiteColorOffset[3] = data;
    
    data = Flash.read16(13);
    if(data != 0xffff)whiteLightOffset[0] = data;
    data = Flash.read16(14);
    if(data != 0xffff)whiteLightOffset[1] = data;
    data = Flash.read16(15);
    if(data != 0xffff)whiteLightOffset[2] = data;
    data = Flash.read16(16);
    if(data != 0xffff)whiteLightOffset[3] = data;
}

void setBlackOffset(bool flag)
{
    getRawData();
    getRawLight();
    
    blackColorOffset[0] = clearCode;
    blackColorOffset[1] = redCode;
    blackColorOffset[2] = greenCode;
    blackColorOffset[3] = blueCode; 
    
    blackLightOffset[0] = lightLA;
    blackLightOffset[1] = lightLB;
    blackLightOffset[2] = lightRB;
    blackLightOffset[3] = lightRA;

    if(flag)
    {
        Flash.write16(1, blackColorOffset[0]);
        Flash.write16(2, blackColorOffset[1]);
        Flash.write16(3, blackColorOffset[2]);
        Flash.write16(4, blackColorOffset[3]);
        
        Flash.write16(5, blackLightOffset[0]);
        Flash.write16(6, blackLightOffset[1]);
        Flash.write16(7, blackLightOffset[2]);
        Flash.write16(8, blackLightOffset[3]);
    }
}

void setWhiteOffset(bool flag)
{
    getRawData();
    getRawLight();

    if(lightLA > blackLightOffset[0])whiteLightOffset[0] = lightLA - blackLightOffset[0];
    if(lightLB > blackLightOffset[1])whiteLightOffset[1] = lightLB - blackLightOffset[1];
    if(lightRB > blackLightOffset[2])whiteLightOffset[2] = lightRB - blackLightOffset[2];
    if(lightRA > blackLightOffset[3])whiteLightOffset[3] = lightRA - blackLightOffset[3];
    
    if(clearCode > blackColorOffset[0])whiteColorOffset[0] = clearCode - blackColorOffset[0];
    if(redCode > blackColorOffset[1])whiteColorOffset[1] = redCode - blackColorOffset[1];
    if(greenCode > blackColorOffset[2])whiteColorOffset[2] = greenCode - blackColorOffset[2];
    if(blueCode > blackColorOffset[3])whiteColorOffset[3] = blueCode - blackColorOffset[3];
    
    if(flag)
    {
        Flash.write16(9, whiteColorOffset[0]);
        Flash.write16(10, whiteColorOffset[1]);
        Flash.write16(11, whiteColorOffset[2]);
        Flash.write16(12, whiteColorOffset[3]);
        
        Flash.write16(13, whiteLightOffset[0]);
        Flash.write16(14, whiteLightOffset[1]);
        Flash.write16(15, whiteLightOffset[2]);
        Flash.write16(16, whiteLightOffset[3]);
    }
}



button_state_t getButtonState() 
{
    int16_t count1 = 0;
    int16_t count2 = 0;

    // idol
    while (1)
    {
        if(!digitalRead(COLOR_BUTTON_PIN_NUM)) count1++;
        else count2++;
        if (count2 > 1) return COLOR_BUTTON_IDOL;
        if (count1 > 10) break;
        delay(10);
    }
    // interval
    count1 = 0;
    count2 = 0;
    while (1)
    {
        if(!digitalRead(COLOR_BUTTON_PIN_NUM)) count1++;
        else count2++;
        if (count2 > 5) break;
        if (count1 > 100) return COLOR_BUTTON_LONG_CLICK;
        delay(10);
    }
    // single or double
    count1 = 0;
    count2 = 0;
    while (1) 
    {
        if(!digitalRead(COLOR_BUTTON_PIN_NUM)) count1++;
        else count2++;
        if (count2 > 30) return COLOR_BUTTON_SINGLE_CLICK;
        if (count1 > 5) return COLOR_BUTTON_DOUBLE_CLICK;
        delay(10);
    }
}


bool isButtonLongPress()
{
    static uint16_t count = 0;
#ifdef DEBUG
    Serial.println(count);
#endif
    if (!digitalRead(COLOR_BUTTON_PIN_NUM)) count++;
    else
    {
        if ((count == 1) || (count == 0)) return false;
        count --;
    }
    if (count > COLOR_BUTTON_LONG_CLICK_TIMES)
    {
        count = 0;
        return true;
    }
    return false;
}
