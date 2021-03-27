
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
#define COLOR_INT_PIN_NUM		PA6
#define COLOR_POWER_PIN_NUM     PA5
#define COLOR_LED_PIN_NUM 		PA4
#define COLOR_VOLTAGE_PIN_NUM	PA0

#define SOFT_SCL_PIN	        PA7
#define SOFT_SDA_PIN	        PB1

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

#define COLOR_SAMPLE_INTERVAL    5
// 50 ms


uint32_t sampleTimerPreviousMillis = 0;
uint16_t clearCode = 0, redCode = 0, greenCode = 0, blueCode = 0;
uint8_t clear = 0, red = 0, green = 0, blue = 0;
uint32_t color = 0;

/***************************************************************
 Communication defines
 ***************************************************************/
// #define DEVICE_I2C_ADDRESS		0x0d
#define DEVICE_I2C_ADDRESS		0x7d
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x000a

#define I2C_DEF_ADDR_FLASH_LOC	0x00
#define I2C_CUR_ADDR_FLASH_LOC	0x01

#define I2C_CMD_GET_DEV_ID		0x00 // 
#define I2C_CMD_GET_DEV_EVENT	0x01 // 
#define I2C_CMD_GET_COLOR_DATA	0x02 // 
#define I2C_CMD_LIGHT_ON        0x03 // 
#define I2C_CMD_LIGHT_OFF       0x04 // 
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
#define I2C_CMD_NULL			0xff // 

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

#define CLEAR_CODE 1300

/***************************************************************
 Device initialization
 ***************************************************************/
void setup()
{
	uint8_t i2cDefaultAddr = Flash.read8(I2C_DEF_ADDR_FLASH_LOC); 
	uint8_t i2cCurrentAddr = Flash.read8(I2C_CUR_ADDR_FLASH_LOC);
	
	if(i2cDefaultAddr == 0xff)Flash.write8(I2C_DEF_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	if(i2cCurrentAddr == 0xff)Flash.write8(I2C_CUR_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	else deviceI2CAddress = i2cCurrentAddr;
	
	packet_01_data.data.deviceVID = DEVICE_VID;
	packet_01_data.data.devicePID = DEVICE_PID;
	packet_01_data.data.deviceEvent = 0;
	
	nrgSave.begin(GROVE_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance

	pinMode(GROVE_LED_PIN_NUM, OUTPUT);
	digitalWrite(GROVE_LED_PIN_NUM, HIGH);
    
    pinMode(COLOR_INT_PIN_NUM, INPUT);
    pinMode(COLOR_POWER_PIN_NUM, OUTPUT);
    // pinMode(COLOR_LED_PIN_NUM, OUTPUT);
    digitalWrite(COLOR_POWER_PIN_NUM, HIGH);
	// digitalWrite(COLOR_LED_PIN_NUM, HIGH);
	analogWrite(COLOR_LED_PIN_NUM, 255);
    
    pinMode(SOFT_SCL_PIN, OUTPUT);
	pinMode(SOFT_SDA_PIN, OUTPUT);
	digitalWrite(SOFT_SCL_PIN, HIGH);
	digitalWrite(SOFT_SDA_PIN, HIGH);
    
    initTCS34725();
	
	Wire.begin(deviceI2CAddress);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);
  
	wwdg.begin();
    
    // Serial.begin(115200);
}

void loop()
{
    uint32_t sampleTimerCurrentMillis = millis();
    if(sampleTimerCurrentMillis - sampleTimerPreviousMillis >= COLOR_SAMPLE_INTERVAL)
    {
        sampleTimerPreviousMillis = sampleTimerCurrentMillis;
        
        getRawData();
        
        float r, g, b;
        r = redCode; r /= clearCode;
        g = greenCode; g /= clearCode;
        b = blueCode; b /= clearCode;
        r *= 256; g *= 256; b *= 256;
        red = r; green = g; blue = b;
        
        color = (red << 16) + (green << 8) + blue;
        
        // Serial.print("C:"); Serial.print(clearCode);
        // Serial.print("\tR:"); Serial.print(redCode);
        // Serial.print("\tG:"); Serial.print(greenCode);
        // Serial.print("\tB:"); Serial.print(blueCode);
        
        // Serial.print("\t");Serial.print(red);Serial.print(",");Serial.print(green);Serial.print(",");Serial.print(blue);
        // Serial.println();
         
        float rp, gp, bp;
        rp = r / (r + g + b);
        gp = g / (r + g + b);
        bp = b / (r + g + b);
        if((rp - gp) > 0.08 && (rp - bp) > 0.08)packet_01_data.data.deviceEvent = 1;
        else if((gp - rp) > 0.07 && (gp - bp) > 0.07)packet_01_data.data.deviceEvent = 2;
		else if((bp -gp) > 0.09 && (bp - rp) > 0.09)packet_01_data.data.deviceEvent = 3;
		// white
		else if (clearCode >= CLEAR_CODE) packet_01_data.data.deviceEvent = 4;
		// black or others or nothing
        else packet_01_data.data.deviceEvent = 0;
        
        // Serial.print((uint8_t)(rp * 100));Serial.print(",");Serial.print((uint8_t)(gp * 100));Serial.print(",");Serial.println((uint8_t)(bp * 100));
        // Serial.println();
        
        // Serial.println(packet_01_data.data.deviceEvent);
        // Serial.println();
    }
	
	if(ledFlashCommand)
	{
		uint32_t ledFlashCurrentMillis = millis();
		if(ledFlashCurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
		{
			ledFlashPreviousMillis = ledFlashCurrentMillis;
			digitalWrite(GROVE_LED_PIN_NUM, ledFlashStatus);
			ledFlashStatus = !ledFlashStatus;
		}
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
            digitalWrite(COLOR_POWER_PIN_NUM, HIGH);
            digitalWrite(COLOR_LED_PIN_NUM, HIGH);
			
			wwdg.end();
			Wire.end();
			pinMode(PA9, INPUT_PULLUP);
			pinMode(PA10, INPUT_PULLUP);
			
			nrgSave.standby();

			Wire.begin(deviceI2CAddress);
			Wire.onReceive(receiveEvent);
			Wire.onRequest(requestEvent);
			wwdg.begin();
            
            digitalWrite(COLOR_POWER_PIN_NUM, LOW);
            digitalWrite(COLOR_LED_PIN_NUM, LOW);
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
	uint8_t i = 0, receiveBuffer[4] = {0,};
	// autoSleepPreviousMillis = millis();
	
	while(Wire.available())
	{	
		receiveBuffer[i ++] = Wire.read();
		if(i >= 4)i = 0;
	}
	
	commandReceive = receiveBuffer[0];
	
	switch(commandReceive)
	{
        case I2C_CMD_LIGHT_ON:
            analogWrite(COLOR_LED_PIN_NUM, 255);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_LIGHT_OFF:
            analogWrite(COLOR_LED_PIN_NUM, 0);
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
        
        case I2C_CMD_GET_COLOR_DATA:
        {
            uint8_t *ptr2 = (uint8_t *)&color;
            Wire.write(ptr2, 4);
			commandReceive = I2C_CMD_NULL;
        }
        break;
        
        case I2C_CMD_TEST_GET_VER:
            Wire.write(versions, 3);
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
    setIntegrationTime(TCS34725_INTEGRATIONTIME_2_4MS);
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
    
    // clearCode = read16(TCS34725_CDATAL);
    // redCode = read16(TCS34725_RDATAL);
    // greenCode = read16(TCS34725_GDATAL);
    // blueCode = read16(TCS34725_BDATAL);
}
