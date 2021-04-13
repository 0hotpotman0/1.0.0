#include "Arduino.h"
#include "stdio.h"
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>

#define GROVE_TWO_TX_PIN_NUM	PA2
#define GROVE_TWO_RX_PIN_NUM	PA3

#define LIGHT_LED_PIN_NUM		PA1
#define LIGHT_VERSION_PIN_NUM   PB1
#define ANALOG_DATA_PIN        PA0 


int val;
int KK;
void setup() {
  Serial.begin(115200);

  pinMode(LIGHT_LED_PIN_NUM, OUTPUT);
// UartInit_Loop(); 
}

void loop() {
val = analogRead(PB1);
KK = analogRead(PA0);
// KK = analogRead(PB1);
//   put your main code here, to run repeatedly:

Serial.print(val);
Serial.print(KK);
delay(500);
}



/*

**********************************************************************************************************************************************
**********************************************************************************************************************************************


*/



// #include <Wire.h>
// #include <Flash.h>
// #include <LowPower.h>
// // #include <RTCTimer.h>
// #include <WatchDog.h>
 

// #define GROVE_TWO_TX_PIN_NUM	PA2
// #define GROVE_TWO_RX_PIN_NUM	PA3

// #define LIGHT_LED_PIN_NUM		PA1
// #define LIGHT_VERSION_PIN_NUM   PB1

// //Version 1.0 is digital light sensor; Version 1.1 is analog light sensor
// static uint8_t lightSensorVersion = 10;

// /***************************************************************
// Macro definition of SG-A1DPHY analog light sensor
//  ***************************************************************/
//  #define ANALOG_DATA_PIN        PA0         //pin for reading analog light sensor data
//  #define ANALOG_POWER_PIN       PA4         //pin for controlling power of light sensor
//  #define ANALOG_CHANNEL_0_PIN   PA5         //write LOW to enable channel0
//  #define ANALOG_CHANNEL_1_PIN   PA6         //write LOW to enable channel1
//  #define ANALOG_CHANNEL_2_PIN   PA7         //write LOW to enable channel2

//  #define FORMULA_0_A            0x0957       //y=0.2391x
//  #define FORMULA_0_B            0x00
//  #define FORMULA_1_A            0x0bac       //y=0.2988x-31.147
//  #define FORMULA_1_B            0x04c0ae
//  #define FORMULA_2_A            0x59b1       //y=2.2961x-23.053
//  #define FORMULA_2_B            0x038482
//  #define FORMULA_3_A            0x51d8       //y=20.952x-77.187
//  #define FORMULA_3_B            0x012d83
//  #define FORMULA_4_A            0x3ad8       //y=150.64x-93145
//  #define FORMULA_4_B            0x016bd9

// /***************************************************************
// Macro definition of tsl2561t digital light sensor
//  ***************************************************************/
// #define SOFT_SCL_PIN	PA7
// #define SOFT_SDA_PIN	PB1

// #define LIGHT_INT_PIN	PA6

// #define I2C_DELAY		1
// #define I2C_WAIT()		delayMicroseconds(I2C_DELAY)
// #define I2C_SDA_IN()	pinMode(SOFT_SDA_PIN, INPUT)
// #define I2C_SDA_OUT()	pinMode(SOFT_SDA_PIN, OUTPUT)
// #define I2C_SDA_READ()	digitalRead(SOFT_SDA_PIN)
// #define I2C_SCL_LOW()	digitalWrite(SOFT_SCL_PIN, LOW)
// #define I2C_SCL_HIGH()	digitalWrite(SOFT_SCL_PIN, HIGH)
// #define I2C_SDA_LOW()	digitalWrite(SOFT_SDA_PIN, LOW)
// #define I2C_SDA_HIGH()	digitalWrite(SOFT_SDA_PIN, HIGH)

// void i2cStart(void);
// void i2cStop(void);
// bool i2cWaitAck(void);
// void i2cAck(void);
// void i2cNAck(void);
// void i2cSendByte(uint8_t txd);
// uint8_t i2cReadByte(void);

// #define	TLS2561T_I2C_ADDR	0x29
// #define	TSL2561T_CONTROL	0x80
// #define	TSL2561T_TIMING		0x81
// #define	TSL2561T_INTERRUPT	0x86
// #define	TSL2561T_CHANNAL0L	0x8C
// #define	TSL2561T_CHANNAL0H	0x8D
// #define	TSL2561T_CHANNAL1L	0x8E
// #define	TSL2561T_CHANNAL1H	0x8F

// #define LUX_SCALE 14           // scale by 2^14
// #define RATIO_SCALE 9          // scale ratio by 2^9
// #define CH_SCALE 10            // scale channel values by 2^10
// #define CHSCALE_TINT0 0x7517   // 322/11 * 2^CH_SCALE
// #define CHSCALE_TINT1 0x0fe7   // 322/81 * 2^CH_SCALE

// #define K1T 0x0040   // 0.125 * 2^RATIO_SCALE
// #define B1T 0x01f2   // 0.0304 * 2^LUX_SCALE
// #define M1T 0x01be   // 0.0272 * 2^LUX_SCALE
// #define K2T 0x0080   // 0.250 * 2^RATIO_SCA
// #define B2T 0x0214   // 0.0325 * 2^LUX_SCALE
// #define M2T 0x02d1   // 0.0440 * 2^LUX_SCALE
// #define K3T 0x00c0   // 0.375 * 2^RATIO_SCALE
// #define B3T 0x023f   // 0.0351 * 2^LUX_SCALE
// #define M3T 0x037b   // 0.0544 * 2^LUX_SCALE
// #define K4T 0x0100   // 0.50 * 2^RATIO_SCALE
// #define B4T 0x0270   // 0.0381 * 2^LUX_SCALE
// #define M4T 0x03fe   // 0.0624 * 2^LUX_SCALE
// #define K5T 0x0138   // 0.61 * 2^RATIO_SCALE
// #define B5T 0x016f   // 0.0224 * 2^LUX_SCALE
// #define M5T 0x01fc   // 0.0310 * 2^LUX_SCALE
// #define K6T 0x019a   // 0.80 * 2^RATIO_SCALE
// #define B6T 0x00d2   // 0.0128 * 2^LUX_SCALE
// #define M6T 0x00fb   // 0.0153 * 2^LUX_SCALE
// #define K7T 0x029a   // 1.3 * 2^RATIO_SCALE
// #define B7T 0x0018   // 0.00146 * 2^LUX_SCALE
// #define M7T 0x0012   // 0.00112 * 2^LUX_SCALE
// #define K8T 0x029a   // 1.3 * 2^RATIO_SCALE
// #define B8T 0x0000   // 0.000 * 2^LUX_SCALE
// #define M8T 0x0000   // 0.000 * 2^LUX_SCALE

// #define K1C 0x0043   // 0.130 * 2^RATIO_SCALE
// #define B1C 0x0204   // 0.0315 * 2^LUX_SCALE
// #define M1C 0x01ad   // 0.0262 * 2^LUX_SCALE
// #define K2C 0x0085   // 0.260 * 2^RATIO_SCALE
// #define B2C 0x0228   // 0.0337 * 2^LUX_SCALE
// #define M2C 0x02c1   // 0.0430 * 2^LUX_SCALE
// #define K3C 0x00c8   // 0.390 * 2^RATIO_SCALE
// #define B3C 0x0253   // 0.0363 * 2^LUX_SCALE
// #define M3C 0x0363   // 0.0529 * 2^LUX_SCALE
// #define K4C 0x010a   // 0.520 * 2^RATIO_SCALE
// #define B4C 0x0282   // 0.0392 * 2^LUX_SCALE
// #define M4C 0x03df   // 0.0605 * 2^LUX_SCALE
// #define K5C 0x014d   // 0.65 * 2^RATIO_SCALE
// #define B5C 0x0177   // 0.0229 * 2^LUX_SCALE
// #define M5C 0x01dd   // 0.0291 * 2^LUX_SCALE
// #define K6C 0x019a   // 0.80 * 2^RATIO_SCALE
// #define B6C 0x0101   // 0.0157 * 2^LUX_SCALE
// #define M6C 0x0127   // 0.0180 * 2^LUX_SCALE
// #define K7C 0x029a   // 1.3 * 2^RATIO_SCALE
// #define B7C 0x0037   // 0.00338 * 2^LUX_SCALE
// #define M7C 0x002b   // 0.00260 * 2^LUX_SCALE
// #define K8C 0x029a   // 1.3 * 2^RATIO_SCALE
// #define B8C 0x0000   // 0.000 * 2^LUX_SCALE
// #define M8C 0x0000   // 0.000 * 2^LUX_SCALE
// /***************************************************************
// functions of tsl2561t digital light sensor
//  ***************************************************************/
// uint8_t tsl2561tReadData(uint8_t dataAddr);
// void tsl2561tReadDataNumber(uint8_t dataAddr, uint8_t dataLen, uint8_t *buffer);
// void tsl2561tWriteData(uint8_t dataAddr, uint8_t dataValue);
// void tsl2561tInit(void);
// void tsl2561tPowerOn(void);
// void tsl2561tPowerOff(void);
// uint32_t calculateLux(uint8_t iGain, uint8_t tInt, uint8_t iType, uint16_t ch0, uint16_t ch1);
// uint16_t tsl2561tGetLightData(void);

// /***************************************************************
// functions of SG-A1DPHY analog light sensor
//  ***************************************************************/
// uint16_t analogGetLightData(void);
// void analogSwitchLightGain(uint8_t gain);
// void analogLightInit(void);
// void analogLightPowerOn(void);
// void analogLightPowerOff(void);

// /***************************************************************
// check light sensor hardware version
//  ***************************************************************/
// uint8_t checkLightSensorVersion(void);

// /***************************************************************

//  ***************************************************************/
// #define DEVICE_I2C_ADDRESS		0x05
// #define DEVICE_VID				0x2886
// #define DEVICE_PID				0x0005

// #define I2C_DEF_ADDR_FLASH_LOC		0x00 // byte address
// #define I2C_CUR_ADDR_FLASH_LOC		0x01 // byte address
// #define I2C_THD_0_ADDR_FLASH_LOC	0x01 // int address
// #define I2C_THD_1_ADDR_FLASH_LOC	0x02 // int address
// #define I2C_THD_2_ADDR_FLASH_LOC	0x03 // int address

// #define I2C_CMD_GET_DEV_ID		0x00 //
// #define I2C_CMD_GET_DEV_EVENT	0x01 //
// #define I2C_CMD_GET_LIGHT		0x02 //
// #define I2C_CMD_SET_THD			0x03 //
// #define I2C_CMD_LED_ON			0xb0 //
// #define I2C_CMD_LED_OFF			0xb1 //
// #define I2C_CMD_AUTO_SLEEP_ON	0xb2 //
// #define I2C_CMD_AUTO_SLEEP_OFF	0xb3 //
// #define I2C_CMD_SET_ADDR		0xc0 //
// #define I2C_CMD_RST_ADDR		0xc1 //
// #define I2C_CMD_TEST_TX_RX_ON   0xe0 //
// #define I2C_CMD_TEST_TX_RX_OFF  0xe1 //
// #define I2C_CMD_TEST_GET_VER    0xe2 //
// #define I2C_CMD_JUMP_TO_BOOT	0xf0 //
// #define I2C_CMD_GET_DEVICE_UID  0xf1 // 
// #define I2C_CMD_NULL			0xff //

// uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS;
// uint8_t commandReceive = I2C_CMD_NULL;
// uint8_t	ledFlashTimes = 0;

// #ifdef BLE_SUPPORT

// uint8_t Core_mode = 0, ErrorCount=0;
// uint32_t StartMillis = 0;
// uint32_t preEvent;

// typedef struct
// {
// 	uint8_t Datalen;
// 	uint8_t type;
// 	uint8_t Address;
// 	uint8_t Option;
// }packet_header_t;

// typedef struct
// {
// 	uint8_t 	Raw_data_type;
// 	uint8_t		delay[2];
// }packet_raw_t;

// typedef struct
// {
// 	uint8_t threshold0[2];
// 	uint8_t threshold1[2];
// 	uint8_t flashSave;
// }packet_thlsd_t;

// typedef struct
// {
// 	uint8_t 	pid[2];
// 	uint8_t 	chipid;
// 	uint8_t 	Newaddress;
// 	uint8_t 	option[5];
// }packet_got_atr;

// typedef struct
// {
// 	packet_header_t	Header;
// 	union 
// 	{
// 		packet_raw_t	raw;
// 		packet_thlsd_t	thread;
// 		packet_got_atr	atr;
// 		uint8_t Option[MAINBOARD_BLE_I2C_DATALEN-sizeof(packet_header_t)];
// 	}commands;

// }packet_thld_t; // 8 bytes

// union
// {
//     packet_thld_t data;
//     uint8_t bytes[MAINBOARD_BLE_I2C_DATALEN]; 
// }commandOption;

// typedef struct
// {
// 	packet_header_t	Header;
// 	uint8_t		data[2];
// }packet_raw;

// typedef struct
// {
// 	packet_header_t	Header;
// 	uint8_t		Event;
// }packet_event;

// typedef struct
// {
// 	packet_header_t	Header;
// 	uint8_t 	pid[2];
// 	uint8_t 	chipid;
// 	uint8_t 	hwaddress;
// 	uint8_t 	version[3];
// 	uint8_t 	option[2];
// }packet_atr;

// union
// {
// 	packet_atr		atr;
// 	packet_event	event;
// 	packet_raw		raw_data;
//     uint8_t 		bytes[MAINBOARD_BLE_I2C_DATALEN]; 
// }InstructionOption;

// uint8_t	LightRawData = 0;
// uint32_t RawDelayMillis = I2C_CMD_RAW_DATA_TIME;
// uint32_t RawPreviousMillis = 0;
// uint8_t *ptr2 = (uint8_t *)&InstructionOption;

// #endif

// typedef struct
// {
// 	uint16_t deviceVID;
// 	uint16_t devicePID;
// 	uint32_t deviceEvent;
// }packet_01_t; // 8 bytes

// union
// {
//     packet_01_t data;
//     uint8_t bytes[8];
// }packet_01_data;

// uint8_t *ptr1 = (uint8_t *)&packet_01_data;

// void requestEvent(void);
// void receiveEvent(int howMany);

// /***************************************************************

//  ***************************************************************/
// LowPower nrgSave;

// #define AUTO_SLEEP_TIMEOUT	2000

// uint32_t autoSleepPreviousMillis = 0, PreMillis=0;
// bool autoSleepFlag = false;
// // bool autoSleepFlag = true;

// /***************************************************************

//  ***************************************************************/
// #define LED_FLASH_TIME	250

// bool ledFlashCommand = true;
// bool ledFlashStatus = false;
// uint32_t ledFlashPreviousMillis = 0;

// /***************************************************************

//  ***************************************************************/
// #define LIGHT_THD_0_NUM	50
// #define LIGHT_THD_1_NUM	200

// #define LIGHT_THD_ADJ	5

// uint16_t lightThd0 = LIGHT_THD_0_NUM;
// uint16_t lightThd1 = LIGHT_THD_1_NUM;

// /***************************************************************

//  ***************************************************************/
// #define LIGHT_MIN_CONVERT_TIME	150
// #define LIGHT_SAMPLE_PERIOD		250

// uint16_t lightSamplePeriod = LIGHT_SAMPLE_PERIOD;
// bool timeoutFlag = false;
// bool sampleFlag = false;
// uint16_t curLightData = 0;

// bool testFlag = false;
// // software version v1.4
// // author: yejiewei
// // change log: now this software will work for both digital
// // (hardware version v1.0) and analog(hardware version v1.1) light
// // sensor.
// char *versions = "V20";
// uint16_t NodeVersion = 0x6100;

// uint32_t intStart = 0;
// uint32_t intEnd = 0;

// /***************************************************************

//  ***************************************************************/
// uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// /***************************************************************

//  ***************************************************************/
// bool flashSave = false;

// /***************************************************************

//  ***************************************************************/
// void setup()
// {
// 	uint8_t i2cDefaultAddr = Flash.read8(I2C_DEF_ADDR_FLASH_LOC);
// 	uint8_t i2cCurrentAddr = Flash.read8(I2C_CUR_ADDR_FLASH_LOC);
    
//     uint8_t *ptr2 = (uint8_t *)Flash.getChipUniqueID();
//     for(uint8_t i = 0; i < 12; i ++)chipId[i] = *(ptr2 + i);

// 	if(i2cDefaultAddr == 0xff)Flash.write8(I2C_DEF_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
// 	if(i2cCurrentAddr == 0xff)Flash.write8(I2C_CUR_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
// 	else deviceI2CAddress = i2cCurrentAddr;

// 	uint16_t lightThd = Flash.read16(I2C_THD_0_ADDR_FLASH_LOC);
// 	if(lightThd == 0xffff)Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, LIGHT_THD_0_NUM);
// 	else lightThd0 = lightThd;

// 	lightThd = Flash.read16(I2C_THD_1_ADDR_FLASH_LOC);
// 	if(lightThd == 0xffff)Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, LIGHT_THD_1_NUM);
// 	else lightThd1 = lightThd;

// 	packet_01_data.data.deviceVID = DEVICE_VID;
// 	packet_01_data.data.devicePID = DEVICE_PID;
// 	packet_01_data.data.deviceEvent = 0;

//     // open rtc
// 	nrgSave.begin(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance

// 	pinMode(LIGHT_LED_PIN_NUM, OUTPUT);
// 	digitalWrite(LIGHT_LED_PIN_NUM, HIGH);

// 	pinMode(LIGHT_INT_PIN, INPUT);
//     // get and check hardware version, it should be 10 or 11
//     lightSensorVersion = checkLightSensorVersion();
//     if (lightSensorVersion == 10) {
//         // digital light sensor init
//         pinMode(SOFT_SCL_PIN, OUTPUT);
//         pinMode(SOFT_SDA_PIN, OUTPUT);
//         digitalWrite(SOFT_SCL_PIN, HIGH);
//         digitalWrite(SOFT_SDA_PIN, HIGH);

//         tsl2561tPowerOn();
//         tsl2561tInit();
//     }
//     else if (lightSensorVersion == 11) {
//         //analog light sensor init
//         analogLightInit();
//     }

	
// #ifdef BLE_SUPPORT
// 	StartMillis = millis();
// #endif

//     // timer
// 	// RTCTimer.begin(LIGHT_MIN_CONVERT_TIME, dummySample);

//     // hardware I2C
// 	Wire.begin(deviceI2CAddress);
// 	Wire.onReceive(receiveEvent);
// 	Wire.onRequest(requestEvent);

//     //watch dog
// 	wwdg.begin();
// }

// void loop()
// {
// 	uint32_t CurrentMillis = millis();
			
// 	if(Wire.isbusidle())PreMillis = CurrentMillis;
// 	if ((CurrentMillis - PreMillis) > 20)
// 	{
// 		Wire.reset();
// 		Wire.begin(deviceI2CAddress);
// 		Wire.onReceive(receiveEvent);
// 		Wire.onRequest(requestEvent);
// 		PreMillis = CurrentMillis;
// 	}
// 	if(timeoutFlag)
// 	{
// 		timeoutFlag = false;

//         if (lightSensorVersion == 10)
//         {
//             pinMode(SOFT_SCL_PIN, OUTPUT);
//     		pinMode(SOFT_SDA_PIN, OUTPUT);
//         }

// 		if(sampleFlag)
// 		{
//             if (lightSensorVersion == 10)
//             {
//                 curLightData = tsl2561tGetLightData();
//     			tsl2561tPowerOff();
//             }
//             else if (lightSensorVersion == 11)
//             {
//                 curLightData = analogGetLightData();
//                 pinMode(ANALOG_CHANNEL_0_PIN, INPUT);
//                 pinMode(ANALOG_CHANNEL_1_PIN, INPUT);
//                 pinMode(ANALOG_CHANNEL_2_PIN, INPUT);
//             }


//             // RTCTimer.setNewPeriod(lightSamplePeriod);
//             if(curLightData < (lightThd0-LIGHT_THD_ADJ))packet_01_data.data.deviceEvent = 1;
//             else if(curLightData >= (lightThd0+LIGHT_THD_ADJ) && curLightData < (lightThd1-LIGHT_THD_ADJ))packet_01_data.data.deviceEvent = 2;
//             else if(curLightData >= (lightThd1+LIGHT_THD_ADJ))packet_01_data.data.deviceEvent = 3;
// #ifdef BLE_SUPPORT
// 			if(Core_mode == CORE_BLE_MODE)
// 			{
// 				if(preEvent != packet_01_data.data.deviceEvent)
// 				{
// 					InstructionOption.event.Header.type = I2C_CMD_NOTIFY_EVENT;
// 					InstructionOption.event.Header.Address = deviceI2CAddress;
// 					InstructionOption.event.Event  = packet_01_data.data.deviceEvent;
// 					InstructionOption.event.Header.Datalen	 = sizeof(packet_event);
// 					Wire.MasterGPIOTransmission(ptr2, sizeof(packet_event));
// 					preEvent = packet_01_data.data.deviceEvent;
// 				}
// 			}
// #endif
// 		}
// 		else
// 		{
//             if (lightSensorVersion == 10)
//             {
//                 tsl2561tPowerOn();
//             }

// 			// RTCTimer.setNewPeriod(LIGHT_MIN_CONVERT_TIME);
// 		}
// 	}

// #ifdef BLE_SUPPORT
// 	if (Core_mode == 0)
// 	{// ATR
// 		uint32_t CurrentMillis = millis();
// 		if(CurrentMillis - StartMillis >= (I2C_CMD_SYS_READY_TIME+deviceI2CAddress*2))
// 		{
// #if 1
// 			Core_mode = CORE_BLE_MODE;
// #else
// 			commandReceive = I2C_CMD_NOTIFY_ATR;
// 			StartMillis = CurrentMillis;
// #endif

// 		}
// 	}
// 	if(Core_mode == CORE_BLE_MODE)
// 	{
// 		uint32_t CurrentMillis = millis();

// 		if (LightRawData)
// 		{
// 			if (CurrentMillis - RawPreviousMillis >= RawDelayMillis)
// 			{
// 				InstructionOption.raw_data.Header.type = I2C_CMD_GET_RAW_DATA;
// 				InstructionOption.raw_data.Header.Address = deviceI2CAddress;
// 				InstructionOption.raw_data.data[0] = curLightData&0xFF;
// 				InstructionOption.raw_data.data[1] = (curLightData>>8) & 0xff;
// 				InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);
// 				if(Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw)) == 0)
// 				{
// 					ErrorCount++;
// 					RawPreviousMillis += 3; // Retry in 3 ms later.
// 				}else
// 				{
// 					ErrorCount = 0;
// 					RawPreviousMillis = CurrentMillis;
// 				}
// 				if (ErrorCount > 10)
// 				{// I2C bus error, reset I2C bus.
// 					Wire.end();
// 					Wire.begin(deviceI2CAddress);
// 					Wire.onReceive(receiveEvent);
// 					Wire.onRequest(requestEvent);
// 					ErrorCount = 0;
// 				}

// 			}
// 		}
// 		switch(commandReceive)
// 		{

// 			case I2C_CMD_NOTIFY_ATR:
// 				InstructionOption.atr.Header.type = I2C_CMD_ATR;
// 				InstructionOption.atr.pid[0] = DEVICE_PID&0xff;
// 				InstructionOption.atr.pid[1] = (DEVICE_PID>>8 ) & 0xff;
// 				InstructionOption.atr.Header.Address = deviceI2CAddress;
// 				InstructionOption.atr.chipid = chipId[0];
// 				InstructionOption.atr.hwaddress = DEVICE_I2C_ADDRESS;
// 				InstructionOption.atr.version[0] = versions[0];
// 				InstructionOption.atr.version[1] = versions[1];
// 				InstructionOption.atr.version[2] = versions[2];
// 				InstructionOption.atr.option[0] = NodeVersion&0xFF;
// 				InstructionOption.atr.option[1] = (NodeVersion>>8)&0xFF;
// 				InstructionOption.atr.Header.Datalen = sizeof(packet_atr);
// 				Wire.MasterGPIOTransmission(ptr2, sizeof(packet_atr));
// 				commandReceive = I2C_CMD_NULL;
// 				break;
// 			case I2C_CMD_ATR:
// 				if ((commandOption.data.commands.atr.pid[0] == (DEVICE_PID&0xFF)) && 
// 					(commandOption.data.commands.atr.pid[1] == ((DEVICE_PID>>8)&0xFF)) && 
// 					(commandOption.data.commands.atr.chipid == chipId[0]))
// 				{// It's for current device
// 					if(commandOption.data.commands.atr.Newaddress != deviceI2CAddress)
// 					{
// 						deviceI2CAddress = commandOption.data.commands.atr.Newaddress;
// 						Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
// 						Wire.begin(deviceI2CAddress);
// 					}
					
// 				}
// 				commandReceive = I2C_CMD_NULL;
// 				break;
// 			case I2C_CMD_BLE_SET_THD:
// 				{
// 					lightThd0 = (int16_t)commandOption.data.commands.thread.threshold0[0] + 
// 										commandOption.data.commands.thread.threshold0[1] * 256;
// 					lightThd1 = (int16_t)commandOption.data.commands.thread.threshold1[0] + 
// 										commandOption.data.commands.thread.threshold1[1] * 256;
// 					if(commandOption.data.commands.thread.flashSave>0)
// 					{
// 						Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, lightThd0);
// 						Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, lightThd1);
// 					}		
// 				}
// 				commandReceive = I2C_CMD_NULL;
// 				break;
			
// 			default:
// 				break;
// 		}
// 	}else 
// 	{
// #endif

// 	if(commandReceive == I2C_CMD_SET_ADDR) // change i2c address
// 	{
// 		commandReceive = I2C_CMD_NULL;
// 		Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
// 		Wire.begin(deviceI2CAddress);
// 	}
// 	else if(commandReceive == I2C_CMD_RST_ADDR) // reset i2c address
// 	{
// 		commandReceive = I2C_CMD_NULL;
// 		deviceI2CAddress = Flash.read8(I2C_DEF_ADDR_FLASH_LOC);
// 		Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
// 		Wire.begin(deviceI2CAddress);
// 	}
// 	else if(commandReceive == I2C_CMD_SET_THD) // set new threshold
// 	{
// 		commandReceive = I2C_CMD_NULL;
//         if(flashSave)
//         {
//             Flash.write16(I2C_THD_0_ADDR_FLASH_LOC, lightThd0);
//             Flash.write16(I2C_THD_1_ADDR_FLASH_LOC, lightThd1);
//         }
// 	}

// #ifdef BLE_SUPPORT
// 	}
// #endif

// 	if(ledFlashCommand)
// 	{
// 		uint32_t ledFlashCurrentMillis = millis();
// 		if(ledFlashCurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
// 		{
// 			ledFlashPreviousMillis = ledFlashCurrentMillis;
// 			digitalWrite(LIGHT_LED_PIN_NUM, ledFlashStatus);
// 			ledFlashStatus = !ledFlashStatus;
// 			if(ledFlashTimes < 3)
// 			{
// 				if (!ledFlashStatus)
// 				{
// 					ledFlashTimes++;
// 					if (ledFlashTimes >= 3)ledFlashCommand = false;
// 				}
// 			}
// 		}
// 	}

// 	if(autoSleepFlag)
// 	{
// 		uint32_t autoSleepCurrentMillis = millis();
// 		if((autoSleepCurrentMillis - autoSleepPreviousMillis) > AUTO_SLEEP_TIMEOUT)
// 		{
// 			// ledFlashCommand = false;
// 			// ledFlashStatus = false;
// 			digitalWrite(LIGHT_LED_PIN_NUM, HIGH);
//             // disconnect soft i2c of digital light sensor
//             if (lightSensorVersion == 10)
//             {
//                 pinMode(SOFT_SCL_PIN, INPUT_PULLUP);
//     			pinMode(SOFT_SDA_PIN, INPUT_PULLUP);
//             }
//             // disconnect analog power pin, channel0/1/2
//             if (lightSensorVersion == 11)
//             {
//                 pinMode(ANALOG_CHANNEL_0_PIN, INPUT);
//                 pinMode(ANALOG_CHANNEL_1_PIN, INPUT);
//                 pinMode(ANALOG_CHANNEL_2_PIN, INPUT);
//                 analogLightPowerOff();
//             }

// 			wwdg.end();
// 			Wire.end();
//             // pullup hardware I2C bus
// 			pinMode(PA9, INPUT_PULLUP);
// 			pinMode(PA10, INPUT_PULLUP);
//             // close RTCTimer
//             // RTCTimer.end();
// 			nrgSave.standby();
//             // restart
//             // RTCTimer.begin(LIGHT_MIN_CONVERT_TIME, dummySample);
// 			Wire.begin(deviceI2CAddress);
// 			Wire.onReceive(receiveEvent);
// 			Wire.onRequest(requestEvent);
//             if (lightSensorVersion == 11)
//             {
//                 analogLightPowerOn();
//             }
// 			wwdg.begin();
// 		}
// 	}

//     if(testFlag)
//     {
//         wwdg.end();
//         pinMode(GROVE_TWO_TX_PIN_NUM, OUTPUT);
//         pinMode(GROVE_TWO_RX_PIN_NUM, OUTPUT);

//         while(1)
//         {
//             digitalWrite(GROVE_TWO_TX_PIN_NUM, HIGH);
//             digitalWrite(GROVE_TWO_RX_PIN_NUM, HIGH);
//             delay(1);
//             digitalWrite(GROVE_TWO_TX_PIN_NUM, LOW);
//             delay(1);

//             digitalWrite(GROVE_TWO_TX_PIN_NUM, HIGH);
//             digitalWrite(GROVE_TWO_RX_PIN_NUM, LOW);
//             delay(1);
//             digitalWrite(GROVE_TWO_TX_PIN_NUM, LOW);
//             delay(1);

//             if(testFlag == false)break;
//         }

//         wwdg.begin();
//         attachInterrupt(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE, INPUT_PULLUP);
//     }

// 	wwdg.reset();
// }

// void dummySample(void)
// {
// 	timeoutFlag = true;
// 	sampleFlag = !sampleFlag;
// }

// void dummy(void)
// {
// 	autoSleepPreviousMillis = millis();

//     if(digitalRead(GROVE_TWO_RX_PIN_NUM) == LOW)intStart = millis();
//     else
//     {
//         intEnd = millis();
//         if((intEnd - intStart) > 20)delay(500);
//         else intStart = intEnd;
//     }
// }

// void receiveEvent(int howMany)
// {
// 	uint8_t i = 0, receiveBuffer[MAINBOARD_BLE_I2C_DATALEN] = {0,};
// 	// autoSleepPreviousMillis = millis();

// 	while(Wire.available())
// 	{
// 		receiveBuffer[i ++] = Wire.read();
// 		if(i >= MAINBOARD_BLE_I2C_DATALEN)i = 0;
// 	}
// #ifdef BLE_SUPPORT
// 	if((receiveBuffer[0] >= MAINBOARD_BLE_COMMAND_LOW) && (receiveBuffer[0] <  MAINBOARD_BLE_COMMAND_HIGH))
// 	{// BLE command: len,cmd,opt,...
// 		Core_mode = CORE_BLE_MODE;
// 		memcpy(commandOption.bytes, receiveBuffer, i);
// 		commandOption.data.Header.Datalen -= MAINBOARD_BLE_COMMAND_LOW;
// 		if (i != commandOption.data.Header.Datalen)
// 		{// Bus error!!!
// 			return;
// 		}
// 		commandReceive = commandOption.data.Header.type;
// 	}else
// 	{
// 	Core_mode = CORE_ATMEL_MODE;
// #endif

// 	commandReceive = receiveBuffer[0];
// #ifdef BLE_SUPPORT
// 	}
// #endif
// 	switch(commandReceive)
// 	{
// #ifdef BLE_SUPPORT
// 			case I2C_CMD_GET_RAW_DATA:
// 			// Raw data request, 
				
// 				if (commandOption.data.commands.raw.Raw_data_type > 0)
// 				{
// 					LightRawData = commandReceive;
// 					RawDelayMillis = commandOption.data.commands.raw.delay[0]+commandOption.data.commands.raw.delay[1]*256+(chipId[0]&0x03);
// 					if (RawDelayMillis == 0)
// 						RawDelayMillis = I2C_CMD_RAW_DATA_TIME;
// 					RawPreviousMillis = 0;
// 				}else
// 					LightRawData = 0;
// 				commandReceive = I2C_CMD_NULL;
				
// 				break;
// #endif
// 		case I2C_CMD_SET_THD:
// 			{
// 				uint16_t lightThd = (int16_t)receiveBuffer[2] + receiveBuffer[3] * 256;
// 				if(receiveBuffer[1] == 0)lightThd0 = lightThd;
// 				else if(receiveBuffer[1] == 1)lightThd1 = lightThd;
//                 flashSave = receiveBuffer[4];
// 			}
// 		break;

// 		case I2C_CMD_LED_ON:
// 			ledFlashCommand = true;
// 			commandReceive = I2C_CMD_NULL;
// 		break;

// 		case I2C_CMD_LED_OFF:
// 			ledFlashCommand = false;
// 			ledFlashStatus = false;
// 			digitalWrite(LIGHT_LED_PIN_NUM, HIGH);
// 			commandReceive = I2C_CMD_NULL;
// 		break;

// 		case I2C_CMD_AUTO_SLEEP_ON:
// 			autoSleepFlag = true;
// 			commandReceive = I2C_CMD_NULL;
// 		break;

// 		case I2C_CMD_AUTO_SLEEP_OFF:
// 			autoSleepFlag = false;
// 			commandReceive = I2C_CMD_NULL;
// 		break;

// 		case I2C_CMD_SET_ADDR:
// 			deviceI2CAddress = receiveBuffer[1];
// 		break;

//         case I2C_CMD_TEST_TX_RX_ON:
//             testFlag = true;
// 			commandReceive = I2C_CMD_NULL;
//         break;

//         case I2C_CMD_TEST_TX_RX_OFF:
//             testFlag = false;
// 			commandReceive = I2C_CMD_NULL;
//         break;
		
// 		case I2C_CMD_JUMP_TO_BOOT:
// 			commandReceive = I2C_CMD_NULL;
// 			jumpToBootloader();
// 		break;
		
// 		default:
// 		break;
// 	}
// }

// void requestEvent(void)
// {
// 	// autoSleepPreviousMillis = millis();
	
// #ifdef BLE_SUPPORT
// 	Core_mode = CORE_ATMEL_MODE;
// #endif

// 	switch(commandReceive)
// 	{
// 		case I2C_CMD_GET_DEV_ID:
// 			Wire.write(ptr1, 4);
// 			commandReceive = I2C_CMD_NULL;
// 		break;

// 		case I2C_CMD_GET_DEV_EVENT:
// 			Wire.write(ptr1 + 4, 4);
// 			commandReceive = I2C_CMD_NULL;
// #ifdef BLE_SUPPORT
// 			preEvent = 0;
// #endif
// 		break;

// 		case I2C_CMD_GET_LIGHT:
// 			Wire.write((uint8_t)(curLightData & 0xff));
// 			Wire.write((uint8_t)(curLightData >> 8));
// 			commandReceive = I2C_CMD_NULL;
// 		break;

//         case I2C_CMD_TEST_GET_VER:
//             Wire.write(versions, 3);
//             commandReceive = I2C_CMD_NULL;
//         break;

//         case I2C_CMD_GET_DEVICE_UID:
//             Wire.write(chipId, 12);
//             commandReceive = I2C_CMD_NULL;
//         break;
        
// 		default:
// 		break;
// 	}
// }

// /***************************************************************

//  ***************************************************************/
// void i2cStart(void)
// {
// 	I2C_SDA_HIGH();
// 	I2C_WAIT();
// 	I2C_SCL_HIGH();
// 	I2C_WAIT();
// 	I2C_SDA_LOW();
// 	I2C_WAIT();
// 	I2C_SCL_LOW();
// 	I2C_WAIT();
// }

// void i2cStop(void)
// {
// 	I2C_SDA_LOW();
// 	I2C_WAIT();
// 	I2C_SCL_HIGH();
// 	I2C_WAIT();
// 	I2C_SDA_HIGH();
// 	I2C_WAIT();
// }

// bool i2cWaitAck(void)
// {
// 	bool ack, ret;

// 	I2C_SDA_IN();

// 	ack = I2C_SDA_READ();

// 	I2C_SCL_HIGH();
// 	I2C_WAIT();
// 	I2C_SCL_LOW();
// 	I2C_WAIT();

// 	I2C_SDA_OUT();

// 	ret = (ack == LOW)?true:false;
// 	return ret;
// }

// void i2cAck(void)
// {
// 	I2C_SDA_LOW();
// 	I2C_WAIT();
// 	I2C_SCL_HIGH();
// 	I2C_WAIT();
// 	I2C_SCL_LOW();
// 	I2C_WAIT();
// }

// void i2cNAck(void)
// {
// 	I2C_SDA_HIGH();
// 	I2C_WAIT();
// 	I2C_SCL_HIGH();
// 	I2C_WAIT();
// 	I2C_SCL_LOW();
// 	I2C_WAIT();
// }

// void i2cSendByte(uint8_t txd)
// {
// 	uint8_t i;

// 	for(i = 0; i < 8; i ++)
// 	{
// 		if(txd & 0x80)I2C_SDA_HIGH();
// 		else  I2C_SDA_LOW();
// 		txd <<= 1;

// 		I2C_SCL_HIGH();
// 		I2C_WAIT();
// 		I2C_SCL_LOW();
// 		I2C_WAIT();
// 	}
// }

// uint8_t i2cReadByte(void)
// {
// 	uint8_t i, res = 0;

// 	I2C_SDA_IN();

// 	for(i = 0; i < 8; i ++)
// 	{
// 		res <<= 1;
// 		if(I2C_SDA_READ())res ++;

// 		I2C_SCL_HIGH();
// 		I2C_WAIT();
// 		I2C_SCL_LOW();
// 		I2C_WAIT();
// 	}

// 	I2C_SDA_OUT();

// 	return res;
// }

// uint8_t tsl2561tReadData(uint8_t dataAddr)
// {
// 	uint8_t data = 0;

// 	i2cStart();
// 	i2cSendByte((TLS2561T_I2C_ADDR << 1) | 0x00);
// 	i2cWaitAck();
// 	i2cSendByte(dataAddr);
// 	i2cWaitAck();
// 	i2cStart();
// 	i2cSendByte((TLS2561T_I2C_ADDR << 1) | 0x01);
// 	i2cWaitAck();
// 	data = i2cReadByte();
// 	i2cAck();
// 	i2cStop();

// 	return data;
// }

// void tsl2561tReadDataNumber(uint8_t dataAddr, uint8_t dataLen, uint8_t *buffer)
// {
// 	uint8_t i;

// 	i2cStart();
// 	i2cSendByte((TLS2561T_I2C_ADDR << 1) | 0x00);
// 	i2cWaitAck();
// 	i2cSendByte(dataAddr);
// 	i2cWaitAck();
// 	i2cStart();
// 	i2cSendByte((TLS2561T_I2C_ADDR << 1) | 0x01);
// 	i2cWaitAck();
// 	for(i = 0; i < (dataLen - 1); i++)
// 	{
// 		buffer[i] = i2cReadByte();
// 		i2cAck();
// 	}
// 	buffer[i] = i2cReadByte();
// 	i2cNAck();
// 	i2cStop();
// }

// void tsl2561tWriteData(uint8_t dataAddr, uint8_t dataValue)
// {
// 	i2cStart();
// 	i2cSendByte((TLS2561T_I2C_ADDR<<1) | 0x00);
// 	i2cWaitAck();
// 	i2cSendByte(dataAddr);
// 	i2cWaitAck();
// 	i2cSendByte(dataValue);
// 	i2cWaitAck();
// 	i2cStop();
// }

// void tsl2561tInit(void)
// {
//     tsl2561tWriteData(TSL2561T_TIMING, 0x00);		// No High Gain(1x), integration time of 13ms
//     tsl2561tWriteData(TSL2561T_INTERRUPT, 0x00);	// Disable interrupt
// }

// void tsl2561tPowerOn(void)
// {
// 	tsl2561tWriteData(TSL2561T_CONTROL, 0x03);
// }

// void tsl2561tPowerOff(void)
// {
// 	tsl2561tWriteData(TSL2561T_CONTROL, 0x00);
// }

// uint32_t calculateLux(uint8_t iGain, uint8_t tInt, uint8_t iType, uint16_t ch0, uint16_t ch1)
// {
// 	uint32_t chScale, channel0, channel1, lux, ratio1;
// 	int32_t ratio, temp;
// 	int16_t b, m;

// 	switch(tInt)
// 	{
// 		case 0:  // 13.7 msec
// 			chScale = CHSCALE_TINT0;
// 		break;

// 		case 1: // 101 msec
// 			chScale = CHSCALE_TINT1;
// 		break;

// 		default: // assume no scaling
// 			chScale = (1 << CH_SCALE);
// 		break;
// 	}

// 	// Scale 1X to 16X
// 	if(!iGain)chScale = chScale << 4;

// 	// Scale the channel values
// 	channel0 = (ch0 * chScale) >> CH_SCALE;
// 	channel1 = (ch1 * chScale) >> CH_SCALE;

// 	ratio1 = 0;
// 	if(channel0 != 0)ratio1 = (channel1 << (RATIO_SCALE+1)) / channel0;

// 	// Round the ratio value
// 	ratio = (ratio1 + 1) >> 1;

// 	switch(iType)
// 	{
// 		case 0: // T package
// 			if((ratio >= 0) && (ratio <= K1T)){ b = B1T; m = M1T; }
// 			else if(ratio <= K2T){ b = B2T; m = M2T; }
// 			else if(ratio <= K3T){ b = B3T; m = M3T; }
// 			else if(ratio <= K4T){ b = B4T; m = M4T; }
// 			else if(ratio <= K5T){ b = B5T; m = M5T; }
// 			else if(ratio <= K6T){ b = B6T; m = M6T; }
// 			else if(ratio <= K7T){ b = B7T; m = M7T; }
// 			else if(ratio > K8T){ b = B8T; m = M8T; }
// 		break;

// 		case 1:// CS package
// 			if ((ratio >= 0) && (ratio <= K1C)) { b = B1C; m = M1C; }
// 			else if(ratio <= K2C){ b = B2C; m = M2C; }
// 			else if(ratio <= K3C){ b = B3C; m = M3C; }
// 			else if(ratio <= K4C){ b = B4C; m = M4C; }
// 			else if(ratio <= K5C){ b = B5C; m = M5C; }
// 			else if(ratio <= K6C){ b = B6C; m = M6C; }
// 			else if(ratio <= K7C){ b = B7C; m = M7C; }
// 		break;
// 	}

// 	temp = ((channel0 * b) - (channel1 * m));

// 	if(temp < 0)temp = 0;
// 	temp += (1 << (LUX_SCALE - 1));

// 	// Strip off fractional portion
// 	lux = temp >> LUX_SCALE;

// 	return (lux);
// }

// uint16_t tsl2561tGetLightData(void)
// {
// 	uint8_t buffer[4];
// 	uint16_t ch0, ch1;
// 	uint32_t lux;

// 	tsl2561tReadDataNumber(TSL2561T_CHANNAL0L, 4, buffer);

// 	ch0 = buffer[0] + buffer[1] * 256;
// 	ch1 = buffer[2] + buffer[3] * 256;

// 	if(ch0 > 4900 && (ch0 / ch1) < 2)// Ch0 out of range, but ch1 not. the lux is not valid in this situation.
// 	{
// 		return 0;
// 	}

// 	lux = calculateLux(0, 0, 0, ch0, ch1);

// 	return (uint16_t)lux;
// }

// uint8_t checkLightSensorVersion(void)
// {
//     uint8_t version;
//     pinMode(LIGHT_VERSION_PIN_NUM, INPUT_PULLUP);
//     pinMode(ANALOG_CHANNEL_0_PIN, OUTPUT);
//     digitalWrite(ANALOG_CHANNEL_0_PIN, LOW);

//     delay(1);
//     if(!digitalRead(LIGHT_VERSION_PIN_NUM)) {
//         version = 11;        //version1.1 analog light sensor
//     }
//     else {
//         version = 10;
//     }
//     // disconnect pins
//     pinMode(LIGHT_VERSION_PIN_NUM, INPUT);
//     pinMode(ANALOG_CHANNEL_0_PIN, INPUT);
//     return version;
// }


// uint16_t analogGetLightData(void)
// {
//     uint32_t value = 0;
//     analogSwitchLightGain(0);
//     uint16_t adcLight0 = analogRead(ANALOG_DATA_PIN);

//     //calculate lux with analog data
//     //y=0.2391x
//     if (adcLight0 <= 250)
//     {
//       value = (adcLight0 * FORMULA_0_A) / 10000;
//       return (uint16_t)value;
//     }

//     //y = 57(lux)
//     if ((adcLight0 > 250) && (adcLight0 < 300))
//     {
//       value = 57;
//       return (uint16_t)value;
//     }

//     //y=0.2988x-31.147
//     if (adcLight0 <= 900)
//     {
//       value = (adcLight0 * FORMULA_1_A - FORMULA_1_B) / 10000;
//       return (uint16_t)value;
//     }

//     analogSwitchLightGain(1);
//     uint16_t adcLight1 = analogRead(ANALOG_DATA_PIN);

//     //y=2.2961x-23.053
//     if (adcLight1 <= 881)
//     {
//       value = (adcLight1 * FORMULA_2_A - FORMULA_2_B) / 10000;
//       return (uint16_t)value;
//     }

//     analogSwitchLightGain(2);
//     uint16_t adcLight2 = analogRead(ANALOG_DATA_PIN);

//     //y=20.952x-77.187
//     if (adcLight2 < 720)
//     {
//       value = (adcLight2 * FORMULA_3_A - FORMULA_3_B) / 1000;
//       return (uint16_t)value;
//     }

//     //y=150.64x-93145
//     if (adcLight2 >= 720)
//     {
//       value = (adcLight2 * FORMULA_4_A / 100) - FORMULA_4_B;
//       return (uint16_t)value;
//     }

//     return 0;

// }

// /*************************************************************
// * Description
// *    Switch gain channels to detect different range of light intensity
// * Parameter
// *    gain: when gain = 0, the range is 0-320lux; when gain = 1, the range
// *          is 320-2000lux; when gain = 2, the range is 2000-100k lux.
// * Return
// *    Null.
// *************************************************************/
// void analogSwitchLightGain(uint8_t gain)
// {
//     switch (gain) {
//       case 0:
//         pinMode(ANALOG_CHANNEL_0_PIN, OUTPUT);
//         digitalWrite(ANALOG_CHANNEL_0_PIN, LOW);
//         pinMode(ANALOG_CHANNEL_1_PIN, INPUT);
//         pinMode(ANALOG_CHANNEL_2_PIN, INPUT);
//         break;
//       case 1:
//         pinMode(ANALOG_CHANNEL_0_PIN, INPUT);
//         pinMode(ANALOG_CHANNEL_1_PIN, OUTPUT);
//         digitalWrite(ANALOG_CHANNEL_1_PIN, LOW);
//         pinMode(ANALOG_CHANNEL_2_PIN, INPUT);
//         break;
//       case 2:
//         pinMode(ANALOG_CHANNEL_0_PIN, INPUT);
//         pinMode(ANALOG_CHANNEL_1_PIN, INPUT);
//         pinMode(ANALOG_CHANNEL_2_PIN, OUTPUT);
//         digitalWrite(ANALOG_CHANNEL_2_PIN, LOW);
//         break;
//       default:
//         // do nothing
//         break;
//     }
//     //dalay 3ms for stable
//     delay(3);
// }

// void analogLightInit(void)
// {
//     // set power pin
//     pinMode(ANALOG_POWER_PIN, OUTPUT);
//     analogLightPowerOn();
//     // set gain 0
//     analogSwitchLightGain(0);

// }

// void analogLightPowerOn(void)
// {
//     digitalWrite(ANALOG_POWER_PIN, LOW);
// }

// void analogLightPowerOff(void)
// {
//     digitalWrite(ANALOG_POWER_PIN, HIGH);
// }


