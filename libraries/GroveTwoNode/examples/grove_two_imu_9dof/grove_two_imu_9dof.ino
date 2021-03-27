
#include <SPI.h>
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>
#include <float.h>
#include <math.h>

#define DEBUG_MAGNET

#define GROVE_TWO_TX_PIN_NUM	PA2
#define GROVE_TWO_RX_PIN_NUM	PA3
#define IMU_LED_PIN_NUM			PA1

/***************************************************************

 ***************************************************************/
#define CHIP_CS_PIN		PA4
#define CHIP_INT_PIN	PA0

#define chipEnable()	digitalWrite(CHIP_CS_PIN, LOW)
#define chipDisable()	digitalWrite(CHIP_CS_PIN, HIGH)

// Register map for gyroscope and accelerometer
#define MPU9250_SELF_TEST_X_GYRO        0x00
#define MPU9250_SELF_TEST_Y_GYRO        0x01
#define MPU9250_SELF_TEST_Z_GYRO        0x02

#define MPU9250_SELF_TEST_X_ACCEL       0x0D
#define MPU9250_SELF_TEST_Y_ACCEL       0x0E
#define MPU9250_SELF_TEST_Z_ACCEL       0x0F

#define MPU9250_XG_OFFSET_H             0x13
#define MPU9250_XG_OFFSET_L             0x14
#define MPU9250_YG_OFFSET_H             0x15
#define MPU9250_YG_OFFSET_L             0x16
#define MPU9250_ZG_OFFSET_H             0x17
#define MPU9250_ZG_OFFSET_L             0x18
#define MPU9250_SMPLRT_DIV              0x19
#define MPU9250_CONFIG                  0x1A
#define MPU9250_GYRO_CONFIG             0x1B
#define MPU9250_ACCEL_CONFIG            0x1C
#define MPU9250_ACCEL_CONFIG2           0x1D
#define MPU9250_LP_ACCEL_ODR            0x1E
#define MPU9250_WOM_THR                 0x1F

#define MPU9250_FIFO_EN                 0x23
#define MPU9250_I2C_MST_CTRL            0x24
#define MPU9250_I2C_SLV0_ADDR           0x25
#define MPU9250_I2C_SLV0_REG            0x26
#define MPU9250_I2C_SLV0_CTRL           0x27
#define MPU9250_I2C_SLV1_ADDR           0x28
#define MPU9250_I2C_SLV1_REG            0x29
#define MPU9250_I2C_SLV1_CTRL           0x2A
#define MPU9250_I2C_SLV2_ADDR           0x2B
#define MPU9250_I2C_SLV2_REG            0x2C
#define MPU9250_I2C_SLV2_CTRL           0x2D
#define MPU9250_I2C_SLV3_ADDR           0x2E
#define MPU9250_I2C_SLV3_REG            0x2F
#define MPU9250_I2C_SLV3_CTRL           0x30
#define MPU9250_I2C_SLV4_ADDR           0x31
#define MPU9250_I2C_SLV4_REG            0x32
#define MPU9250_I2C_SLV4_DO             0x33
#define MPU9250_I2C_SLV4_CTRL           0x34
#define MPU9250_I2C_SLV4_DI             0x35
#define MPU9250_I2C_MST_STATUS          0x36
#define MPU9250_INT_PIN_CFG             0x37
#define MPU9250_INT_ENABLE              0x38

#define MPU9250_INT_STATUS              0x3A
#define MPU9250_ACCEL_XOUT_H            0x3B
#define MPU9250_ACCEL_XOUT_L            0x3C
#define MPU9250_ACCEL_YOUT_H            0x3D
#define MPU9250_ACCEL_YOUT_L            0x3E
#define MPU9250_ACCEL_ZOUT_H            0x3F
#define MPU9250_ACCEL_ZOUT_L            0x40
#define MPU9250_TEMP_OUT_H              0x41
#define MPU9250_TEMP_OUT_L              0x42
#define MPU9250_GYRO_XOUT_H             0x43
#define MPU9250_GYRO_XOUT_L             0x44
#define MPU9250_GYRO_YOUT_H             0x45
#define MPU9250_GYRO_YOUT_L             0x46
#define MPU9250_GYRO_ZOUT_H             0x47
#define MPU9250_GYRO_ZOUT_L             0x48
#define MPU9250_EXT_SENS_DATA_00        0x49
#define MPU9250_EXT_SENS_DATA_01        0x4A
#define MPU9250_EXT_SENS_DATA_02        0x4B
#define MPU9250_EXT_SENS_DATA_03        0x4C
#define MPU9250_EXT_SENS_DATA_04        0x4D
#define MPU9250_EXT_SENS_DATA_05        0x4E
#define MPU9250_EXT_SENS_DATA_06        0x4F
#define MPU9250_EXT_SENS_DATA_07        0x50
#define MPU9250_EXT_SENS_DATA_08        0x51
#define MPU9250_EXT_SENS_DATA_09        0x52
#define MPU9250_EXT_SENS_DATA_10        0x53
#define MPU9250_EXT_SENS_DATA_11        0x54
#define MPU9250_EXT_SENS_DATA_12        0x55
#define MPU9250_EXT_SENS_DATA_13        0x56
#define MPU9250_EXT_SENS_DATA_14        0x57
#define MPU9250_EXT_SENS_DATA_15        0x58
#define MPU9250_EXT_SENS_DATA_16        0x59
#define MPU9250_EXT_SENS_DATA_17        0x5A
#define MPU9250_EXT_SENS_DATA_18        0x5B
#define MPU9250_EXT_SENS_DATA_19        0x5C
#define MPU9250_EXT_SENS_DATA_20        0x5D
#define MPU9250_EXT_SENS_DATA_21        0x5E
#define MPU9250_EXT_SENS_DATA_22        0x5F
#define MPU9250_EXT_SENS_DATA_23        0x60

#define MPU9250_I2C_SLV0_DO             0x63
#define MPU9250_I2C_SLV1_DO             0x64
#define MPU9250_I2C_SLV2_DO             0x65
#define MPU9250_I2C_SLV3_DO             0x66
#define MPU9250_I2C_MST_DELAY_CTRL      0x67
#define MPU9250_SIGNAL_PATH_RESET       0x68
#define MPU9250_MOT_DETECT_CTRL         0x69
#define MPU9250_USER_CTRL               0x6A
#define MPU9250_PWR_MGMT_1              0x6B
#define MPU9250_PWR_MGMT_2              0x6C

#define MPU9250_FIFO_COUNTH             0x72
#define MPU9250_FIFO_COUNTL             0x73
#define MPU9250_FIFO_R_W                0x74
#define MPU9250_WHO_AM_I                0x75
#define MPU9250_XA_OFFSET_H             0x77
#define MPU9250_XA_OFFSET_L             0x78

#define MPU9250_YA_OFFSET_H             0x7A
#define MPU9250_YA_OFFSET_L             0x7B

#define MPU9250_ZA_OFFSET_H             0x7D
#define MPU9250_ZA_OFFSET_L             0x7E
//
#define MPU9250_I2C_READ 0x80

// Register map for magnetometer
#define MPU9250_AK8963_WIA				0x00
#define MPU9250_AK8963_INFO				0x01
#define MPU9250_AK8963_ST1				0x02
#define MPU9250_AK8963_XOUT_L			0x03
#define MPU9250_AK8963_XOUT_H			0x04
#define MPU9250_AK8963_YOUT_L			0x05
#define MPU9250_AK8963_YOUT_H			0x06
#define MPU9250_AK8963_ZOUT_L			0x07
#define MPU9250_AK8963_ZOUT_H			0x08
#define MPU9250_AK8963_ST2				0x09
#define MPU9250_AK8963_CNTL				0x0A
#define MPU9250_AK8963_CNTL2			0x0B
#define MPU9250_AK8963_RSV				0x0B
#define MPU9250_AK8963_ASTC				0x0C
#define MPU9250_AK8963_TS1				0x0D
#define MPU9250_AK8963_TS2				0x0E
#define MPU9250_AK8963_I2CDIS			0x0F
#define MPU9250_AK8963_ASAX				0x10
#define MPU9250_AK8963_ASAY				0x11
#define MPU9250_AK8963_ASAZ				0x12

#define MPU9250_AK8963_I2C_ADDR					0x0C
#define MPU9250_AK8963_POWER_DOWN				0x10
#define MPU9250_AK8963_FUSE_ROM_ACCESS			0x1F
#define MPU9250_AK8963_SINGLE_MEASUREMENT		0x11
#define MPU9250_AK8963_CONTINUOUS_MEASUREMENT	0x16
#define MPU9250_AK8963_DATA_READY				0x01
#define MPU9250_AK8963_DATA_OVERRUN				0x02
#define MPU9250_AK8963_OVERFLOW					0x80
#define MPU9250_AK8963_DATA_ERROR				0x40
#define MPU9250_AK8963_CNTL2_SRST				0x01

#define MPU9250_I2C_SLV4_EN			0x80
#define MPU9250_I2C_SLV4_DONE		0x40
#define MPU9250_I2C_SLV4_NACK		0x10

#define MPU9250_I2C_IF_DIS			0x10
#define MPU9250_I2C_MST_EN			0x20
#define MPU9250_FIFO_RST			0x04
#define MPU9250_FIFO_ENABLE			0x40

#define MPU9250_RESET				0x80
#define MPU9250_CLOCK_MASK			0xF8
#define MPU9250_CLOCK_INTERNAL		0x00
#define MPU9250_CLOCK_PLL			0x01
#define MPU9250_CLOCK_PLLGYROZ		0x03
#define MPU9250_FS_SEL_MASK			0xE7
#define MPU9250_SLEEP_MASK			0x40

#define MPU9250_XYZ_GYRO			0xC7
#define MPU9250_XYZ_ACCEL			0xF8

#define MPU9250_RAW_RDY_EN			0x01
#define MPU9250_RAW_DATA_RDY_INT	0x01
#define MPU9250_FIFO_OVERFLOW		0x10

#define MPU9250_INT_ANYRD_2CLEAR	0x10
#define MPU9250_LATCH_INT_EN		0x20

#define MPU9250_MAX_FIFO			1024
#define MPU9250_FIFO_SIZE_1024		0x40
#define MPU9250_FIFO_SIZE_2048		0x80
#define MPU9250_FIFO_SIZE_4096		0xC0

#define MPU9250_TEMP_OUT			0x80
#define MPU9250_GYRO_XOUT			0x40
#define MPU9250_GYRO_YOUT			0x20
#define MPU9250_GYRO_ZOUT			0x10
#define MPU9250_ACCEL				0x08

#define SMPLRT_DIV					0

enum MPU9250_ACCEL_FSR
{
    MPU9250_FSR_2G = 0,
    MPU9250_FSR_4G,
    MPU9250_FSR_8G,
    MPU9250_FSR_16G,
    MPU9250_NUM_ACCEL_FSR
};

enum MPU9250_ACCEL_DLPF
{
    MPU9250_ACCEL_DLPF_460HZ = 0,
    MPU9250_ACCEL_DLPF_184HZ,
    MPU9250_ACCEL_DLPF_92HZ,
    MPU9250_ACCEL_DLPF_41HZ,
    MPU9250_ACCEL_DLPF_20HZ,
    MPU9250_ACCEL_DLPF_10HZ,
    MPU9250_ACCEL_DLPF_5HZ,
    MPU9250_ACCEL_DLPF_460HZ2,
    MPU9250_NUM_ACCEL_DLPF
};

enum MPU9250_GYRO_FSR
{
    MPU9250_FSR_250DPS = 0,
    MPU9250_FSR_500DPS,
    MPU9250_FSR_1000DPS,
    MPU9250_FSR_2000DPS,
    MPU9250_NUM_GYRO_FSR
};

enum MPU9250_GYRO_DLPF
{
    MPU9250_GYRO_DLPF_250HZ = 0,
    MPU9250_GYRO_DLPF_184HZ,
    MPU9250_GYRO_DLPF_92HZ,
    MPU9250_GYRO_DLPF_41HZ,
    MPU9250_GYRO_DLPF_20HZ,
    MPU9250_GYRO_DLPF_10HZ,
    MPU9250_GYRO_DLPF_5HZ,
    MPU9250_GYRO_DLPF_3600HZ,
    NUM_GYRO_DLPF
};

enum MPU9250_CLK
{
    MPU9250_CLK_INTERNAL = 0,
    MPU9250_CLK_PLL,
    MPU9250_NUM_CLK
};

bool mpu9250IsNewData = false;
uint16_t mpu9250ak8963asa[3] = {0, 0, 0};

#define MPU9250_MAGNET_RANGE	4800 // +/- 4800 uT
#define MAGNET_DATA_READY		135  // 8 Hz
#define CALI_PREPARE_TIME		4000

uint8_t accelFSR = (uint8_t)MPU9250_FSR_8G;
uint8_t accelDLPF = (uint8_t)MPU9250_ACCEL_DLPF_10HZ;
uint8_t gyroFSR = (uint8_t)MPU9250_FSR_1000DPS;
uint8_t gyroDLPF = (uint8_t)MPU9250_GYRO_DLPF_10HZ;

uint16_t sample_count = 0;
int32_t mag_bias[3] = {0, 0, 0};
int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
uint8_t data[8];

int16_t accel[3] = {0,};
int16_t gyro[3] = {0, };
int16_t mag[3] = {0, };
int16_t accelTemp[3] = {0,};
int16_t gyroTemp[3] = {0, };
int16_t magTemp[3] = {0, };
bool imuDataUpdateFlag = false;
uint16_t heading = 0;
int32_t MPU9250magBias[3] = {0, 0, 0};      // Bias corrections for mag
int32_t MPU9250gyroBias[3] = {0, 0, 0};      // Bias corrections for gyro
int32_t MPU9250accelBias[3] = {0, 0, 0};      // Bias corrections for accelerometer

uint8_t mpu9250SendByte(uint8_t data);
uint8_t mpu9250Read(uint8_t regAddr);
void mpu9250Reads(uint8_t regAddr, uint8_t len, uint8_t *data);
void mpu9250Write(uint8_t regAddr, uint8_t data);
void mpu9250Writes(uint8_t regAddr, uint8_t len, uint8_t *data);

int16_t ak8963Read(uint8_t akmAddr, uint8_t regAddr, uint8_t *data);
int16_t ak8963Reads(uint8_t akmAddr, uint8_t regAddr, uint8_t len, uint8_t *data);
int16_t ak8963Write(uint8_t akmAddr, uint8_t regAddr, uint8_t data);
int16_t ak8963Writes(uint8_t akmAddr, uint8_t regAddr, uint8_t len, uint8_t *data);

void mpu9250Init(void);
bool mpu9250IsDataReady(void);
void mpu9250Get9AxisRawData(int16_t *accel, int16_t *gyro, int16_t *mag);
void mpu9250Get6AxisRawData(int16_t *accel, int16_t*gyro);
void mpu9250Get3AxisAccelRawData(int16_t *accel);
void mpu9250Get3AxisGyroRawData(int16_t *gyro);
void mpu9250Get3AxisMagnetRawData(int16_t *mag);
void mpu9250GetTemperatureRawData(int16_t *temperature);
void mpu9250SetAccelRange(MPU9250_ACCEL_FSR range);
void mpu9250SetAccelRate(MPU9250_ACCEL_DLPF rate);
void mpu9250SetGyroRange(MPU9250_GYRO_FSR range);
void mpu9250SetGyroRate(MPU9250_GYRO_DLPF rate);
void mpu9250PowerDown(void);
void mpu9250PowerUp(void);
void magcalMPU9250(void * dest1);
int16_t convertRawAccel(int16_t aRaw);
int16_t convertRawGyro(int16_t gRaw);
int16_t convertRawMagnet(int16_t mRaw);
void accelgyrocalMPU9250(int32_t * dest1, int32_t * dest2);
uint16_t atan2approx(int16_t y, int16_t x);

/***************************************************************

 ***************************************************************/
#define DEVICE_I2C_ADDRESS		0x04
#define DEVICE_VID				0x2886
#define DEVICE_PID				0x0004

#define I2C_DEF_ADDR_FLASH_LOC	0x00 // by byte
#define I2C_CUR_ADDR_FLASH_LOC	0x01 // by byte

#define ACCEL_FSR_FLASH_LOC		0x02 // by byte
#define ACCEL_DLPF_FLASH_LOC	0x03 // by byte
#define GYRO_FSR_FLASH_LOC		0x04 // by byte
#define GYRO_DLPF_FLASH_LOC		0x05 // by byte

#define I2C_CMD_GET_DEV_ID		0x00 //
#define I2C_CMD_GET_DEV_EVENT	0x01 //

#define I2C_CMD_GET_ACCEL_X		0x02 //
#define I2C_CMD_GET_ACCEL_Y		0x03 //
#define I2C_CMD_GET_ACCEL_Z		0x04 //

#define I2C_CMD_GET_GYRO_X		0x05 //
#define I2C_CMD_GET_GYRO_Y		0x06 //
#define I2C_CMD_GET_GYRO_Z		0x07 //

#define I2C_CMD_GET_MAG_X		0x08 //
#define I2C_CMD_GET_MAG_Y		0x09 //
#define I2C_CMD_GET_MAG_Z		0x0a //

#define I2C_CMD_GET_ACCEL_X_Y_Z	0x0b //
#define I2C_CMD_GET_GYRO_X_Y_Z	0x0c //
#define I2C_CMD_GET_MAG_X_Y_Z	0x0d //
#define I2C_CMD_GET_ALL_X_Y_Z	0x0e //

#define I2C_CMD_GET_HEADING		0x0f //
#define I2C_CMD_GET_ROTATION	0x10 //

#define I2C_CMD_SET_RANGE		0x11 //
#define I2C_CMD_SET_RATE		0x12 //

#define I2C_CMD_CALI_START      0x13 //

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

#define I2C_CMD_RAW_DATA_TIME 		200

uint8_t Core_mode = 0, ErrorCount=0;
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
	uint8_t Raw_data_type;
	uint8_t		delay[2];
}packet_raw_t;

typedef struct
{
	uint8_t threshold0[2];
	uint8_t threshold1[2];
	uint8_t flashSave;
}packet_thlsd_t;

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
		uint8_t Option[MAINBOARD_BLE_I2C_DATALEN-3];
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
	uint8_t		data[6];
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
uint32_t RawDelayMillis = 0;
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

 ***************************************************************/
LowPower nrgSave;

#define AUTO_SLEEP_TIMEOUT	2000

uint32_t clickCheckPreviousMillis = 0, PreMillis = 0;
uint32_t autoSleepPreviousMillis = 0, CalibrationMillis = 0, CaliPreMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

/***************************************************************

 ***************************************************************/
#define LED_FLASH_TIME	250

bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;

/***************************************************************

 ***************************************************************/
#define TILT_X_P_MAX	(1200)
#define TILT_X_P_MIN	(800)
#define TILT_X_N_MAX	(-800)
#define TILT_X_N_MIN	(-1200)

#define TILT_Y_P_MAX	(1200)
#define TILT_Y_P_MIN	(800)
#define TILT_Y_N_MAX	(-800)
#define TILT_Y_N_MIN	(-1200)

#define TILT_Z_P_MAX	(1200)
#define TILT_Z_P_MIN	(800)
#define TILT_Z_N_MAX	(-800)
#define TILT_Z_N_MIN	(-1200)

// Free fall thresh: 300 - 600 mG
// Free fall time: 100 - 350 ms
#define FREE_FALL_MAX	(600)
#define FREE_FALL_MIN	(300)
#define FREE_FALL_TIME_MAX (350)
#define FREE_FALL_TIME_MAX (100)

#define SHAKE_DET_MIN   300
uint8_t numShake = 0;
uint8_t countShake = 0;

#define IMU_GET_DATA_TIMUOUT 25

uint32_t imuPreviousMillis = 0;

bool testFlag = false;
char *versions = "V20";
uint16_t NodeVersion = 0x6102;

uint32_t intStart = 0;
uint32_t intEnd = 0;

/***************************************************************

 ***************************************************************/
uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/***************************************************************

 ***************************************************************/
void setup()
{
	uint8_t i2cDefaultAddr = Flash.read8(I2C_DEF_ADDR_FLASH_LOC);
	uint8_t i2cCurrentAddr = Flash.read8(I2C_CUR_ADDR_FLASH_LOC);

	delay(1000);
    uint8_t *ptr2 = (uint8_t *)Flash.getChipUniqueID();
    for(uint8_t i = 0; i < 12; i ++)chipId[i] = *(ptr2 + i);

	uint8_t accelFSRTemp = Flash.read8(ACCEL_FSR_FLASH_LOC);
	uint8_t accelDLPFTemp = Flash.read8(ACCEL_DLPF_FLASH_LOC);
	uint8_t gyroFSRTemp = Flash.read8(GYRO_FSR_FLASH_LOC); ;
	uint8_t gyroDLPFTemp = Flash.read8(GYRO_DLPF_FLASH_LOC);

	if(i2cDefaultAddr == 0xff)Flash.write8(I2C_DEF_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	if(i2cCurrentAddr == 0xff)Flash.write8(I2C_CUR_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
	else deviceI2CAddress = i2cCurrentAddr;

	if(accelFSRTemp == 0xff)Flash.write8(ACCEL_FSR_FLASH_LOC, (uint8_t)MPU9250_FSR_2G);
	else accelFSR = accelFSRTemp;
	if(accelDLPFTemp == 0xff)Flash.write8(ACCEL_DLPF_FLASH_LOC, (uint8_t)MPU9250_ACCEL_DLPF_10HZ);
	else accelDLPF = accelDLPFTemp;

	if(gyroFSRTemp == 0xff)Flash.write8(GYRO_FSR_FLASH_LOC, (uint8_t)MPU9250_FSR_250DPS);
	else gyroFSR = gyroFSRTemp;
	if(gyroDLPFTemp == 0xff)Flash.write8(GYRO_DLPF_FLASH_LOC, (uint8_t)MPU9250_GYRO_DLPF_10HZ);
	else gyroDLPF = gyroDLPFTemp;

	packet_01_data.data.deviceVID = DEVICE_VID;
	packet_01_data.data.devicePID = DEVICE_PID;
	packet_01_data.data.deviceEvent = 0;

	nrgSave.begin(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance

	pinMode(IMU_LED_PIN_NUM, OUTPUT);
	digitalWrite(IMU_LED_PIN_NUM, HIGH);

	pinMode (CHIP_CS_PIN, OUTPUT);
	digitalWrite(CHIP_CS_PIN, HIGH);

	pinMode(CHIP_INT_PIN, INPUT_PULLUP);

	SPI.begin();

	mpu9250Init();

#ifdef BLE_SUPPORT
	StartMillis = millis();
#endif

	Wire.begin(deviceI2CAddress);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);

	wwdg.begin();
}

void loop()
{
	uint32_t imuCurrentMillis = millis();
	uint16_t Value;

	if(Wire.isbusidle())PreMillis = imuCurrentMillis;
	if ((imuCurrentMillis - PreMillis) > 20)
	{
		Wire.reset();
		Wire.begin(deviceI2CAddress);
		Wire.onReceive(receiveEvent);
		Wire.onRequest(requestEvent);
		PreMillis = imuCurrentMillis;
	}
	if (CalibrationMillis > 0)
	{
		if ((imuCurrentMillis - CalibrationMillis >= CALI_PREPARE_TIME) && (CaliPreMillis == 0))
		{
			CaliPreMillis = imuCurrentMillis;
			CalibrationMillis = imuCurrentMillis;
		}
		if(CaliPreMillis > 0)
		{
			if((imuCurrentMillis - CaliPreMillis) > MAGNET_DATA_READY)
			{
				mpu9250Reads(MPU9250_EXT_SENS_DATA_00, 8, data);

				mag_temp[0] = (data[2] << 8) | data[1];
				mag_temp[1] = (data[4] << 8) | data[3];
				mag_temp[2] = (data[6] << 8) | data[5];

				for (int jj = 0; jj < 3; jj++) {
					if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
					if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
				}
				CaliPreMillis = imuCurrentMillis;
				sample_count++;
			}
			if (sample_count >= 64)
			{
				mag_bias[0]  = (mag_max[0] + mag_min[0])/2 + mag_min[0];  // get average x mag bias in counts
				mag_bias[1]  = (mag_max[1] + mag_min[1])/2 + mag_min[1];  // get average y mag bias in counts
				mag_bias[2]  = (mag_max[2] + mag_min[2])/2 + mag_min[2];  // get average z mag bias in counts
#if 0
				MPU9250magBias[0] = mag_bias[0];
				MPU9250magBias[1] = mag_bias[1];
				MPU9250magBias[2] = mag_bias[2];
#else
	// 1.19 1.19 1.19 		CC -2B -B0 		24 -3 -1A 

				MPU9250magBias[0] = (mag_bias[0]*mpu9250ak8963asa[0]) >> 8;
				MPU9250magBias[1] = (mag_bias[1]*mpu9250ak8963asa[1]) >> 8;
				MPU9250magBias[2] = (mag_bias[2]*mpu9250ak8963asa[2]) >> 8;
				MPU9250magBias[0] = convertRawMagnet(MPU9250magBias[0]);
				MPU9250magBias[1] = convertRawMagnet(MPU9250magBias[1]);
				MPU9250magBias[2] = convertRawMagnet(MPU9250magBias[2]);
#endif
				InstructionOption.event.Header.type = I2C_CMD_NOTIFY_EVENT;
				InstructionOption.event.Header.Address = deviceI2CAddress;
				InstructionOption.event.Event  = 0x80;
				InstructionOption.event.Header.Datalen	 = sizeof(packet_event);
				Wire.MasterGPIOTransmission(ptr2, sizeof(packet_event));
				CalibrationMillis = 0;
				CaliPreMillis	= 0;
			}
		}
	}
	if(imuCurrentMillis - imuPreviousMillis >= IMU_GET_DATA_TIMUOUT)
	{
		imuPreviousMillis = imuCurrentMillis;

		imuDataUpdateFlag = false;

		mpu9250Get9AxisRawData(accelTemp, gyroTemp, magTemp);
		for(uint8_t i = 0; i < 3; i ++)
		{
			accelTemp[i] = convertRawAccel(accelTemp[i]) - MPU9250accelBias[i];
			gyroTemp[i] = convertRawGyro(gyroTemp[i]);
			magTemp[i] = convertRawMagnet(magTemp[i]);
		}

		imuDataUpdateFlag = true;

        int16_t gyroShake[3] = {0, };
        int16_t sumShake = 0;

        gyroShake[0] = abs(gyroTemp[0]);
        gyroShake[1] = abs(gyroTemp[1]);
        gyroShake[2] = abs(gyroTemp[2]);

        sumShake = gyroShake[0] + gyroShake[1] + gyroShake[2];
        if(sumShake > SHAKE_DET_MIN)countShake ++;
        numShake ++;
        if(numShake >= 20) // the detect timeout, now is 25 ms * 20 = 500 ms
        {
            if(countShake >= 17)
            {
                packet_01_data.data.deviceEvent = 8;// the times of shake motion more than threshold
            }
            else packet_01_data.data.deviceEvent = 0;
            numShake = 0;
            countShake = 0;
        }

        int16_t accelFree[3] = {0, };

        accelFree[0] = abs(accelTemp[0]);
        accelFree[1] = abs(accelTemp[1]);
        accelFree[2] = abs(accelTemp[2]);

		if(accelFree[0] < FREE_FALL_MAX && accelFree[0] > FREE_FALL_MIN)
		{
			if(accelFree[1] < FREE_FALL_MAX && accelFree[1] > FREE_FALL_MIN)
			{
				if(accelFree[2] < FREE_FALL_MAX && accelFree[2] > FREE_FALL_MIN)
				{
					packet_01_data.data.deviceEvent = 7;
				}
                else packet_01_data.data.deviceEvent = 0;
			}
		}

        if(packet_01_data.data.deviceEvent <= 6)
        {
            if(accelTemp[0] < TILT_X_P_MAX && accelTemp[0] > TILT_X_P_MIN)packet_01_data.data.deviceEvent = 1;
            else if(accelTemp[0] < TILT_X_N_MAX && accelTemp[0] > TILT_X_N_MIN)packet_01_data.data.deviceEvent = 2;
            else if(accelTemp[1] < TILT_Y_P_MAX && accelTemp[1] > TILT_Y_P_MIN)packet_01_data.data.deviceEvent = 3;
            else if(accelTemp[1] < TILT_Y_N_MAX && accelTemp[1] > TILT_Y_N_MIN)packet_01_data.data.deviceEvent = 4;
            else if(accelTemp[2] < TILT_Z_P_MAX && accelTemp[2] > TILT_Z_P_MIN)packet_01_data.data.deviceEvent = 5;
            else if(accelTemp[2] < TILT_Z_N_MAX && accelTemp[2] > TILT_Z_N_MIN)packet_01_data.data.deviceEvent = 6;
            else packet_01_data.data.deviceEvent = 0;
        }
#ifdef BLE_SUPPORT
			if(Core_mode == CORE_BLE_MODE)
			{
				if ((preEvent != packet_01_data.data.deviceEvent) && (packet_01_data.data.deviceEvent > 0))
				{
					InstructionOption.event.Header.type = I2C_CMD_NOTIFY_EVENT;
					InstructionOption.event.Header.Address = deviceI2CAddress;
					InstructionOption.event.Event  = packet_01_data.data.deviceEvent;
					InstructionOption.event.Header.Datalen	 = sizeof(packet_event);
					Wire.MasterGPIOTransmission(ptr2, sizeof(packet_event));
					preEvent = packet_01_data.data.deviceEvent;
				}
			}
#endif
	}

	if(ledFlashCommand)
	{
		uint32_t ledFlashCurrentMillis = millis();
		if(ledFlashCurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
		{
			ledFlashPreviousMillis = ledFlashCurrentMillis;
			digitalWrite(IMU_LED_PIN_NUM, ledFlashStatus);
			ledFlashStatus = !ledFlashStatus;
			if(ledFlashTimes < 3)
			{
				if (!ledFlashStatus)
				{
					ledFlashTimes++;
					if (ledFlashTimes >= 3)ledFlashCommand = false;
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

		if (LightRawData > 0xD0)
		{
			if (CurrentMillis - RawPreviousMillis >= RawDelayMillis)
			{
//				accel[i] = accelTemp[i];
//				gyro[i] = gyroTemp[i];
//				mag[i] = magTemp[i];
				InstructionOption.raw_data.Header.Address = deviceI2CAddress;
				InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);

				if ((LightRawData & 0xD1) == 0xD1)
				{
					InstructionOption.raw_data.Header.type = 0xD1;
					InstructionOption.raw_data.data[0] = accelTemp[0]&0xFF;
					InstructionOption.raw_data.data[1] = (accelTemp[0]>>8)&0xFF;
					InstructionOption.raw_data.data[2] = accelTemp[1]&0xFF;
					InstructionOption.raw_data.data[3] = (accelTemp[1]>>8)&0xFF;
					InstructionOption.raw_data.data[4] = accelTemp[2]&0xFF;
					InstructionOption.raw_data.data[5] = (accelTemp[2]>>8)&0xFF;
					// Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw));
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
					delay(1);
				}
				if ((LightRawData & 0xD2) == 0xD2)
				{
					InstructionOption.raw_data.Header.type = 0xD2;
					InstructionOption.raw_data.data[0] = gyroTemp[0]&0xFF;
					InstructionOption.raw_data.data[1] = (gyroTemp[0]>>8)&0xFF;
					InstructionOption.raw_data.data[2] = gyroTemp[1]&0xFF;
					InstructionOption.raw_data.data[3] = (gyroTemp[1]>>8)&0xFF;
					InstructionOption.raw_data.data[4] = gyroTemp[2]&0xFF;
					InstructionOption.raw_data.data[5] = (gyroTemp[2]>>8)&0xFF;
					// Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw));
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
					delay(1);
				}

				if ((LightRawData & 0xD4) == 0xD4)
				{
					InstructionOption.raw_data.Header.type = 0xD4;
//					Value = 180+atan2(magTemp[1], magTemp[0])*180/3.14159;
					InstructionOption.raw_data.data[0] = magTemp[0]&0xFF;
					InstructionOption.raw_data.data[1] = (magTemp[0]>>8)&0xFF;
					InstructionOption.raw_data.data[2] = magTemp[1]&0xFF;
					InstructionOption.raw_data.data[3] = (magTemp[1]>>8)&0xFF;
					InstructionOption.raw_data.data[4] = magTemp[2]&0xFF;
					InstructionOption.raw_data.data[5] = (magTemp[2]>>8)&0xFF;
					// Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw));
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
					delay(1);
				}
				if ((LightRawData & 0xD8) == 0xD8)
				{
					Value = atan2approx(magTemp[1], magTemp[0]);
					InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw)-4;
					
					InstructionOption.raw_data.Header.type = 0xD8;
					InstructionOption.raw_data.data[0] = Value&0xFF;
					InstructionOption.raw_data.data[1] = (Value>>8)&0xFF;
					if(Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw)-4) == 0)
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
	else if((commandReceive == I2C_CMD_SET_RANGE) || (commandReceive == I2C_CMD_SET_RATE))
	{
		commandReceive = I2C_CMD_NULL;
		Flash.write8(ACCEL_FSR_FLASH_LOC, accelFSR);
		Flash.write8(ACCEL_DLPF_FLASH_LOC, accelDLPF);
		Flash.write8(GYRO_FSR_FLASH_LOC, gyroFSR); ;
		Flash.write8(GYRO_DLPF_FLASH_LOC, gyroDLPF);
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
			digitalWrite(IMU_LED_PIN_NUM, HIGH);

			mpu9250PowerDown();
			SPI.end();

			pinMode(SCK, INPUT_PULLUP);
			pinMode(MOSI, INPUT_PULLUP);
			pinMode(MISO, INPUT_PULLUP);

			wwdg.end();
			Wire.end();
			pinMode(PA9, INPUT_PULLUP);
			pinMode(PA10, INPUT_PULLUP);

			nrgSave.standby();

			Wire.begin(deviceI2CAddress);
			Wire.onReceive(receiveEvent);
			Wire.onRequest(requestEvent);
			wwdg.begin();

			SPI.begin();
			mpu9250PowerUp();
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
#ifdef BLE_SUPPORT
	if ((commandReceive & 0xF0) == I2C_CMD_GET_RAW_DATA)
	{// Raw data request,

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
	}
#endif

	switch(commandReceive)
	{
		case I2C_CMD_SET_RANGE:
			if(receiveBuffer[1] == 0) // Accel range
			{
				accelFSR = receiveBuffer[2];
				mpu9250SetAccelRange((MPU9250_ACCEL_FSR)accelFSR);
			}

			else if(receiveBuffer[1] == 1) // Gyro range
			{
				gyroFSR = receiveBuffer[2];
				mpu9250SetGyroRange((MPU9250_GYRO_FSR)gyroFSR);
			}
		break;

		case I2C_CMD_SET_RATE:
			if(receiveBuffer[1] == 0) // Accel rate
			{
				accelDLPF = receiveBuffer[2];
				mpu9250SetAccelRate((MPU9250_ACCEL_DLPF)accelDLPF);
			}
			else if(receiveBuffer[1] == 1) // Gyro rate
			{
				gyroDLPF = receiveBuffer[2];
				mpu9250SetGyroRate((MPU9250_GYRO_DLPF)gyroDLPF);
			}
		break;

        case I2C_CMD_CALI_START:
			CalibrationMillis = millis();
			sample_count = 0;
            commandReceive = I2C_CMD_NULL;
        break;

		case I2C_CMD_LED_ON:
			ledFlashCommand = true;
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_LED_OFF:
			ledFlashCommand = false;
			ledFlashStatus = false;
			digitalWrite(IMU_LED_PIN_NUM, HIGH);
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

	if(commandReceive <= I2C_CMD_GET_ALL_X_Y_Z && commandReceive >= I2C_CMD_GET_ACCEL_X)
	{
		if(imuDataUpdateFlag)
		{
			// mpu9250Get9AxisRawData(accel, gyro, mag);
			for(uint8_t i = 0; i < 3; i ++)
			{
				// accel[i] = convertRawAccel(accel[i]);
				// gyro[i] = convertRawGyro(gyro[i]);
				// mag[i] = convertRawMagnet(mag[i]);
				accel[i] = accelTemp[i];
				gyro[i] = gyroTemp[i];
				mag[i] = magTemp[i];
			}
		}
	}

	switch(commandReceive)
	{
		case I2C_CMD_GET_DEV_ID:
			Wire.write(ptr1, 4);
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_GET_DEV_EVENT:
			Wire.write(ptr1 + 4, 4);
			commandReceive = I2C_CMD_NULL;
#ifdef BLE_SUPPORT
			preEvent = 0;
#endif
		break;

		case I2C_CMD_GET_ACCEL_X:
			Wire.write((uint8_t)(((uint16_t)accel[0]) & 0xff));
			Wire.write((uint8_t)(((uint16_t)accel[0]) >> 8));
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_GET_ACCEL_Y:
			Wire.write((uint8_t)(((uint16_t)accel[1]) & 0xff));
			Wire.write((uint8_t)(((uint16_t)accel[1]) >> 8));
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_GET_ACCEL_Z:
			Wire.write((uint8_t)(((uint16_t)accel[2]) & 0xff));
			Wire.write((uint8_t)(((uint16_t)accel[2]) >> 8));
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_GET_GYRO_X:
			Wire.write((uint8_t)(((uint16_t)gyro[0]) & 0xff));
			Wire.write((uint8_t)(((uint16_t)gyro[0]) >> 8));
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_GET_GYRO_Y:
			Wire.write((uint8_t)(((uint16_t)gyro[1]) & 0xff));
			Wire.write((uint8_t)(((uint16_t)gyro[1]) >> 8));
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_GET_GYRO_Z:
			Wire.write((uint8_t)(((uint16_t)gyro[2]) & 0xff));
			Wire.write((uint8_t)(((uint16_t)gyro[2]) >> 8));
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_GET_MAG_X:
			Wire.write((uint8_t)(((uint16_t)mag[0]) & 0xff));
			Wire.write((uint8_t)(((uint16_t)mag[0]) >> 8));
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_GET_MAG_Y:
			Wire.write((uint8_t)(((uint16_t)mag[1]) & 0xff));
			Wire.write((uint8_t)(((uint16_t)mag[1]) >> 8));
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_GET_MAG_Z:
			Wire.write((uint8_t)(((uint16_t)mag[2]) & 0xff));
			Wire.write((uint8_t)(((uint16_t)mag[2]) >> 8));
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_GET_ACCEL_X_Y_Z:
			{
				for(uint8_t i = 0; i < 3; i ++)
				{
					Wire.write((uint8_t)(((uint16_t)accel[i]) & 0xff));
					Wire.write((uint8_t)(((uint16_t)accel[i]) >> 8));
				}
				commandReceive = I2C_CMD_NULL;
			}
		break;

		case I2C_CMD_GET_GYRO_X_Y_Z:
			{
				for(uint8_t i = 0; i < 3; i ++)
				{
					Wire.write((uint8_t)(((uint16_t)gyro[i]) & 0xff));
					Wire.write((uint8_t)(((uint16_t)gyro[i]) >> 8));
				}
				commandReceive = I2C_CMD_NULL;
			}
		break;

		case I2C_CMD_GET_MAG_X_Y_Z:
			{
				for(uint8_t i = 0; i < 3; i ++)
				{
					Wire.write((uint8_t)(((uint16_t)mag[i]) & 0xff));
					Wire.write((uint8_t)(((uint16_t)mag[i]) >> 8));
				}
				commandReceive = I2C_CMD_NULL;
			}
		break;

		case I2C_CMD_GET_ALL_X_Y_Z:
			{
				for(uint8_t i = 0; i < 3; i ++)
				{
					Wire.write((uint8_t)(((uint16_t)accel[i]) & 0xff));
					Wire.write((uint8_t)(((uint16_t)accel[i]) >> 8));
				}
				for(uint8_t i = 0; i < 3; i ++)
				{
					Wire.write((uint8_t)(((uint16_t)gyro[i]) & 0xff));
					Wire.write((uint8_t)(((uint16_t)gyro[i]) >> 8));
				}
				for(uint8_t i = 0; i < 3; i ++)
				{
					Wire.write((uint8_t)(((uint16_t)mag[i]) & 0xff));
					Wire.write((uint8_t)(((uint16_t)mag[i]) >> 8));
				}
				commandReceive = I2C_CMD_NULL;
			}
		break;

		case I2C_CMD_GET_HEADING:
			heading = atan2approx(magTemp[1], magTemp[0]);
			Wire.write((uint8_t)(heading & 0xff));
			Wire.write((uint8_t)(heading >> 8));
			commandReceive = I2C_CMD_NULL;
		break;

		case I2C_CMD_GET_ROTATION:

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

		default:
		break;
	}
}

/***************************************************************

 ***************************************************************/
uint8_t mpu9250SendByte(uint8_t data)
{
	uint8_t result = 0;
	result = SPI.transfer(data);
	return result;
}

uint8_t mpu9250Read(uint8_t regAddr)
{
	uint8_t dummy = 0;
	uint8_t data = 0;

	chipEnable();
	mpu9250SendByte(0x80 | regAddr);
	data = mpu9250SendByte(dummy);
	chipDisable();

	return data;
}

void mpu9250Reads(uint8_t regAddr, uint8_t len, uint8_t *data)
{
	uint16_t i = 0;
	uint8_t dummy = 0;

	chipEnable();
	mpu9250SendByte(MPU9250_I2C_READ | regAddr);
	while(i < len)data[i++] = mpu9250SendByte(dummy);
	chipDisable();
}

void mpu9250Write(uint8_t regAddr, uint8_t data)
{
	chipEnable();
	mpu9250SendByte(regAddr);
	mpu9250SendByte(data);
	chipDisable();
}

void mpu9250Writes(uint8_t regAddr, uint8_t len, uint8_t *data)
{
	uint16_t i = 0;

	chipEnable();
	mpu9250SendByte(regAddr);
	while(i < len)mpu9250SendByte(data[i++]);
	chipDisable();
}

int16_t ak8963Reads(uint8_t akmAddr, uint8_t regAddr, uint8_t len, uint8_t *data)
{
	uint8_t index = 0;
	uint8_t status = 0;
	uint16_t timeout = 0;
	uint8_t temp = 0;

	temp = akmAddr | MPU9250_I2C_READ;
	mpu9250Writes(MPU9250_I2C_SLV4_ADDR, 1, &temp);
	delay(1);
	while(index < len)
	{
		temp = regAddr + index;
		mpu9250Writes(MPU9250_I2C_SLV4_REG, 1, &temp);
		delay(1);
		temp = MPU9250_I2C_SLV4_EN;
		mpu9250Writes(MPU9250_I2C_SLV4_CTRL, 1, &temp);
		delay(1);
		do
		{
			if(timeout ++ > 50)return (-2);
			mpu9250Reads(MPU9250_I2C_MST_STATUS, 1, &status);
			delay(2);
		}while((status & MPU9250_I2C_SLV4_DONE) == 0);

		mpu9250Reads(MPU9250_I2C_SLV4_DI, 1, data + index);
		delay(1);
		index++;
	}
	return 0;
}

int16_t ak8963Write(uint8_t akmAddr, uint8_t regAddr, uint8_t data)
{
	uint16_t timeout = 0;
	uint8_t status = 0;
	uint8_t temp = 0;

	temp = akmAddr;
	mpu9250Writes(MPU9250_I2C_SLV4_ADDR, 1, &temp);
	delay(1);
	temp = regAddr;
	mpu9250Writes(MPU9250_I2C_SLV4_REG, 1, &temp);
	delay(1);
	temp = data;
	mpu9250Writes(MPU9250_I2C_SLV4_DO, 1, &temp);
	delay(1);
	temp = MPU9250_I2C_SLV4_EN;
	mpu9250Writes(MPU9250_I2C_SLV4_CTRL, 1, &temp);
	delay(1);

	do
	{
		if(timeout++ > 50)	return (-2);
		mpu9250Reads(MPU9250_I2C_MST_STATUS, 1, &status);
		delay(1);
	}while((status & MPU9250_I2C_SLV4_DONE) == 0);

	if(status & MPU9250_I2C_SLV4_NACK)return (-3);

	return 0;
}

void mpu9250Init(void)
{
	uint8_t state = 0;
	uint8_t response[3] = {0, 0, 0};

	accelgyrocalMPU9250(MPU9250gyroBias, MPU9250accelBias);

	//MPU9250 Reset
	mpu9250Write(MPU9250_PWR_MGMT_1, MPU9250_RESET);
	delay(100);

	//MPU9250 Set Clock Source
	mpu9250Write(MPU9250_PWR_MGMT_1, MPU9250_CLOCK_PLLGYROZ);
	delay(1);

	//MPU9250 Set Interrupt
	mpu9250Write(MPU9250_INT_PIN_CFG, MPU9250_INT_ANYRD_2CLEAR | 0x80 | 0x40);
	delay(1);
	// mpu9250Write(MPU9250_INT_ENABLE, 0x01);
	// delay(1);

	//MPU9250 Set Sensors
	mpu9250Write(MPU9250_PWR_MGMT_2, MPU9250_XYZ_GYRO & MPU9250_XYZ_ACCEL);
	delay(1);

	//MPU9250 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	mpu9250Write(MPU9250_SMPLRT_DIV, SMPLRT_DIV);
	delay(1);

	mpu9250SetAccelRange((MPU9250_ACCEL_FSR)accelFSR);
	delay(1);
	mpu9250SetAccelRate((MPU9250_ACCEL_DLPF)accelDLPF);
	delay(1);
	mpu9250SetGyroRange((MPU9250_GYRO_FSR)gyroFSR);
	delay(1);
	mpu9250SetGyroRate((MPU9250_GYRO_DLPF)gyroDLPF);
	delay(1);

	//MPU9250 Set SPI Mode
	state = mpu9250Read(MPU9250_USER_CTRL);
	delay(1);
	mpu9250Write(MPU9250_USER_CTRL, state | MPU9250_I2C_IF_DIS);
	delay(1);
	state = mpu9250Read(MPU9250_USER_CTRL);
	delay(1);
	mpu9250Write(MPU9250_USER_CTRL, state | MPU9250_I2C_MST_EN);
	delay(1);

	//reset AK8963
	ak8963Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);
	delay(5);
	ak8963Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	delay(1);
	ak8963Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
	delay(1);

	//AK8963 get calibration data
	ak8963Reads(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
	//AK8963_SENSITIVITY_SCALE_FACTOR
	mpu9250ak8963asa[0] = (uint16_t)(response[0]) + 128;
	mpu9250ak8963asa[1] = (uint16_t)(response[1]) + 128;
	mpu9250ak8963asa[2] = (uint16_t)(response[2]) + 128;
	delay(1);
	ak8963Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	delay(1);
	mpu9250Write(MPU9250_I2C_MST_CTRL, 0x5D);
	delay(1);
	mpu9250Write(MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
	delay(1);
	mpu9250Write(MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
	delay(1);
	mpu9250Write(MPU9250_I2C_SLV0_CTRL, 0x88);
	delay(1);
	ak8963Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
	delay(1);
	mpu9250Write(MPU9250_I2C_SLV4_CTRL, 0x09);
	delay(1);
	mpu9250Write(MPU9250_I2C_MST_DELAY_CTRL, 0x81);
	delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelgyrocalMPU9250(int32_t * dest1, int32_t * dest2)
{
#if 0
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

 // reset device
  mpu9250Write(MPU9250_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
 // else use the internal oscillator, bits 2:0 = 001
  mpu9250Write(MPU9250_PWR_MGMT_1, 0x01);
  mpu9250Write(MPU9250_PWR_MGMT_2, 0x00);
  delay(200);

// Configure device for bias calculation
  mpu9250Write(MPU9250_INT_ENABLE, 0x00);   // Disable all interrupts
  mpu9250Write(MPU9250_FIFO_EN, 0x00);      // Disable FIFO
  mpu9250Write(MPU9250_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  mpu9250Write(MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
  mpu9250Write(MPU9250_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  mpu9250Write(MPU9250_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

// Configure MPU6050 gyro and accelerometer for bias calculation
  mpu9250Write(MPU9250_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  mpu9250Write(MPU9250_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  mpu9250Write(MPU9250_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  mpu9250Write(MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  mpu9250Write(MPU9250_USER_CTRL, 0x40);   // Enable FIFO
  mpu9250Write(MPU9250_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  mpu9250Write(MPU9250_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  mpu9250Reads(MPU9250_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int32_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    mpu9250Reads(MPU9250_FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int32_t) (((int32_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int32_t) (((int32_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int32_t) (((int32_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int32_t) (((int32_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int32_t) (((int32_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int32_t) (((int32_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
    accel_bias[0] /= ((int32_t) packet_count); // Normalize sums to get average count biases
    accel_bias[1] /= ((int32_t) packet_count);
    accel_bias[2] /= ((int32_t) packet_count);
    gyro_bias[0]  /= ((int32_t) packet_count);
    gyro_bias[1]  /= ((int32_t) packet_count);
    gyro_bias[2]  /= ((int32_t) packet_count);

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  mpu9250Write(MPU9250_XG_OFFSET_H, data[0]);
  mpu9250Write(MPU9250_XG_OFFSET_L, data[1]);
  mpu9250Write(MPU9250_YG_OFFSET_H, data[2]);
  mpu9250Write(MPU9250_YG_OFFSET_L, data[3]);
  mpu9250Write(MPU9250_ZG_OFFSET_H, data[4]);
  mpu9250Write(MPU9250_ZG_OFFSET_L, data[5]);

// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

// Output scaled accelerometer biases for display in the main program
   dest2[0] = accel_bias[0]/16;
   dest2[1] = accel_bias[1]/16;
   dest2[2] = accel_bias[2]/16;
#endif

}

void sampleDummy(void)
{
	mpu9250IsNewData = true;
}

bool mpu9250IsDataReady(void)
{
	bool isNewData = mpu9250IsNewData;
	mpu9250IsNewData = false;
	return isNewData;
}

void mpu9250Get9AxisRawData(int16_t *accel, int16_t *gyro, int16_t *mag)
{
	uint8_t data[22];

	mpu9250Reads(MPU9250_ACCEL_XOUT_H, 22, data);

	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];

	gyro[0] = (data[8] << 8) | data[9];
	gyro[1] = (data[10] << 8) | data[11];
	gyro[2] = (data[12] << 8) | data[13];

	// if(!(data[14] & MPU9250_AK8963_DATA_READY) || (data[14] & MPU9250_AK8963_DATA_OVERRUN))return;
	// if(data[21] & MPU9250_AK8963_OVERFLOW)return;

	mag[0] = (data[16] << 8) | data[15];
	mag[1] = (data[18] << 8) | data[17];
	mag[2] = (data[20] << 8) | data[19];

	mag[0] = ((mag[0] * mpu9250ak8963asa[0]) >> 8) - MPU9250magBias[0];
	mag[1] = ((mag[1] * mpu9250ak8963asa[1]) >> 8) - MPU9250magBias[1];
	mag[2] = ((mag[2] * mpu9250ak8963asa[2]) >> 8) - MPU9250magBias[2];
}

void mpu9250Get6AxisRawData(int16_t *accel, int16_t*gyro)
{
	uint8_t data[14];

	mpu9250Reads(MPU9250_ACCEL_XOUT_H, 14, data);

	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];

	gyro[0] = (data[8] << 8) | data[9];
	gyro[1] = (data[10] << 8) | data[11];
	gyro[2] = (data[12] << 8) | data[13];
}

void mpu9250Get3AxisAccelRawData(int16_t *accel)
{
	uint8_t data[6];

	mpu9250Reads(MPU9250_ACCEL_XOUT_H, 6, data);

	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];
}

void mpu9250Get3AxisGyroRawData(int16_t *gyro)
{
	uint8_t data[6];

	mpu9250Reads(MPU9250_GYRO_XOUT_H, 6, data);

	gyro[0] = (data[0] << 8) | data[1];
	gyro[1] = (data[2] << 8) | data[3];
	gyro[2] = (data[4] << 8) | data[5];
}

void mpu9250Get3AxisMagnetRawData(int16_t *mag)
{
	uint8_t data[8];

	mpu9250Reads(MPU9250_EXT_SENS_DATA_00, 8, data);

	// if(!(data[0] & MPU9250_AK8963_DATA_READY) || (data[0] & MPU9250_AK8963_DATA_OVERRUN))return;
	// if(data[7] & MPU9250_AK8963_OVERFLOW)return;

	mag[0] = (data[2] << 8) | data[1];
	mag[1] = (data[4] << 8) | data[3];
	mag[2] = (data[6] << 8) | data[5];

}

void mpu9250GetTemperatureRawData(int16_t *temperature)
{
	uint8_t data[2];

	mpu9250Reads(MPU9250_TEMP_OUT_H, 2, data);

	temperature[0] = (data[0] << 8) | data[1];
}

void mpu9250SetAccelRange(MPU9250_ACCEL_FSR range)
{
	uint8_t data;

	data = mpu9250Read(MPU9250_ACCEL_CONFIG);
	data = data & 0xe7;
	data = data | (range << 3);
	mpu9250Write(MPU9250_ACCEL_CONFIG, data);
}

void mpu9250SetAccelRate(MPU9250_ACCEL_DLPF rate)
{
	uint8_t data;

	data = mpu9250Read(MPU9250_ACCEL_CONFIG2);
	data = data & 0xf8;
	data = data | rate;
	mpu9250Write(MPU9250_ACCEL_CONFIG2, data);
}

void mpu9250SetGyroRange(MPU9250_GYRO_FSR range)
{
	uint8_t data;

	data = mpu9250Read(MPU9250_GYRO_CONFIG);
	data = data & 0xe7;
	data = data | (range << 3);
	mpu9250Write(MPU9250_GYRO_CONFIG, data);
}

void mpu9250SetGyroRate(MPU9250_GYRO_DLPF rate)
{
	uint8_t data;

	data = mpu9250Read(MPU9250_CONFIG);
	data = data & 0xf8;
	data = data | rate;
	mpu9250Write(MPU9250_CONFIG, data);
}

void mpu9250PowerDown(void)
{
	uint8_t data;

	ak8963Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);

	data = mpu9250Read(MPU9250_PWR_MGMT_1);
	data = data | 0x40;
	mpu9250Write(MPU9250_PWR_MGMT_1, data);
}

void mpu9250PowerUp(void)
{
	uint8_t data;

	data = mpu9250Read(MPU9250_PWR_MGMT_1);
	data = data & 0xbf;
	mpu9250Write(MPU9250_PWR_MGMT_1, data);

	ak8963Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
}

int16_t convertRawAccel(int16_t aRaw)
{
	float a;

#if 1
	switch(accelFSR)
	{
		case MPU9250_FSR_2G:
			a = aRaw * 2000 / 32768;
		break;

		case MPU9250_FSR_4G:
			a = aRaw * 4000 / 32768;
		break;

		case MPU9250_FSR_8G:
			a = aRaw * 8000 / 32768;
		break;

		case MPU9250_FSR_16G:
			a = aRaw * 16000 / 32768;
		break;

		default:
		break;
	}
#else
	a = aRaw * 25 * (1<<accelFSR);
	a /= 2048;
#endif

	return (int16_t)a;
}

int16_t convertRawGyro(int16_t gRaw)
{
	float g;

	switch(gyroFSR)
	{
		case MPU9250_FSR_250DPS:
			g = gRaw * 250 / 32768;
		break;

		case MPU9250_FSR_500DPS:
			g = gRaw * 500 / 32768;
		break;

		case MPU9250_FSR_1000DPS:
			g = gRaw * 1000 / 32768;
		break;

		case MPU9250_FSR_2000DPS:
			g = gRaw * 2000 / 32768;
		break;

		default:
		break;
	}

	return (int16_t)g;
}

int16_t convertRawMagnet(int16_t mRaw)
{
	float m = mRaw * MPU9250_MAGNET_RANGE / 32768;

    return (int16_t)m;
}


uint16_t atan2approx(int16_t y, int16_t x)
{
  int16_t absx, absy;
  int32_t val, resuilt;
  absy = abs(y);
  absx = abs(x);
  int8_t octant = ((x<0)?(1 << 2):0) + ((y<0)?(1 << 1 ):0) + ((absx <= absy)?1:0);
  
  switch (octant) {
    case 0: {
        if (x == 0 && y == 0)
          return 0;
        val = absy*100/absx;
        resuilt = (10584 - 27*val)*val; //1st octant
        break;
      }
    case 1:{
        if (x == 0 && y == 0)
          return 0.0;
        val = absx*100/absy;
        resuilt = 1570796 - (10584 - 27*val)*val; //2nd octant
        break;
      }
    case 2: {
        val =absy*100/absx;
        resuilt = -(10584 - 27*val)*val; //8th octant
        break;
      }
    case 3: {
        val =absx*100/absy;
        resuilt = -1570796 + (10584 - 27*val)*val;//7th octant
        break;
      }
    case 4: {
        val =absy*100/absx;
        resuilt =  3141593 - (10584 - 27*val)*val;  //4th octant
      }
    case 5: {
        val =absx*100/absy;
        resuilt =  1570796 + (10584 - 27*val)*val;//3rd octant
        break;
      }
    case 6: {
        val =absy*100/absx;
        resuilt = -3141593+ (10584 - 27*val)*val; //5th octant
        break;
      }
    case 7: {
        val =absx*100/absy;
        resuilt = -1570796 - (10584 - 27*val)*val; //6th octant
        break;
      }
    default:
      return 0.0;
    }
	resuilt *= 57;
	resuilt += 180000000;
	resuilt /= 1000000;
	
	return (uint16_t)resuilt;
}
