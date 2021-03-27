
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>

#define GROVE_LED_PIN_NUM       PA1

/***************************************************************

 ***************************************************************/
#define DEVICE_I2C_ADDRESS      0x26
#define DEVICE_VID              0x2886
#define DEVICE_PID              0x8026

#define I2C_DEF_ADDR_FLASH_LOC      0x00 // byte address
#define I2C_CUR_ADDR_FLASH_LOC      0x01 // byte address

#define I2C_CMD_GET_DEV_ID          0x00 //
#define I2C_CMD_NEXT_ONE            0x01
#define I2C_CMD_PREVIOUS_ONE        0x02
#define I2C_CMD_FIXED_ONE           0x03
#define I2C_CMD_VOLUME_CHANGE       0x04
#define I2C_CMD_VOLUME_DECREASE     0x05
#define I2C_CMD_VOLUME_VALUE        0x06
#define I2C_CMD_MUSIC_STYLE         0x07
#define I2C_CMD_FIXED_LOOP          0x08
#define I2C_CMD_DEVICE_SELECT       0x09
#define I2C_CMD_ENTER_SLEEP         0x0A

#define I2C_CMD_CHIP_RESET          0x0C
#define I2C_CMD_START               0x0D
#define I2C_CMD_PAUSE               0x0E
#define I2C_CMD_DIRECT              0x0F

#define I2C_CMD_LOOP_ALL            0x11
#define I2C_CMD_DIRECT_MUSIC        0x12

#define I2C_CMD_BIG_DIRECT          0x14

#define I2C_CMD_STOP_BACKGROUND     0x15
#define I2C_CMD_STOP                0x16
#define I2C_CMD_LOOP_DIRECT         0x17
#define I2C_CMD_RANDOM              0x18
#define I2C_CMD_LOOP_CURRENT        0x19
#define I2C_CMD_OPEN_CLOSE_DAC      0x1A

#define I2C_CMD_PLAY_IN_ORDER       0x1B

#define I2C_CMD_VOLUME_GET          0x43
#define I2C_CMD_GET_MP3_NUMBER      0x48
#define I2C_CMD_GET_CURRENT         0x4C

#define I2C_CMD_LED_ON          0xb0 //
#define I2C_CMD_LED_OFF         0xb1 //
#define I2C_CMD_AUTO_SLEEP_ON   0xb2 //
#define I2C_CMD_AUTO_SLEEP_OFF  0xb3 //
#define I2C_CMD_SET_ADDR        0xc0 //
#define I2C_CMD_RST_ADDR        0xc1 //
#define I2C_CMD_TEST_TX_RX_ON   0xe0 //
#define I2C_CMD_TEST_TX_RX_OFF  0xe1 //
#define I2C_CMD_TEST_GET_VER    0xe2 //
#define I2C_CMD_JUMP_TO_BOOT    0xf0 //
#define I2C_CMD_GET_DEVICE_UID  0xf1 //
#define I2C_CMD_NULL            0xff //

#define I2C_CMD_GET_EVENT       0x20

#define I2C_CMD_ATMEL_ONE       0x93
#define I2C_CMD_ATMEL_NEXT      0x91

#define MP3_MAX_VOLUME          30
#define APP_MAX_VOLUME          100
#define COMMAND_MILLIS_DELAY    30

#define DEVICE_UDISK    1
#define DEVICE_SD       2
#define DEVICE_FLASH    5

#define KT403_START_BYTE        0x7E
#define KT403_VERSION_BYTE      0xFF
#define KT403_END_BYTE          0xEF

uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS, MaxMusic = 0;
uint8_t commandReceive = I2C_CMD_NULL;
uint8_t ledFlashTimes = 0;

#ifdef BLE_SUPPORT

#define I2C_CMD_LOW_PRIORITY    0x90

uint8_t Core_mode = 0, DeviceAvailable = 0;
uint32_t StartMillis = 0, PreMillis = 0;
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
    uint8_t     parameters[3];
}packet_cmd_t;

typedef struct
{
    uint8_t     pid[2];
    uint8_t     chipid;
    uint8_t     Newaddress;
    uint8_t     option[5];
}packet_got_atr;

typedef struct
{
    packet_header_t Header;
    union
    {
        packet_cmd_t    cmd;
        packet_got_atr  atr;
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
    packet_header_t Header;
    uint8_t     pid[2];
    uint8_t     chipid;
    uint8_t     hwaddress;
    uint8_t     version[3];
    uint8_t     option[2];
}packet_atr;

typedef struct
{
    packet_header_t Header;
    uint8_t         data[4];
}packet_raw;

typedef struct
{
    packet_header_t Header;
    uint8_t         Event;
}packet_event;

union
{
    packet_atr      atr;
    packet_raw      raw_data;
    packet_event    event;
    uint8_t         bytes[MAINBOARD_BLE_I2C_DATALEN];
}InstructionOption;

uint8_t LightRawData = 1, ErrorCount = 0, ReportRawData = 0;
uint32_t RawDelayMillis = 5000, CommandMillis = 0;
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
uint8_t KT403_feedback[16], GetCmd = 0, PutCmd = 0;

#define K403A_RANDOM_PLAY       1
#define K403A_ORDER_PLAY        2
#define K403A_LOOPONE_PLAY      3
#define K403A_STOP_PLAY         4

int32_t CurrentMusic = -1, PreStatus = 0, CurrentStatus = K403A_ORDER_PLAY;

#define K403A_START_BYTE        0x7E
#define K403A_VERSION_BYTE      0xFF
#define K403A_STOP_BYTE         0xEF
#define K403A_COMMAND_NUM       16
#define K403A_LENGTH_BYTE       6

uint8_t cmdData[K403A_COMMAND_NUM][10];

void SendCMD(uint8_t cmd, uint16_t para);

void requestEvent(void);
void receiveEvent(int howMany);

/***************************************************************

 ***************************************************************/
LowPower nrgSave;

#define AUTO_SLEEP_TIMEOUT  2000

uint32_t autoSleepPreviousMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

/***************************************************************

 ***************************************************************/
#define LED_FLASH_TIME  250

bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;

/***************************************************************

 ***************************************************************/

#define SOUND_SAMPPLE_TIME  5000

uint32_t samplePreviousMillis = 0;

char *versions = "V21";

uint16_t NodeVersion = 0x6110;

uint8_t fbnum = 0;

uint8_t VolumeValue = APP_MAX_VOLUME;

/***************************************************************

 ***************************************************************/
uint8_t chipId[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/***************************************************************

 ***************************************************************/
bool flashSave = false;

/***************************************************************

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

    pinMode(GROVE_LED_PIN_NUM, OUTPUT);
    digitalWrite(GROVE_LED_PIN_NUM, LOW);


    Wire.begin(deviceI2CAddress);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    
    Serial.begin(9600);
    
    delay(2000);

    wwdg.begin();

//  SendCMD(I2C_CMD_DEVICE_SELECT, DEVICE_SD); // Select device SD card.
#ifdef BLE_SUPPORT
    StartMillis = millis();
#endif
}

void loop()
{
    uint32_t CurrentMillis = millis();
            
#ifdef BLE_SUPPORT
    if(Wire.isbusidle())PreMillis = CurrentMillis;
    if ((CurrentMillis - PreMillis) > 20)
    {
        Wire.reset();
        Wire.begin(deviceI2CAddress);
        Wire.onReceive(receiveEvent);
        Wire.onRequest(requestEvent);
        PreMillis = CurrentMillis;
    }
#endif

    if (((CurrentMillis - CommandMillis) >= COMMAND_MILLIS_DELAY) && (PutCmd != GetCmd) && (DeviceAvailable >0))
    {
        SendCmd();
    }
        
    if(Serial.available())
    {
        KT403_feedback[fbnum++] = Serial.read();

        if((fbnum >= 10) && (KT403_feedback[2] == 6) && (KT403_feedback[1] == KT403_VERSION_BYTE) && 
                (KT403_feedback[9] == KT403_END_BYTE) && (KT403_feedback[0] == KT403_START_BYTE))
        {

            packet_01_data.data.deviceEvent = (KT403_feedback[3]<<16)+KT403_feedback[5]*256+KT403_feedback[6];
            fbnum = 0;
            
            if (KT403_feedback[3] == 0x3F)
            {// SD card is available now.
                DeviceAvailable = KT403_feedback[6];
            }
#if 1
            switch(KT403_feedback[3])
            {// CMD
                case 0x4C:
                    // Current mp3 in TF card.
                //  packet_01_data.data.deviceEvent = KT403_feedback[6];
                    if(CurrentStatus == K403A_STOP_PLAY);
                    CurrentMusic = KT403_feedback[5]*256+KT403_feedback[6];
                    break;
                case 0x48:
                    // Total mp3 in TF card.
                //  packet_01_data.data.deviceEvent = KT403_feedback[6];
                    MaxMusic = KT403_feedback[5]*256+KT403_feedback[6];

                    break;
                case 0x3F:
                    // KT403 power on, return available device.
                    packet_01_data.data.deviceEvent = KT403_feedback[6];
                    if (KT403_feedback[6] == 2)
                    {
                        SendCMD(I2C_CMD_GET_MP3_NUMBER, 0);
                    }
                    break;
                case 0x3D:
                    // xx is play finished in device TF.
                    // packet_01_data.data.deviceEvent = KT403_feedback[5]*256+KT403_feedback[6];
                    if (CurrentStatus == K403A_RANDOM_PLAY)
                    {
                        SendCMD(I2C_CMD_RANDOM, 0);
                        SendCMD(I2C_CMD_NEXT_ONE, 0);
                    }else if (CurrentStatus == K403A_ORDER_PLAY)
                    {
                        SendCMD(I2C_CMD_NEXT_ONE, 0);
                        SendCMD(I2C_CMD_GET_CURRENT, 0);
                    }else if (CurrentStatus == K403A_LOOPONE_PLAY)
                    {
                        SendCMD(I2C_CMD_FIXED_ONE, KT403_feedback[5]*256+KT403_feedback[6]);
                    }
                    
                    break;
                case 0x3A:
                    // KT403 device plug in.
                    // 01 U-Disk.
                    // 02 TF card.
                    // 04 Flash.
                    packet_01_data.data.deviceEvent = KT403_feedback[6];
                    
                    break;
                case 0x3B:
                    // KT403 device Unplug in.
                    // 01 U-Disk.
                    // 02 TF card.
                    // 04 Flash.
                    packet_01_data.data.deviceEvent = KT403_feedback[6];
                    
                    break;
                case 0x40:
                    // KT403 feedback error.
                    // 01 KT403 is in file system initial.
                    // 02 KT403 is in sleep mode.
                    // 03 KT403 lose one frame data in UART.
                    // 04 KT403 CRC error.
                    // 05 KT403 file number is out of range.
                    // 06 KT403 file was not found.
                    // 07 KT403 can't insert in current status.
                    packet_01_data.data.deviceEvent = KT403_feedback[6];
                    
                    break;
                default:
                    break;
            }
#endif
        }

    }
    
#ifdef BLE_SUPPORT

    if (Core_mode == 0)
    {// ATR
        if(CurrentMillis - StartMillis >= (I2C_CMD_SYS_READY_TIME))
        {
#if 1
            Core_mode = CORE_BLE_MODE;
//          SendCMD(I2C_CMD_LOOP_ALL, 0x01); 
#else
            commandReceive = I2C_CMD_NOTIFY_ATR;
            StartMillis = CurrentMillis;
#endif
        }

    }
    if(Core_mode == CORE_BLE_MODE)
    {
        if ((ReportRawData) && (packet_01_data.data.deviceEvent > 0))
        {
            InstructionOption.raw_data.Header.type = I2C_CMD_GET_RAW_DATA;
            InstructionOption.raw_data.Header.Address = deviceI2CAddress;
            memcpy(InstructionOption.raw_data.data, ptr1+4, 4);
            InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);
            Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw));
            packet_01_data.data.deviceEvent = 0;
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
            case I2C_CMD_VOLUME_GET:
                InstructionOption.raw_data.Header.type = I2C_CMD_GET_RAW_DATA;
                InstructionOption.raw_data.Header.Address = deviceI2CAddress;
                InstructionOption.raw_data.data[0] = VolumeValue;
                InstructionOption.raw_data.data[1] = 0;
                InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);
                Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw));
                commandReceive = I2C_CMD_NULL;
                break;
            case I2C_CMD_SET_ADDR:
                commandReceive = I2C_CMD_NULL;
                Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
                Wire.begin(deviceI2CAddress);
                break;
            case I2C_CMD_RST_ADDR:
                commandReceive = I2C_CMD_NULL;
                deviceI2CAddress = Flash.read8(I2C_DEF_ADDR_FLASH_LOC);
                Flash.write8(I2C_CUR_ADDR_FLASH_LOC, deviceI2CAddress);
                Wire.begin(deviceI2CAddress);
                break;

            default:
                break;
        }
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

    wwdg.reset();
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
        if((commandReceive >= I2C_CMD_LOW_PRIORITY) && (commandReceive < (I2C_CMD_LOW_PRIORITY+0x20)))
        {
            commandReceive -= I2C_CMD_LOW_PRIORITY;
            commandOption.data.Header.type -= I2C_CMD_LOW_PRIORITY;
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
            ReportRawData = receiveBuffer[4];
            commandReceive = I2C_CMD_NULL;

            break;
#endif
        case I2C_CMD_ATMEL_ONE:
            CurrentMusic = receiveBuffer[2]*256+receiveBuffer[1];
            SendCMD(I2C_CMD_FIXED_ONE, receiveBuffer[2]*256+receiveBuffer[1]);
            commandReceive = I2C_CMD_NULL;
            break;
        case I2C_CMD_ATMEL_NEXT:
            if (CurrentStatus == K403A_RANDOM_PLAY)
            {
                SendCMD(I2C_CMD_RANDOM, 0);
                SendCMD(I2C_CMD_NEXT_ONE, 0);
            }else
            {
                if (CurrentStatus == K403A_ORDER_PLAY)
                {
                    CurrentMusic++;
                }
                if (CurrentMusic > MaxMusic)
                {
                    CurrentMusic = 1;
                }
                if (CurrentMusic < 1)
                {
                    CurrentMusic = 1;
                }
                commandReceive = I2C_CMD_NULL;
                SendCMD(I2C_CMD_FIXED_ONE, CurrentMusic);
            }
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

        case I2C_CMD_JUMP_TO_BOOT:
            commandReceive = I2C_CMD_NULL;
            jumpToBootloader();
        break;

        default:
        break;
    }
    switch(commandReceive)
    {

        case I2C_CMD_NEXT_ONE:
            if (PreStatus > 0)
            {
                CurrentStatus  = PreStatus;
                PreStatus = 0;
            }
            if (CurrentStatus == K403A_RANDOM_PLAY)
            {
                SendCMD(I2C_CMD_RANDOM, 0);
                SendCMD(I2C_CMD_NEXT_ONE, 0);
            }else
            {
                if (CurrentStatus == K403A_ORDER_PLAY)
                {
                    CurrentMusic++;
                }
                if (CurrentMusic > MaxMusic)
                {
                    CurrentMusic = 1;
                }
                if (CurrentMusic < 1)
                {
                    CurrentMusic = 1;
                }
                SendCMD(I2C_CMD_FIXED_ONE, CurrentMusic);
            }
            commandReceive = I2C_CMD_NULL;
            break;
        case I2C_CMD_PREVIOUS_ONE:
            if (PreStatus > 0)
            {
                CurrentStatus  = PreStatus;
                PreStatus = 0;
            }

            if (CurrentStatus == K403A_RANDOM_PLAY)
            {
                SendCMD(I2C_CMD_RANDOM, 0);
                SendCMD(I2C_CMD_NEXT_ONE, 0);
            }else
            {
                if (CurrentStatus == K403A_ORDER_PLAY)
                {
                    if (CurrentMusic > 1)CurrentMusic--;
                    else CurrentMusic = MaxMusic;
                }
                if (CurrentMusic < 1)
                {
                    CurrentMusic = 1;
                }
                SendCMD(I2C_CMD_FIXED_ONE, CurrentMusic);
            }
            commandReceive = I2C_CMD_NULL;
            break;
            
        case I2C_CMD_PLAY_IN_ORDER:
        case I2C_CMD_LOOP_ALL:
            commandReceive = I2C_CMD_NULL;
            CurrentStatus  = K403A_ORDER_PLAY;
            PreStatus = 0;
            break;

        case I2C_CMD_RANDOM:
            commandReceive = I2C_CMD_NULL;
            CurrentStatus  = K403A_RANDOM_PLAY;
            PreStatus = 0;
            break;
        case I2C_CMD_STOP:
            commandReceive = I2C_CMD_NULL;
            PreStatus  = CurrentStatus;
            CurrentStatus  = K403A_STOP_PLAY;
            SendCMD(I2C_CMD_GET_CURRENT, 0);
            SendCMD(I2C_CMD_STOP, 0);
            break;

        case I2C_CMD_START:
            if(CurrentStatus == K403A_STOP_PLAY)
            {
                SendCMD(I2C_CMD_FIXED_ONE, CurrentMusic);
                CurrentStatus  = PreStatus;
                PreStatus = 0;
                break;
            }
    
        case I2C_CMD_GET_MP3_NUMBER:
        case I2C_CMD_GET_CURRENT:
        case I2C_CMD_ENTER_SLEEP:
        case I2C_CMD_CHIP_RESET:
        case I2C_CMD_PAUSE:
        case I2C_CMD_STOP_BACKGROUND:
        case I2C_CMD_OPEN_CLOSE_DAC:
            commandReceive = I2C_CMD_NULL;
            SendCMD(commandOption.data.Header.type, 0);
            break;
            
        case I2C_CMD_BIG_DIRECT:
            commandReceive = I2C_CMD_NULL;
            
            SendCMD(commandOption.data.Header.type,  commandOption.data.commands.cmd.parameters[0]+
                                                    (commandOption.data.commands.cmd.parameters[1]&0xFF)*256+
                                                    (commandOption.data.commands.cmd.parameters[2]&0x0F)*256*16);
            break;


        case I2C_CMD_VOLUME_CHANGE:
            commandReceive = I2C_CMD_NULL;
            if(commandOption.data.commands.cmd.parameters[0] == 1)
            {
                VolumeValue += commandOption.data.commands.cmd.parameters[1];
            }else
            {
                if(VolumeValue >= commandOption.data.commands.cmd.parameters[1])
                {
                    VolumeValue -= commandOption.data.commands.cmd.parameters[1];
                }else
                {
                    VolumeValue = 0;
                }
            }
            if(VolumeValue > APP_MAX_VOLUME)VolumeValue = APP_MAX_VOLUME;
            SendCMD(I2C_CMD_VOLUME_VALUE, (VolumeValue*MP3_MAX_VOLUME)/APP_MAX_VOLUME);
            break;

        case I2C_CMD_VOLUME_VALUE:
            commandReceive = I2C_CMD_NULL;
            VolumeValue = commandOption.data.commands.cmd.parameters[0];
            if(VolumeValue > APP_MAX_VOLUME)VolumeValue = APP_MAX_VOLUME;
            SendCMD(I2C_CMD_VOLUME_VALUE, (VolumeValue*MP3_MAX_VOLUME)/APP_MAX_VOLUME);
            break;
        case I2C_CMD_LOOP_CURRENT:
            if (CurrentMusic != -1)
            {
                SendCMD(commandReceive, commandOption.data.commands.cmd.parameters[0]);
            }
            commandReceive = I2C_CMD_NULL;
            CurrentStatus = K403A_LOOPONE_PLAY;
            PreStatus = 0;
            break;
        
        case I2C_CMD_LOOP_DIRECT:
        case I2C_CMD_MUSIC_STYLE:
            SendCMD(commandReceive, commandOption.data.commands.cmd.parameters[0]);
            commandReceive = I2C_CMD_NULL;
            break;
        case I2C_CMD_FIXED_ONE:
            CurrentMusic = commandOption.data.commands.cmd.parameters[1]*256+commandOption.data.commands.cmd.parameters[0];
        case I2C_CMD_DEVICE_SELECT:
        case I2C_CMD_DIRECT:
        case I2C_CMD_FIXED_LOOP:
        case I2C_CMD_DIRECT_MUSIC:
            SendCMD(commandReceive, commandOption.data.commands.cmd.parameters[1]*256+commandOption.data.commands.cmd.parameters[0]);
            commandReceive = I2C_CMD_NULL;
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
		
        case I2C_CMD_GET_EVENT:
            Wire.write(ptr1+4, 4);
            commandReceive = I2C_CMD_NULL;
			packet_01_data.data.deviceEvent = 0;
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


// Note: You must define a SoftwareSerial class object that the name must be Serial, 
//       but you can change the pin number according to the actual situation.
//SoftwareSerial Serial(2, 3);         // define in the demo file 

void DoSum( uint8_t *Str, uint8_t len)
{
    uint16_t xorsum = 0;
    uint8_t i;

    for(i=0; i<len; i++)
    {
        xorsum  = xorsum + Str[i];
    }
    xorsum     = 0 -xorsum;
    *(Str+i)   = (uint8_t)(xorsum >>8);
    *(Str+i+1) = (uint8_t)(xorsum & 0x00ff);
}

/**************************************************************** 
 * Function Name: SelectPlayerDevice
 * Description: Select the player device, U DISK or SD card.
 * Parameters: 0x01:U DISK;  0x02:SD card
 * Return: none
****************************************************************/ 
void SendCmd()
{
    uint8_t i;
    
    for(i=0;i<10;i++)
    {
        Serial.write(cmdData[GetCmd&(K403A_COMMAND_NUM-1)][i]);
    }
    GetCmd++;
    CommandMillis = millis();
}

/**************************************************************** 
 * Function Name: SendCMD
 * Description: SendCMD.
 * Parameters: command parameters.
 * Return: none
****************************************************************/ 
void SendCMD(uint8_t cmd, uint16_t para)
{
    cmdData[PutCmd&(K403A_COMMAND_NUM-1)][0] = K403A_START_BYTE;
    cmdData[PutCmd&(K403A_COMMAND_NUM-1)][1] = K403A_VERSION_BYTE;
    cmdData[PutCmd&(K403A_COMMAND_NUM-1)][2] = K403A_LENGTH_BYTE;
    cmdData[PutCmd&(K403A_COMMAND_NUM-1)][3] = cmd;
    cmdData[PutCmd&(K403A_COMMAND_NUM-1)][4] = LightRawData;
    cmdData[PutCmd&(K403A_COMMAND_NUM-1)][5] = (uint8_t((para>>8)&0xFF));
    cmdData[PutCmd&(K403A_COMMAND_NUM-1)][6] = (uint8_t(para&0xFF));
    cmdData[PutCmd&(K403A_COMMAND_NUM-1)][9] = K403A_STOP_BYTE;
    
    DoSum(cmdData[PutCmd&(K403A_COMMAND_NUM-1)]+1, K403A_LENGTH_BYTE);

    PutCmd++;

}

