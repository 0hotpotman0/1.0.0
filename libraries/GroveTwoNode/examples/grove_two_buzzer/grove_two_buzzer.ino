
#include <Wire.h>
#include <Flash.h>
#include <LowPower.h>
#include <WatchDog.h>


#define GROVE_TWO_TX_PIN_NUM    PA2
#define GROVE_TWO_RX_PIN_NUM    PA3

#define GROVE_LED_PIN_NUM       PA1

#define GROVE_BUZZER_PIN        PB1

enum gamut_type_t
{ 
    NOTE_R0 = 0, // Null
    
    NOTE_C3 = 1, // Low C
    NOTE_D3 = 2, // Low D
    NOTE_E3 = 3, // Low E
    NOTE_F3 = 4, // Low F
    NOTE_G3 = 5, // Low G
    NOTE_A3 = 6, // Low A
    NOTE_B3 = 7, // Low B
    
    NOTE_C4 = 8,  // Middle C
    NOTE_D4 = 9,  // Middle D
    NOTE_E4 = 10, // Middle E
    NOTE_F4 = 11, // Middle F
    NOTE_G4 = 12, // Middle G
    NOTE_A4 = 13, // Middle A
    NOTE_B4 = 14, // Middle B
    
    NOTE_C5 = 15, // High C
    NOTE_D5 = 16, // High D
    NOTE_E5 = 17, // High E
    NOTE_F5 = 18, // High F
    NOTE_G5 = 19, // High G
    NOTE_A5 = 20, // High A
    NOTE_B5 = 21, // High B
   
    NOTE_CSharp3 = 22,  //Low C#
    NOTE_Eb3 = 23,      //Low D#
    NOTE_FSharp3 = 24,  //Low F#
    NOTE_GSharp3 = 25,  //Low G#
    NOTE_Bb3 = 26,      //Low A#
    NOTE_CSharp4 = 27,  //Middle C#
    NOTE_Eb4 = 28,      //Middle D#
    NOTE_FSharp4 = 29,  //Middle F#
    NOTE_GSharp4 = 30,  //Middle G#
    NOTE_Bb4 = 31,      //Middle A#
    NOTE_CSharp5 = 32,  //High C#
    NOTE_Eb5 = 33,      //High D#
    NOTE_FSharp5 = 34,  //High F#
    NOTE_GSharp5 = 35,  //High G#
    NOTE_Bb5 = 36,      //High A#
/*  
    NOTE_CS3 = 22,
    NOTE_Eb3 = 23,
    NOTE_FS3 = 24,
    NOTE_GS3 = 25,
    NOTE_Bb3 = 26,
    
    NOTE_CS4 = 27,
    NOTE_Eb4 = 28,
    NOTE_FS4 = 29,
    NOTE_GS4 = 30,
    NOTE_Bb4 = 31,
    
    NOTE_CS5 = 32,
    NOTE_Eb5 = 33,
    NOTE_FS5 = 34,
    NOTE_GS5 = 35,
    NOTE_Bb5 = 36,
    */
    NOTE_C6 = 37,
    NOTE_D6 = 38,
    NOTE_E6 = 39,
    NOTE_F6 = 40,
    NOTE_G6 = 41,
    NOTE_A6 = 42,
    NOTE_B6 = 43,

    NOTE_C7 = 44,
    NOTE_D7 = 45,
    NOTE_E7 = 46,
    NOTE_F7 = 47, 
    NOTE_G7 = 48,
    NOTE_A7 = 49, 
    NOTE_B7 = 50,

    NOTE_CS6 = 51,
    NOTE_Eb6 = 52,
    NOTE_FS6 = 53, 
    NOTE_GS6 = 54,
    NOTE_Bb6 = 55,

    NOTE_CS7 = 56,
    NOTE_Eb7 = 57,
    NOTE_FS7 = 58, 
    NOTE_GS7 = 59,
    NOTE_Bb7 = 60,

};

const uint16_t gamutFreqTable[] = 
{
    0, // stop, 0
    
    131, 147, 165, 175, 196, 220, 247, // #3, 1 - 7
    262, 294, 330, 349, 392, 440, 494, // #4, 8 - 14
    523, 587, 659, 698, 784, 880, 988, // #5, 15 - 21
    
    139, // C#3, 22
    156, // Eb3, 23
    185, // F#3, 24
    208, // G#3, 25
    233, // Bb3, 26
    
    277, // C#4, 27
    311, // Eb4, 28
    370, // F#4, 29
    415, // G#4, 30
    466, // Bb4, 31
    
    555, // C#5, 32
    622, // Eb5, 33
    740, // F#5, 34
    831, // G#5, 35
    932, // Bb5, 36
    
    1047, 1175, 1319, 1397, 1568, 1760, 1976, // #6, 37 - 43
    2093, 2349, 2637, 2794, 3136, 3520, 3951, // #7, 44 - 50

    1109, // C#6, 52
    1245, // Eb6, 53
    1480, // F#6, 54
    1661, // G#6, 55
    1865, // Bb6, 56
    
    2217, // C#7, 57
    2489, // Eb6, 58
    2960, // F#6, 59
    3322, // G#6, 60
    3729, // Bb6, 61
};

enum beat_type_t
{
    BEAT_1 = 0, // 1 beat
    BEAT_2 = 1, // 2 beat
    BEAT_4 = 2, // 4 beat
    BEAT_8 = 3, // 8 beat
    BEAT_1_2 = 4, // 1/2 beat
    BEAT_1_4 = 5, // 1/4 beat
    BEAT_1_8 = 6, // 1/8 beat
    BEAT_1_16 = 7, // 1/16 beat

    BEAT_MAX
};

const float beatTable[] = 
{
    1, 2, 4, 8, 0.5, 0.25, 0.125, 0.0625,
};

const uint8_t BaDing[] = {NOTE_B5, 1, NOTE_E6, 3};
const uint8_t Wawawawaa[] = {NOTE_E3, 3, NOTE_R0, 1, NOTE_Eb3, 3, NOTE_R0, 1, NOTE_D3, 4, NOTE_R0, 1, NOTE_CSharp3, 8};
const uint8_t JumpUp[] = {NOTE_C5, 1, NOTE_D5, 1, NOTE_E5, 1, NOTE_F5, 1, NOTE_G5, 1};
const uint8_t JumpDown[] = {NOTE_G5, 1, NOTE_F5, 1, NOTE_E5, 1, NOTE_D5, 1, NOTE_C5, 1};
const uint8_t PowerUp[] = {NOTE_G4, 1, NOTE_C5, 1, NOTE_E5, 1, NOTE_G5, 2, NOTE_E5, 1, NOTE_G5, 3};
const uint8_t PowerDown[] = {NOTE_G5, 1, NOTE_Eb5, 1, NOTE_C5, 1, NOTE_G4, 2, NOTE_B4, 1, NOTE_C5, 3};
const uint8_t MagicWand[] = {NOTE_FS6, 1, NOTE_GS6, 1, NOTE_Bb6, 1, NOTE_B6, 1, NOTE_CS7, 1, NOTE_Eb7, 1, NOTE_F7, 1, NOTE_FS7, 1, NOTE_GS7, 1, NOTE_Bb7, 1, NOTE_B7, 6};
const uint8_t Siren[] = {NOTE_A4, 4, NOTE_D5, 4, NOTE_A4, 4, NOTE_D5, 4, NOTE_A4, 4, NOTE_D5, 4};


#define BPM_MIN_NUM 60
#define BPM_MAX_NUM 960
#define BEAT_DEFAULT_NUM    120

uint16_t beatPerMin = BEAT_DEFAULT_NUM;
uint32_t playTimer = 0;
uint32_t playTimerPreviousMillis = 0;
bool playFlag = false;

bool melodyFlag = false;
bool melodyRepeat = false;
uint8_t melodyItem = 0;
uint16_t melodyLen = 0;
uint16_t melodyCount = 0;
uint16_t melodyStep = 0;
const uint8_t *ptr = NULL;
uint8_t ledFlashTimes = 0;

void buzzerPlayToneWithBeat(gamut_type_t type, beat_type_t beat);
void buzzerPlayToneWithTime(gamut_type_t type, uint16_t times);
void buzzerPlayMelody(uint8_t type);

/***************************************************************

 ***************************************************************/
#define DEVICE_I2C_ADDRESS      0x08
#define DEVICE_VID              0x2886
#define DEVICE_PID              0x8003

#define I2C_DEF_ADDR_FLASH_LOC      0x00 // byte address
#define I2C_CUR_ADDR_FLASH_LOC      0x01 // byte address
#define I2C_BPM_ADDR_FLASH_LOC      0x01 // int address

#define I2C_CMD_GET_DEV_ID      0x00 //  
#define I2C_CMD_PLAY_TONE       0x01 // 
#define I2C_CMD_RING_TONE       0x02 // 
#define I2C_CMD_PLAY_FREQ       0x11 // 
#define I2C_CMD_RING_FREQ       0x12 // 
#define I2C_CMD_PLAY_MELODY     0x03 //
#define I2C_CMD_PLAY_STOP       0x04 // 
#define I2C_CMD_SET_BPM         0x05 // 
#define I2C_CMD_CHG_BPM         0x06 // 
#define I2C_CMD_GET_BPM         0x07 //
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

uint16_t deviceI2CAddress =  DEVICE_I2C_ADDRESS;
uint8_t commandReceive = I2C_CMD_NULL;

#ifdef BLE_SUPPORT

#define I2C_CMD_BLE_PLAY_TONE   0x90
#define I2C_CMD_BLE_RING_TONE   0x91
#define I2C_CMD_BLE_PLAY_MELODY 0x92
#define I2C_CMD_BLE_PLAY_STOP   0x93
#define I2C_CMD_BLE_SET_BPM     0x94
#define I2C_CMD_BLE_CHG_BPM     0x95

#define I2C_CMD_LOW_PRIORITY    0x90

uint8_t Core_mode = 0;
uint32_t StartMillis = 0;
uint32_t preEvent;
bool displayPriorityFlag = false;

typedef struct
{
    uint8_t Datalen;
    uint8_t type;
    uint8_t Address;
    uint8_t Option;
}packet_header_t;

typedef struct
{
    uint8_t     gamut;
    uint8_t     beat;
}packet_play_tone;

typedef struct
{
    uint8_t     gamut;
    uint8_t     time[2];
}packet_ring_tone;

typedef struct
{
    uint8_t melody;
    uint8_t repeat;
}packet_play_melody;

typedef struct
{
    uint8_t     bpm[2];
    uint8_t     flashSave;
}packet_set_bpm;

typedef struct
{
    uint8_t     sign;
    uint8_t     bpm[2];
    uint8_t     flashSave;
}packet_chg_bpm;

typedef struct
{
    packet_header_t Header;
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
        packet_play_tone    play_tone;
        packet_ring_tone    ring_tone;
        packet_play_melody  play_melody;
        packet_set_bpm      beatpermin;
        packet_chg_bpm      beatchg;
        packet_got_atr      atr;
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
    packet_header_t Header;
    uint8_t     data[2];
}packet_raw;

typedef struct
{
    packet_header_t Header;
    uint8_t     Event;
}packet_event;

typedef struct
{
    packet_header_t Header;
    uint8_t     pid[2];
    uint8_t     chipid;
    uint8_t     hwaddress;
    uint8_t     version[3];
    uint8_t     option[2];
}packet_atr;

union
{
    packet_atr      atr;
    packet_event    event;
    packet_raw      raw_data;
    uint8_t         bytes[MAINBOARD_BLE_I2C_DATALEN]; 
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

void requestEvent(void);
void receiveEvent(int howMany);

/***************************************************************

 ***************************************************************/
LowPower nrgSave;

#define AUTO_SLEEP_TIMEOUT  2000

uint32_t autoSleepPreviousMillis = 0, PreMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;


/***************************************************************

 ***************************************************************/
#define LED_FLASH_TIME  250

bool ledFlashCommand = true;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;

bool testFlag = false;
char *versions = "V20";
uint16_t NodeVersion = 0x6100;

uint32_t intStart = 0;
uint32_t intEnd = 0;

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
    uint16_t tempBPM = Flash.read16(I2C_BPM_ADDR_FLASH_LOC);
    
    uint8_t *ptr3 = (uint8_t *)Flash.getChipUniqueID();
    for(uint8_t i = 0; i < 12; i ++)chipId[i] = *(ptr3 + i);
    
    if(i2cDefaultAddr == 0xff)Flash.write8(I2C_DEF_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
    if(i2cCurrentAddr == 0xff)Flash.write8(I2C_CUR_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
    else deviceI2CAddress = i2cCurrentAddr;
    if(tempBPM == 0xffff)Flash.write16(I2C_BPM_ADDR_FLASH_LOC, BEAT_DEFAULT_NUM);
    else beatPerMin = tempBPM;
    
    packet_01_data.data.deviceVID = DEVICE_VID;
    packet_01_data.data.devicePID = DEVICE_PID;
    packet_01_data.data.deviceEvent = 0;
    
    nrgSave.begin(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance
#ifdef BLE_SUPPORT
    StartMillis = millis();
#endif
        
    pinMode(GROVE_BUZZER_PIN, OUTPUT);
    digitalWrite(GROVE_BUZZER_PIN, LOW);
    
    pinMode(GROVE_LED_PIN_NUM, OUTPUT);
    digitalWrite(GROVE_LED_PIN_NUM, HIGH);
    
    Wire.begin(deviceI2CAddress);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    
    wwdg.begin();
    
    buzzerPlayMelody(0, 0);
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

        switch(commandReceive)
        {
            case  I2C_CMD_GET_RAW_DATA:
            // Raw data request, send BPM.
                InstructionOption.raw_data.Header.type      = I2C_CMD_GET_RAW_DATA;
                InstructionOption.raw_data.Header.Address   = deviceI2CAddress;
                InstructionOption.raw_data.data[0]  = beatPerMin&0xFF;
                InstructionOption.raw_data.data[1]  = (beatPerMin>>8)&0xFF;
                InstructionOption.raw_data.Header.Datalen = sizeof(packet_raw);
                Wire.MasterGPIOTransmission(ptr2, sizeof(packet_raw));
                commandReceive = I2C_CMD_NULL;
                
                break;
            case I2C_CMD_NOTIFY_ATR:
                InstructionOption.atr.Header.type   = I2C_CMD_ATR;
                InstructionOption.atr.Header.Address= deviceI2CAddress;
                InstructionOption.atr.pid[0]        = DEVICE_PID&0xff;
                InstructionOption.atr.pid[1]        = (DEVICE_PID>>8 ) & 0xff;
                InstructionOption.atr.chipid        = chipId[0];
                InstructionOption.atr.hwaddress     = DEVICE_I2C_ADDRESS;
                InstructionOption.atr.version[0]    = versions[0];
                InstructionOption.atr.version[1]    = versions[1];
                InstructionOption.atr.version[2]    = versions[2];
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
    else if(commandReceive == I2C_CMD_SET_BPM || commandReceive == I2C_CMD_CHG_BPM) // set new bpm
    {
        commandReceive = I2C_CMD_NULL;
        if(flashSave)Flash.write16(I2C_BPM_ADDR_FLASH_LOC, beatPerMin);
    }
#ifdef BLE_SUPPORT
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
            }else
            {
                ledFlashCommand = false;
                digitalWrite(GROVE_LED_PIN_NUM, HIGH);
            }
        }
    }

    if(playFlag)
    {
        uint32_t playTimerCurrentMillis = millis();
        if(playTimerCurrentMillis - playTimerPreviousMillis >= playTimer)
        {
            playTimerPreviousMillis = playTimerCurrentMillis;
            noTone(GROVE_BUZZER_PIN);
            playFlag = false;

            if(melodyFlag)
            {
                melodyCount ++;
                if(melodyCount < melodyLen)
                {
                    ptr = ptr + 2;
                    buzzerPlayToneWithTime((gamut_type_t)(*ptr), 15000 / beatPerMin * (*(ptr + 1)));
                }
                else 
                {
                    if(melodyRepeat)
                    {
                        buzzerPlayMelody(melodyItem, true);
                    }else{
                        melodyFlag = false;
                    } 
                }
            }
            if((Core_mode == CORE_BLE_MODE) && (melodyFlag == false))
            {
                displayPriorityFlag = false;
            }
        }
    }
    else // no play, go to sleep
    {
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
                
                nrgSave.standby();
                
                Wire.begin(deviceI2CAddress);
                Wire.onReceive(receiveEvent);
                Wire.onRequest(requestEvent);
                wwdg.begin();
            }
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
    uint8_t i = 0, j, receiveBuffer[MAINBOARD_BLE_I2C_DATALEN] = {0,};
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
        if((commandReceive & 0xF0) == I2C_CMD_LOW_PRIORITY)
        {// Low priority.
            if (displayPriorityFlag)
            {   // High priority task is working.
                commandReceive = I2C_CMD_NULL;
                return;
            }
            commandReceive &= 0x0F;
            commandReceive++;
        }else if(commandReceive < 0x0A)
        {
            commandReceive++;
            displayPriorityFlag = true;         // High priority
        }
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

        case I2C_CMD_PLAY_TONE:
            if(receiveBuffer[1] > 36)receiveBuffer[1] = 36;
            if(receiveBuffer[2] > 7)receiveBuffer[2] = 7;
            buzzerPlayToneWithBeat((gamut_type_t)receiveBuffer[1], (beat_type_t)receiveBuffer[2]);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_RING_TONE:
            if(receiveBuffer[1] > 36)receiveBuffer[1] = 36;
            buzzerPlayToneWithTime((gamut_type_t)receiveBuffer[1], receiveBuffer[2] + receiveBuffer[3] * 256);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_PLAY_FREQ:
            buzzerPlayFreqWithBeat(receiveBuffer[1] + receiveBuffer[2] * 256, (beat_type_t)receiveBuffer[3]);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_RING_FREQ:
            buzzerPlayFreqWithTime(receiveBuffer[1] + receiveBuffer[2] * 256, receiveBuffer[3] + receiveBuffer[4] * 256);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_PLAY_MELODY:
            if(receiveBuffer[1] > 7)receiveBuffer[1] = 7;
            buzzerPlayMelody(receiveBuffer[1], receiveBuffer[2]);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_PLAY_STOP:
            playTimer = 0;
            playFlag = false;
            displayPriorityFlag = false;
            noTone(GROVE_BUZZER_PIN);
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_SET_BPM: // need save to flash
            {
                uint16_t newBPM = receiveBuffer[1] + receiveBuffer[2] * 256;
                if(newBPM < BPM_MIN_NUM)beatPerMin = BPM_MIN_NUM;
                else if(newBPM > BPM_MAX_NUM)beatPerMin = BPM_MAX_NUM;
                else beatPerMin = newBPM;
                flashSave = receiveBuffer[3];
            }
        break;
        
        case I2C_CMD_CHG_BPM: // need save to flash
            {
                uint16_t stepBPM = receiveBuffer[2] + receiveBuffer[3] * 256;
                if(stepBPM < BPM_MAX_NUM)
                {
                    if(receiveBuffer[1]) // Increase BPM
                    {
                        if((int16_t)(beatPerMin + stepBPM) > BPM_MAX_NUM)beatPerMin = BPM_MAX_NUM;
                        else beatPerMin = beatPerMin + stepBPM;
                    }
                    else // Decrease BPM
                    {
                        if((int16_t)(beatPerMin - stepBPM) < BPM_MIN_NUM)beatPerMin = BPM_MIN_NUM;
                        else beatPerMin = beatPerMin - stepBPM;
                    }
                }
                flashSave = receiveBuffer[4];
            }
        break;
        
        case I2C_CMD_LED_ON:
            ledFlashCommand = true;
            displayPriorityFlag = false;
            commandReceive = I2C_CMD_NULL;
        break;
        
        case I2C_CMD_LED_OFF:
            ledFlashCommand = false;
            ledFlashStatus = false;
            displayPriorityFlag = false;
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
        
        case I2C_CMD_GET_BPM:
            packet_01_data.data.deviceEvent = beatPerMin;
            Wire.write(ptr1 + 4, 2);
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
void buzzerPlayToneWithBeat(gamut_type_t type, beat_type_t beat)
{   
    uint16_t durations = 0;
    
    durations = 60000 / beatPerMin * beatTable[(uint8_t)beat];
    playTimerPreviousMillis = millis();
    playTimer = durations;
    playFlag = true;
    tone(GROVE_BUZZER_PIN, gamutFreqTable[(uint8_t)type], durations);
}

void buzzerPlayToneWithTime(gamut_type_t type, uint16_t times)
{       
    playTimerPreviousMillis = millis();
    if(times == 0)playTimer = 0xffffffff;
    else playTimer = times;
    playFlag = true;
    tone(GROVE_BUZZER_PIN, gamutFreqTable[(uint8_t)type], playTimer);
}

void buzzerPlayFreqWithBeat(uint16_t freq, beat_type_t beat)
{   
    uint16_t durations = 0;
    
    durations = 60000 / beatPerMin * beatTable[(uint8_t)beat];
    playTimerPreviousMillis = millis();
    playTimer = durations;
    playFlag = true;
    tone(GROVE_BUZZER_PIN, freq, durations);
}

void buzzerPlayFreqWithTime(uint16_t freq, uint16_t times)
{       
    playTimerPreviousMillis = millis();
    if(times == 0)playTimer = 0xffffffff;
    else playTimer = times;
    playFlag = true;
    tone(GROVE_BUZZER_PIN, freq, playTimer);
}

void buzzerPlayMelody(uint8_t type, bool repeat)
{
    melodyFlag = true;
    melodyCount = 0;
    melodyRepeat = repeat;
    
    buzzerLoadyMelody(type);
    buzzerPlayToneWithTime((gamut_type_t)(*ptr), melodyStep * (*(ptr + 1)));
}

void buzzerLoadyMelody(uint8_t type)
{
    melodyItem = type;
    
    switch(type)
    {
        case 0:
            melodyStep = 125;
            melodyLen = sizeof(BaDing) / 2;
            ptr = BaDing;
        break;
        
        case 1:
            melodyStep = 125;
            melodyLen = sizeof(Wawawawaa) / 2;
            ptr = Wawawawaa;
        break;
        
        case 2:
            melodyStep = 125;
            melodyLen = sizeof(JumpUp) / 2;
            ptr = JumpUp;
        break;
        
        case 3:
            melodyStep = 125;
            melodyLen = sizeof(JumpDown) / 2;
            ptr = JumpDown;
        break;
        
        case 4:
            melodyStep = 125;
            melodyLen = sizeof(PowerUp) / 2;
            ptr = PowerUp;
        break;
        
        case 5:
            melodyStep = 125;
            melodyLen = sizeof(PowerDown) / 2;
            ptr = PowerDown;
        break;
        
        case 6:
            melodyStep = 62;
            melodyLen = sizeof(MagicWand) / 2;
            ptr = MagicWand;
        break;
        
        case 7:
            melodyStep = 125;
            melodyLen = sizeof(Siren) / 2;
            ptr = Siren;
        break;
        
        default:
        break;
    }
}