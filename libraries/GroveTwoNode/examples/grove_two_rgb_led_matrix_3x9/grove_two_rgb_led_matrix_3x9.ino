
#include "Wire.h"
#include "Flash.h"
#include <LowPower.h>
#include <WatchDog.h>
#include "Timer3.h"
#include "Timer14.h"

#define GROVE_RX_PIN_NUM        PA10    // I2C_SDA

/***************************************************************
 Board defines
 ***************************************************************/
#define MATRIX_DI_PIN_NUM       PA7
#define MATRIX_DCK_PIN_NUM      PA5
#define MATRIX_DI_PIN_BIT       0x80
#define MATRIX_DCK_PIN_BIT      0x20
// when LINE_SELECT_EN_PIN is high, V_drive=3.3v, line selector works,
// my9221 works， screen number selector works
#define LINE_SELECT_EN_PIN      PA6
#define LINE_ADDR_0             PF0
#define LINE_ADDR_1             PF1
#define LINE_ADDR_2             PA0
#define LINE_ADDR_3             PA4
#define LINE_ADDR_0_1_BIT       0x03
#define LINE_ADDR_2_3_BIT       0x11
#define LINE_EN_BIT             0x40

#define MAX_LINES_COUNT        9
#define MAX_ROWS_COUNT         3
/***************************************************************

 ***************************************************************/
#define MATRIX_CMD_MODE_0       0x0000
#define MATRIX_CMD_MODE_1       0x0010  //apdm

/***************************************************************

 ***************************************************************/

uint8_t receiveBuffer[128] = {0,};
uint8_t display_buf[MAX_LINES_COUNT+7][MAX_ROWS_COUNT][3] = {0,};
uint8_t display_buf1[MAX_LINES_COUNT+7][MAX_ROWS_COUNT][3] = {0,};
uint32_t RGBFrame = 0;
/***************************************************************
 Communication defines
 ***************************************************************/
// #define DEVICE_I2C_BASE_ADDRESS 0x60
// change it from 0x10 to 0x60

#define DEVICE_I2C_ADDRESS          0x58
#define DEVICE_VID                  0x2886
#define DEVICE_PID                  0x8005

#define I2C_DEF_ADDR_FLASH_LOC      0x00
#define I2C_CUR_ADDR_FLASH_LOC      0x01

#define I2C_CMD_GET_DEV_ID                      0x00 // This command gets device ID information
#define I2C_CMD_DISP_BAR                        0x01 // This command displays LED bar
#define I2C_CMD_DISP_EMOJI                      0x02 // This command displays emoji
#define I2C_CMD_DISP_NUM                        0x03 // This command displays number
#define I2C_CMD_DISP_STR                        0x04 // This command displays string
#define I2C_CMD_DISP_CUSTOM                     0x05 // This command displays user-defined pictures
#define I2C_CMD_DISP_OFF                        0x06 // This command cleans the display
#define I2C_CMD_DISP_SET                        0x07 // This command set or clear the display by position
#define I2C_CMD_DISP_FLASH                      0x08 // This command displays pictures which are stored in flash
#define I2C_CMD_DISP_COLOR_BAR                  0x09 // This command displays colorful led bar
#define I2C_CMD_DISP_COLOR_WAVE                 0x0a // This command displays built-in wave animation
#define I2C_CMD_DISP_COLOR_CLOCKWISE            0x0b // This command displays built-in clockwise animation
#define I2C_CMD_DISP_COLOR_ANIMATION            0x0c // This command displays other built-in animation
#define I2C_CMD_DISP_COLOR_BLOCK                0x0d // This command displays an user-defined color
#define I2C_CMD_DISP_SWITCH                     0x0e // This command displays an user-defined color
#define I2C_CMD_STORE_FLASH                     0xa0 // This command stores frames in flash
#define I2C_CMD_DELETE_FLASH                    0xa1 // This command deletes all the frames in flash

#define I2C_CMD_LED_ON              0xb0 //
#define I2C_CMD_LED_OFF             0xb1 //
#define I2C_CMD_AUTO_SLEEP_ON       0xb2 //
#define I2C_CMD_AUTO_SLEEP_OFF      0xb3 //

#define I2C_CMD_DISP_ROTATE         0xb4 // This command setting the display orientation
#define I2C_CMD_DISP_OFFSET         0xb5 // This command setting the display offset

#define I2C_CMD_SET_ADDR            0xc0 //
#define I2C_CMD_RST_ADDR            0xc1 //

#define I2C_CMD_DISP_GET            0xd1 //

#define I2C_CMD_TEST_TX_RX_ON       0xe0 //
#define I2C_CMD_TEST_TX_RX_OFF      0xe1 //
#define I2C_CMD_TEST_GET_VER        0xe2 //

#define I2C_CMD_DISP_ICON           0xe5 //

#define I2C_CMD_JUMP_TO_BOOT        0xf0 //
#define I2C_CMD_GET_DEVICE_UID      0xf1
#define I2C_CMD_NULL                0xff //

#define DISPLAY_NOTHING             0
#define DISPLAY_SINGLE_CHARACTER    1
#define DISPLAY_MUTLI_CHARACTERS    2
#define DISPLAY_FLASH_BUFFER        3
#define DISPLAY_COLOR_BAR           4
#define DISPLAY_COLOR_EMOJI         5
#define DISPLAY_COLOR_WAVE_GIF      6
#define DISPLAY_COLOR_CLOCKWISE_GIF 7
#define DISPLAY_COLOR_NORMAL_GIF    8
#define DISPLAY_RAINBOW_CYCLE       9
#define DISPLAY_STOP                10
#define DISPLAY_COLOR_BLOCK         11
#define DISPLAY_SINGLE_PIXEL        12
#define DISPLAY_SWITCH              13
#define DISPLAY_COLOR_ICON          14

uint8_t base_address = DEVICE_I2C_ADDRESS;
uint16_t device_i2c_address = base_address;
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

uint32_t autoSleepPreviousMillis = 0;
bool autoSleepFlag = false;
// bool autoSleepFlag = true;

#define LED_FLASH_TIME  1000

// bool ledFlashCommand = true;
bool ledFlashCommand = false;
bool ledFlashStatus = false;
uint32_t ledFlashPreviousMillis = 0;
uint8_t ledFlashTimes = 0;

uint16_t NodeVersion = 0x6307;
/* Update for bug FLASH, increase delay time to 12 us from 10 us in latchData, 
       and add BSRR clear in clearDisplay(void).
6305: Move to STM32030F4.
6304: Update to 16 line flash, and send random data to 9-15 line.
6303: Update to 16 lines flash.
6302: Update to 24-bits real color mode.
*/

uint32_t intStart = 0;
uint32_t intEnd = 0;

// display flags and variable
uint8_t display_stop_flag = 0, current_line = 0, ContinueNumber = 0;              

// display list
uint8_t display_list[MAX_LINES_COUNT*MAX_ROWS_COUNT*3] = {0, };  // list of display, default display picture is ASCII[1]

uint16_t timer3_period = 50000;         // 20ms-50ms
uint8_t RGBData = 0;
// new
uint8_t display_type;

/***************************************************************
 Device initialization
 ***************************************************************/
void setup()
{

    uint8_t default_base_address = Flash.read8(I2C_DEF_ADDR_FLASH_LOC);
    uint8_t current_base_address = Flash.read8(I2C_CUR_ADDR_FLASH_LOC);
    if (default_base_address != DEVICE_I2C_ADDRESS) Flash.write8(I2C_DEF_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
    if (current_base_address == 0xff) Flash.write8(I2C_CUR_ADDR_FLASH_LOC, DEVICE_I2C_ADDRESS);
    else base_address = current_base_address;
    
    *((uint8_t *)(Flash.blockdata)+I2C_DEF_ADDR_FLASH_LOC) = default_base_address;
    *((uint8_t *)(Flash.blockdata)+I2C_CUR_ADDR_FLASH_LOC) = current_base_address;

    device_i2c_address = base_address;

    packet_01_data.data.deviceVID = DEVICE_VID;
    packet_01_data.data.devicePID = DEVICE_PID;
    packet_01_data.data.deviceEvent = 0;

    matrixInit();
    Wire.begin(device_i2c_address);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    wwdg.begin();
}

void loop()
{
    uint32_t CurrentMillis = millis();

    if(ledFlashCommand)
    {
        if(CurrentMillis - ledFlashPreviousMillis >= LED_FLASH_TIME)
        {
            ledFlashPreviousMillis = CurrentMillis;
			ledFlashTimes++;
			if(ledFlashTimes % 2)memset(display_list, 255, MAX_LINES_COUNT*MAX_ROWS_COUNT*3);
			else memset(display_list, 0, MAX_LINES_COUNT*MAX_ROWS_COUNT*3);
			if (display_stop_flag)
			{
				writeDisplayBuf1(display_list);
			}else
			{
				writeDisplayBuf(display_list);
			}
			display_stop_flag = 1-display_stop_flag;
        }
    }

    if(autoSleepFlag)
    {
        autoSleepFlag = false;
        Timer14.stop();
        Timer3.stop();
        
        digitalWrite(LINE_SELECT_EN_PIN, LOW);
        digitalWrite(LINE_ADDR_0, LOW);
        digitalWrite(LINE_ADDR_1, LOW);
        digitalWrite(LINE_ADDR_2, LOW);
        digitalWrite(LINE_ADDR_3, LOW);
        digitalWrite(LINE_SELECT_EN_PIN, LOW);

        wwdg.end();
        Wire.end();
        pinMode(PA9, INPUT_PULLUP);
        pinMode(PA10, INPUT_PULLUP);

        nrgSave.begin(GROVE_RX_PIN_NUM, dummy, CHANGE);
        delay(100);

        nrgSave.standby();

        Wire.begin(device_i2c_address);
        Wire.onReceive(receiveEvent);
        Wire.onRequest(requestEvent);
        
        wwdg.begin();
        matrixInit();
    }

    wwdg.reset();
}

void dummy(void)
{
    autoSleepPreviousMillis = millis();
}

void receiveEvent(int howMany)
{
    uint8_t i = 0, j = 0;
    // autoSleepPreviousMillis = millis();

    while(Wire.available())
    {
        receiveBuffer[i ++] = Wire.read();
        if(i > 120)i = 0;
    }

    commandReceive = receiveBuffer[0];

    switch(commandReceive)
    {
        case I2C_CMD_DISP_CUSTOM:
            // display_flash_buf_flag = true;
            display_type = DISPLAY_FLASH_BUFFER;
            // data start at receiveBuffer[3]
            for (j=0;j<(i-1);j++)
            {
                display_list[j] = receiveBuffer[1+j];
            }
			ledFlashCommand = false;
        break;

        case I2C_CMD_LED_ON:
            ledFlashCommand = true;
        break;

        case I2C_CMD_LED_OFF:
            ledFlashCommand = false;
            ledFlashStatus = false;
        break;

        case I2C_CMD_DISP_OFF:
            ledFlashCommand = false;
            display_type = DISPLAY_NOTHING;
        break;
        
        case I2C_CMD_DISP_SET:
            // r, g, b
            if ((receiveBuffer[1] < MAX_LINES_COUNT) && (receiveBuffer[2] < MAX_ROWS_COUNT))
            {
				display_list[(receiveBuffer[1]*MAX_ROWS_COUNT+receiveBuffer[2])*3+0] = receiveBuffer[3];
				display_list[(receiveBuffer[1]*MAX_ROWS_COUNT+receiveBuffer[2])*3+1] = receiveBuffer[4];
				display_list[(receiveBuffer[1]*MAX_ROWS_COUNT+receiveBuffer[2])*3+2] = receiveBuffer[5];
			}
            display_type = DISPLAY_SINGLE_PIXEL;
			ledFlashCommand = false;
        break;  

        case I2C_CMD_AUTO_SLEEP_ON:
            autoSleepFlag = true;
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_AUTO_SLEEP_OFF:
            autoSleepFlag = false;
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

        case I2C_CMD_TEST_GET_VER:
            Wire.write((const uint8_t *)&NodeVersion, 2);
            commandReceive = I2C_CMD_NULL;
        break;

        case I2C_CMD_GET_DEVICE_UID:
            Wire.write((char *)(Flash.getChipUniqueID()), 12);
            commandReceive = I2C_CMD_NULL;
        break;

        default:
        break;
    }
}

/***************************************************************
 Device driver
 ***************************************************************/
void matrixInit()
{
    pinMode(MATRIX_DCK_PIN_NUM, OUTPUT);
    pinMode(MATRIX_DI_PIN_NUM, OUTPUT);
    pinMode(LINE_ADDR_0, OUTPUT);
    pinMode(LINE_ADDR_1, OUTPUT);
    pinMode(LINE_ADDR_2, OUTPUT);
    pinMode(LINE_ADDR_3, OUTPUT);
    pinMode(LINE_SELECT_EN_PIN, OUTPUT);
    digitalWrite(MATRIX_DCK_PIN_NUM, LOW);
    digitalWrite(MATRIX_DI_PIN_NUM, LOW);
    digitalWrite(LINE_ADDR_0, LOW);
    digitalWrite(LINE_ADDR_1, LOW);
    digitalWrite(LINE_ADDR_2, LOW);
    digitalWrite(LINE_ADDR_3, LOW);
    digitalWrite(LINE_SELECT_EN_PIN, HIGH);
    clearDisplay();

    Timer3.init(timer3_period); // 50ms timer
    Timer3.attachInterrupt(displayControlThread);
    // 40fps => 40*8 hz => 3.3ms oneline
    // 1ms * 8lines => 8ms => 125Hz
    // displayOneLine() takes about 0.4ms
    Timer14.init(1000);
    Timer14.attachInterrupt(timerIsr);

}


 void timerIsr()
 {
   //An Ideal ISR has to be short and not make any function calls
   //But, in this case only data exchange happens.
   // blink the led PA1 
//    GPIOA->BRR = 0x02;
   displayOneLine();
//    GPIOA->BSRR = 0x02;
 }


// Routine to send 16bit data to MY9221 driver chips
// send MSB first
void send16BitData(uint16_t data)
{
    for (uint8_t i = 0; i < 16; i++)
    {
      GPIOA->BRR = MATRIX_DI_PIN_BIT;
      GPIOA->BSRR = ((1 && (data & 0x8000)) << 7); //if (data & 0x8000){GPIOA->BSRR = DI_PIN_BIT;}
      GPIOA->ODR ^= MATRIX_DCK_PIN_BIT;
      data <<= 1;
    }
}

// clear data in MY9221
void clearDisplay(void)
{
    send16BitData(MATRIX_CMD_MODE_1);
    // set data pin MATRIX_DI_PIN_NUM to low
    // BRR to set low, BSRR to set high
	GPIOA->BRR = MATRIX_DI_PIN_BIT;
	GPIOA->BSRR = 0;
    for (uint8_t i=0;i<192;i++)
    {
        GPIOA->ODR ^= MATRIX_DCK_PIN_BIT;
    }

    latchData();
}

// line 0-8
void switchToLine(uint8_t line)
{
	uint8_t line_a;
	
    // set PF0 PF1 PA0 PA4 to low
    GPIOF->BRR = LINE_ADDR_0_1_BIT;
    GPIOA->BRR = LINE_ADDR_2_3_BIT;
    // set line bit for PF and PA.
	line_a = ((line & 0x08) << 1) | ((line & 0x04) >> 2);
    GPIOF->BSRR = (LINE_ADDR_0_1_BIT & line);
    GPIOA->BSRR = (LINE_ADDR_2_3_BIT & line_a);
    // enable the line selector PA6
    GPIOA->BSRR = LINE_EN_BIT;
}

void latchData(void)
{
    //why not 220us ? need to test
    delayMicroseconds(220);
    // send 4 DI pulses
    GPIOA->BRR = MATRIX_DI_PIN_BIT;
    for (uint8_t i=0;i<8;i++)
    {
        GPIOA->ODR ^= MATRIX_DI_PIN_BIT;
    }
}

// clearDisplay() takes 0.13ms
// send16BitData() totally takes 0.26ms
void displayOneLine(void)
{
    uint8_t scroll_mark_copy = 0;
    uint8_t line = (current_line % (MAX_LINES_COUNT+7));

	current_line++;
//	if (line >= MAX_LINES_COUNT)return;

    clearDisplay();
    // clearDisplay();

    send16BitData(MATRIX_CMD_MODE_1);
    // none
    send16BitData(0);
    send16BitData(0);
    send16BitData(0);

    if (display_stop_flag)
    {
		// blue
        send16BitData(display_buf[line][2][2]);
        send16BitData(display_buf[line][1][2]);
        send16BitData(display_buf[line][0][2]);
        // green
        send16BitData(display_buf[line][2][1]);
        send16BitData(display_buf[line][1][1]);
        send16BitData(display_buf[line][0][1]);
        // red
        send16BitData(display_buf[line][2][0]);
        send16BitData(display_buf[line][1][0]);
        send16BitData(display_buf[line][0][0]);
    }else
    {
		// blue
        send16BitData(display_buf1[line][2][2]);
        send16BitData(display_buf1[line][1][2]);
        send16BitData(display_buf1[line][0][2]);
        // green
        send16BitData(display_buf1[line][2][1]);
        send16BitData(display_buf1[line][1][1]);
        send16BitData(display_buf1[line][0][1]);
        // red
        send16BitData(display_buf1[line][2][0]);
        send16BitData(display_buf1[line][1][0]);
        send16BitData(display_buf1[line][0][0]);
    }
    switchToLine(line);
//    switchToLine(MAX_LINES_COUNT-line-1);

    latchData();

}

/***************************************************************
 name:      writeDisplayBuf(1)
 function:  write a 8byte character to displaybuf after rotation
            and offset.
 para:      @data: 8byte character data, such as ASCII[0]
            @color_num: 0-255, 254 is white and 255 is black
 global:    
            @display_buf, @temp_rotation_buf
 ***************************************************************/
void writeDisplayColor(uint8_t data_x, uint8_t data_y, uint8_t color_r, uint8_t color_g, uint8_t color_b)
{
    RGBFrame |= (1 << (data_x*MAX_ROWS_COUNT+data_y));
    display_buf[data_x][data_y][0] = color_r;
    display_buf[data_x][data_y][1] = color_g;
    display_buf[data_x][data_y][2] = color_b;
}


/***************************************************************
 name:      writeDisplayBuf(2)
 function:  write a 27 bytes picture to displaybuf, no rotation
            and offset.
 para:      @buf: uint32_t pointer of the picture buf(Flash.blockdata)
 global:    @display_buf
 ***************************************************************/
void writeDisplayBuf(uint8_t *buf)
{
	RGBFrame = 0;
    for (uint8_t i = 0; i < MAX_LINES_COUNT; i++)
    {
        for (uint8_t j = 0; j < MAX_ROWS_COUNT; j++)
        {
            display_buf[MAX_LINES_COUNT-1-i][j][0] = buf[(i*MAX_ROWS_COUNT+j)*3+0];
            display_buf[MAX_LINES_COUNT-1-i][j][1] = buf[(i*MAX_ROWS_COUNT+j)*3+1];
            display_buf[MAX_LINES_COUNT-1-i][j][2] = buf[(i*MAX_ROWS_COUNT+j)*3+2];
			if ((display_buf[MAX_LINES_COUNT-1-i][j][0]+
				 display_buf[MAX_LINES_COUNT-1-i][j][1]+
				 display_buf[MAX_LINES_COUNT-1-i][j][2]) > 0)
				RGBFrame |= (1 << (i*MAX_ROWS_COUNT+j));
        }
    }
}


/***************************************************************
 name:      writeDisplayBuf(2)
 function:  write a 27 bytes picture to displaybuf, no rotation
            and offset.
 para:      @buf: uint32_t pointer of the picture buf(Flash.blockdata)
 global:    @display_buf
 ***************************************************************/
void writeDisplayBuf1(uint8_t *buf)
{
	RGBFrame = 0;
    for (uint8_t i = 0; i < MAX_LINES_COUNT; i++)
    {
        for (uint8_t j = 0; j < MAX_ROWS_COUNT; j++)
        {
            display_buf1[MAX_LINES_COUNT-1-i][j][0] = buf[(i*MAX_ROWS_COUNT+j)*3+0];
            display_buf1[MAX_LINES_COUNT-1-i][j][1] = buf[(i*MAX_ROWS_COUNT+j)*3+1];
            display_buf1[MAX_LINES_COUNT-1-i][j][2] = buf[(i*MAX_ROWS_COUNT+j)*3+2];
			if ((display_buf1[MAX_LINES_COUNT-1-i][j][0]+
				 display_buf1[MAX_LINES_COUNT-1-i][j][1]+
				 display_buf1[MAX_LINES_COUNT-1-i][j][2]) > 0)
				RGBFrame |= (1 << (i*MAX_ROWS_COUNT+j));
        }
    }
}

/***************************************************************
 name:      displayControlThread
 function:
 para:
 global:
 ***************************************************************/
void displayControlThread(void)
{

    switch(display_type){
        case DISPLAY_NOTHING:  //clear screen
            if (display_stop_flag)
            {
				memset(display_buf1, 0, MAX_LINES_COUNT*MAX_ROWS_COUNT*3);
			}else
			{
				memset(display_buf, 0, MAX_LINES_COUNT*MAX_ROWS_COUNT*3);
			}
            RGBFrame = 0;
            display_stop_flag = 1-display_stop_flag;
			display_type = DISPLAY_STOP;
        break;

        case DISPLAY_SINGLE_PIXEL: //display_list[0]
            if (display_stop_flag)
            {
                writeDisplayBuf1(display_list);
			}else
			{
				writeDisplayBuf(display_list);
			}
            display_stop_flag = 1-display_stop_flag;
			display_type = DISPLAY_STOP;
        break;

        case DISPLAY_FLASH_BUFFER:      // Display 27 bytes custom pictures
            if (display_stop_flag)
            {
                writeDisplayBuf1(display_list);
			}else
			{
				writeDisplayBuf(display_list);
			}
            display_stop_flag = 1-display_stop_flag;
			display_type = DISPLAY_STOP;
        break;

        default:  
		// DISPLAY_STOP, do nothing;
        break;
    }
}

