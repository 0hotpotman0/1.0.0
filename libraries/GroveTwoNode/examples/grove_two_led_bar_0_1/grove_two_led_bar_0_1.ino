#include <Wire.h>
#include <Timer3.h>
#include <RTCTimer.h>
#include <LowPower.h>

LowPower nrgSave;

#define I2C_CMD_JUMP_TO_BOOT	0xf0 //

#define GROVE_TX_PIN_NUM		PA2
#define GROVE_TWO_RX_PIN_NUM	PA3
#define GROVE_LED_PIN_NUM		PA1

#define BAR_DI_PIN_NUM		    PA7
#define BAR_DCK_PIN_NUM		    PA5
#define BAR_DI_PIN_BIT		    0x80
#define BAR_DCK_PIN_BIT		    0x20
#define BAR_GCKI_PIN_NUM        PA4
#define BAR_POWER_DETECT_PIN_NUM PA6
#define BAR_POWER_PIN_NUM       PB1
#define CMD_MODE_0              0x0000


#define AUTO_SLEEP_TIMEOUT	5000

uint32_t autoSleepPreviousMillis = 0;
// bool autoSleepFlag = false;
bool autoSleepFlag = true;

uint32_t intStart = 0;
uint32_t intEnd = 0;


uint8_t commandReceive = 0;
// 用来存放8个led的亮度
// D2 D3 D4 D5 D6 D7 D8 D9
uint8_t led_brightness_buf[8] = {0,};

void setup()
{

  Wire.begin(0x07);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  nrgSave.begin(GROVE_TWO_RX_PIN_NUM, dummy, CHANGE); // The pin need pull up by a resistance
  BarInit();
  pinMode(GROVE_LED_PIN_NUM, OUTPUT);
  Serial.begin(115200);
  Serial.println("after nrgSave");
}

void loop()
{
    blinkOneByOne();
    delay(500);

    if(autoSleepFlag)
    {
        uint32_t autoSleepCurrentMillis = millis();
        if((autoSleepCurrentMillis - autoSleepPreviousMillis) > AUTO_SLEEP_TIMEOUT)
        {
            digitalWrite(GROVE_LED_PIN_NUM, HIGH);
            // disconnect soft i2c of digital light sensor
            pinMode(BAR_DI_PIN_NUM, INPUT);  //DCK
            pinMode(BAR_DCK_PIN_NUM, INPUT);   //DI
            digitalWrite(BAR_POWER_PIN_NUM, LOW);

            Timer3.stop();
            Wire.end();
            // pullup hardware I2C bus
            pinMode(PA9, INPUT_PULLUP);
            pinMode(PA10, INPUT_PULLUP);
            // close RTCTimer
            nrgSave.standby();
            // restart
            Wire.begin(0x07);
            Wire.onReceive(receiveEvent);
            Wire.onRequest(requestEvent);
            pinMode(BAR_DI_PIN_NUM, OUTPUT);  //DCK
            pinMode(BAR_DCK_PIN_NUM, OUTPUT);   //DI
            digitalWrite(BAR_POWER_PIN_NUM, HIGH);
        }
    }

}


void dummy(void)
{
	autoSleepPreviousMillis = millis();
    Serial.println("dummy");

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
  while (Wire.available())commandReceive = Wire.read();
  if (commandReceive == I2C_CMD_JUMP_TO_BOOT)
  {
    Timer3.stop();
    commandReceive = 0;
    clearLED();
    jumpToBootloader();
  }
}

void requestEvent(void)
{

}

void timerIsr()
{
  flashLEDBar();
}

void BarInit()
{
  pinMode(BAR_DI_PIN_NUM, OUTPUT);  //DCK
  pinMode(BAR_DCK_PIN_NUM, OUTPUT);   //DI
  pinMode(BAR_POWER_PIN_NUM, OUTPUT);   //power pin PB1

  digitalWrite(BAR_DI_PIN_NUM, LOW);
  digitalWrite(BAR_DCK_PIN_NUM, LOW);
  digitalWrite(BAR_POWER_PIN_NUM, HIGH);

  clearLED();
  Timer3.init(1000); //1000uS timer
  Timer3.attachInterrupt(timerIsr);
}

// Routine to send 16bit data to MY9221 driver chips
// send MSB first
void send16BitData(uint16_t data)
{
	for (uint8_t i = 0; i < 16; i++)
    {
      GPIOA->BRR = BAR_DI_PIN_BIT;
      GPIOA->BSRR = ((1 && (data & 0x8000)) << 7); //if (data & 0x8000){GPIOA->BSRR = DI_PIN_BIT;}
      GPIOA->ODR ^= BAR_DCK_PIN_BIT;
      data <<= 1;
    }
}

// latch routine for MY9221 data exchange
void latchData(void)
{
  delayMicroseconds(10);
  GPIOA->BRR = BAR_DI_PIN_BIT; //set data pin to low
  for (uint8_t i = 0; i < 8; i++)
  {
    GPIOA->ODR ^= BAR_DI_PIN_BIT;
  }
}


void fullLEDBuf(void)
{
    memset(led_brightness_buf, 0xff, 8);
}

void clearLEDBuf(void)
{
    memset(led_brightness_buf, 0, 8);
}


void clearLED(void)
{
  send16BitData(CMD_MODE_0);
  digitalWrite(BAR_DI_PIN_NUM, LOW); //set data pin to low
  for (uint8_t i = 0; i < 192; i++)
  {
    GPIOA->ODR ^= BAR_DCK_PIN_BIT;
  }
  latchData();
}



void flashLEDBar(void)
{
  clearLED();

  send16BitData(CMD_MODE_0);

  send16BitData(0); // A3
  send16BitData(0); // B3
  send16BitData(0); // C3
  send16BitData(led_brightness_buf[6]); // A2 => D8
  send16BitData(led_brightness_buf[7]); // B2 => D9
  send16BitData(0); // C2
  send16BitData(led_brightness_buf[5]); // D7
  send16BitData(led_brightness_buf[4]); // D6
  send16BitData(led_brightness_buf[3]); // D5
  send16BitData(led_brightness_buf[2]); // D4
  send16BitData(led_brightness_buf[1]); // D3
  send16BitData(led_brightness_buf[0]); // D2

  latchData();
}

void blinkOneByOne(void)
{
    static uint8_t count = 0;
    memset(led_brightness_buf, 0, 8);
    led_brightness_buf[count] = 0xff;
    count++;
    if (count >= 8) count = 0;
}
