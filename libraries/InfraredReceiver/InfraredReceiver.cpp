/*
 * IRremote
 * Version 0.11 August, 2009
 * Copyright 2009 Ken Shirriff
 * For details, see http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
 *
 * Modified by Paul Stoffregen <paul@pjrc.com> to support other boards and timers
 * Modified  by Mitra Ardron <mitra@mitra.biz> 
 * Added Sanyo and Mitsubishi controllers
 * Modified Sony to spot the repeat codes that some Sony's send
 * 
 * Modifier by
 * Interrupt code based on NECIRrcv by Joe Knapp
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
 * Also influenced by http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
 *
 * JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
 */


#include "InfraredReceiver.h"


static unsigned long codeTable[21] = 
{
    0x00ff6897, // 0 - '0'
    0x00ff30cf, // 1 - '1'
    0x00ff18e7, // 2 - '2'
    0x00ff7a85, // 3 - '3'
    0x00ff10ef, // 4 - '4'
    0x00ff38c7, // 5 - '5'
    0x00ff5aa5, // 6 - '6'
    0x00ff42bd, // 7 - '7'
    0x00ff4ab5, // 8 - '8'
    0x00ff52ad, // 9 - '9'
    0x00ffa25d, // 10 - 'Power'
    0x00ff629d, // 11 - 'Mode'
    0x00ffe21d, // 12 - 'Speaker'
    0x00ff22dd, // 13 - 'Play/Pause'
    0x00ff02fd, // 14 - 'Fast backward'
    0x00ffc23d, // 15 - 'Fast forward'
    0x00ffe01f, // 16 - 'EQ'
    0x00ffa857, // 17 - 'VOL-'
    0x00ff906f, // 18 - 'VOL+'
    0x00ff9867, // 19 - 'RPT'
    0x00ffb04f, // 20 - 'U/SD'
};

irparams_t irparams[MAX_IRPARAMS];
uint8_t IRCount = 0; 


InfraredReceiver::InfraredReceiver(int revPin)
{
    if(IRCount < MAX_IRPARAMS)
    {
        IRIndex = IRCount ++;
        _pin = revPin;
    }
}

void InfraredReceiver::timerInit(void)
{
	uint16_t PrescalerValue = 0;
    
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	
  	/* Compute the prescaler value */
  	PrescalerValue = (uint16_t) (SystemCoreClock  / 1000000) - 1;

  	/* Time base configuration */
  	TIM_TimeBaseStructure.TIM_Period = 50; // 50 us
  	TIM_TimeBaseStructure.TIM_Prescaler = 0;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

  	/* Prescaler configuration */
  	TIM_PrescalerConfig(TIM14, PrescalerValue, TIM_PSCReloadMode_Immediate);

  	/* Output Compare Timing Mode configuration: Channel1 */
  	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  	/* Output Compare Timing Mode configuration: Channel3 */
  	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  	TIM_OCInitStructure.TIM_Pulse = 230;

  	TIM_OC3Init(TIM14, &TIM_OCInitStructure);

  	TIM_OC3PreloadConfig(TIM14, TIM_OCPreload_Disable);
   
  	/* TIM Interrupts enable */
  	TIM_ITConfig(TIM14, TIM_IT_CC1, ENABLE);

  	/* TIMx enable counter */
  	TIM_Cmd(TIM14, ENABLE);
}

void InfraredReceiver::Init(void)
{
    irparams[IRIndex].recvpin = _pin;
    
    enableIRIn(); // Start the receiver
    delay(20);
    Clear();
}

// initialization
void InfraredReceiver::enableIRIn()
{
	timerInit();

	// initialize state machine variables
	irparams[IRIndex].rcvstate = STATE_IDLE;
	irparams[IRIndex].rawlen = 0;
	
	// set pin modes
	pinMode(irparams[IRIndex].recvpin, INPUT);
}

void InfraredReceiver::Clear()
{
	irparams[IRIndex].rcvstate = STATE_IDLE;
	irparams[IRIndex].rawlen = 0;
}

// Decodes the received IR message
// Returns 0 if no data ready, 1 if data ready.
// Results of decoding are stored in results
int InfraredReceiver::decode(decode_results *results)
{
	results->rawbuf = irparams[IRIndex].rawbuf;
	results->rawlen = irparams[IRIndex].rawlen;
    
	if(irparams[IRIndex].rcvstate != STATE_STOP)
	{
		return ERR;
	}
    
	// Throw away and start over
	Clear();
    
	return 1;
}

unsigned char *InfraredReceiver::InfraredReceiverRead(void)
{
	if(IsData() == 0)return NULL;
	
    int count       = results.rawlen;
    int nshort      = 0;
    int nlong       = 0;
    int count_data  = 0;
	
	memset(revData, 0, sizeof(revData));	

    count_data = (count-4)/16;

    for(int i = 0; i<10; i++)           // count nshort
    {
        nshort += results.rawbuf[3+2*i];
    }
    nshort /= 10;

    int i = 0;
    int j = 0;
    
    while(1)        // count nlong
    {
        if(results.rawbuf[4+2*i] > (2*nshort))
        {
            nlong += results.rawbuf[4+2*i];
            j++;
        }
        i++;
        if(j==10)break;
        if((4+2*i)>(count-10))break;
    }
    nlong /= j;

    int doubleshort = 2*nshort;
    
    for(i = 0; i<count_data; i++)
    {
        revData[i+D_DATA] = 0x00;
        for(j = 0; j<8; j++)
        {
            if(results.rawbuf[4 + 16*i + j*2] > doubleshort) // 1
            {
                revData[i+D_DATA] |= 0x01<< (7-j);
            }
            else
            {
                revData[i+D_DATA] &= ~(0x01<<(7-j));
            }
        }
    }
    
    revData[D_LEN]      = count_data+5;
    revData[D_STARTH]   = results.rawbuf[1];
    revData[D_STARTL]   = results.rawbuf[2];
    revData[D_SHORT]    = nshort;
    revData[D_LONG]     = nlong;
    revData[D_DATALEN]  = count_data;
 
#if __DEBUG
    Serial.print("\r\n*************************************************************\r\n");
    Serial.print("len\t = ");Serial.println(revData[D_LEN]);
    Serial.print("start_h\t = ");Serial.println(revData[D_STARTH]);
    Serial.print("start_l\t = ");Serial.println(revData[D_STARTL]);
    Serial.print("short\t = ");Serial.println(revData[D_SHORT]);
    Serial.print("long\t = ");Serial.println(revData[D_LONG]);
    Serial.print("data_len = ");Serial.println(revData[D_DATALEN]);
    
    for(int i = 0; i<revData[D_DATALEN]; i++)
    {
        Serial.print(revData[D_DATA+i]);Serial.print("\t");
    }
    Serial.print("\r\n*************************************************************\r\n");
#endif

    Clear(); // Receive the next value
	
	return (revData + 5);
}

int InfraredReceiver::InfraredReceiverReadCode(void)
{
    char i;
    unsigned long codeTemp;
    unsigned char *data = InfraredReceiverRead();
    
    if(data) // get IR data
    {	
        int length = data[0];
        if(length == 4)
        {
            codeTemp = data[1];
            codeTemp <<= 8;
            codeTemp += data[2];
            codeTemp <<= 8;
            codeTemp += data[3];
            codeTemp <<= 8;
            codeTemp += data[4];
            
            for(i = 0; i < 21; i ++)
            {
                if(codeTemp == codeTable[i])return i;    
            }
            if(i >= 21)return -1;
        }
        else return -1;
    }
    else return -1;
}

//if get some data from IR
unsigned char InfraredReceiver::IsData()
{
    if(decode(&results))
    {
        int count = results.rawlen;
        
        if(count < 20 || (count -4) % 8 != 0)
        {
#if __DEBUG
            Serial.print("IR GET BAD DATA!\r\n");
#endif
            Clear();        // Receive the next value
            return 0;
        }
        int count_data = (count-4) / 16;
#if __DEBUG
        Serial.print("ir get data! count_data = ");
        Serial.println(count_data);
#endif
        return (unsigned char)(count_data + 6);
    }
    else 
    {
        return 0;
    }
}

// TIMER2 interrupt code to collect raw data.
// Widths of alternating SPACE, MARK are recorded in rawbuf.
// Recorded in ticks of 50 microseconds.
// rawlen counts the number of entries recorded so far.
// First entry is the SPACE between transmissions.
// As soon as a SPACE gets long, ready is set, state switches to IDLE, timing of SPACE continues.
// As soon as first MARK arrives, gap width is recorded, ready is cleared, and new logging starts

void TIM14_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);
	
    for(uint8_t i = 0; i < IRCount; i ++)
    {
        uint8_t irdata = (uint8_t)digitalRead(irparams[i].recvpin);

        irparams[i].timer++; // One more 50us tick
        
        if(irparams[i].rawlen >= RAWBUF) // Buffer overflow
        {
            irparams[i].rcvstate = STATE_STOP;
        }
        
        switch(irparams[i].rcvstate)
        {
            case STATE_IDLE: // In the middle of a gap
                if (irdata == MARK)
                {
                    if (irparams[i].timer < GAP_TICKS) // Not big enough to be a gap.
                    {
                        irparams[i].timer = 0;
                    } 
                    else // gap just ended, record duration and start recording transmission
                    {
                        irparams[i].rawlen = 0;
                        irparams[i].rawbuf[irparams[i].rawlen++] = irparams[i].timer;
                        irparams[i].timer = 0;
                        irparams[i].rcvstate = STATE_MARK;
                    }
                }
            break;
            
            case STATE_MARK: // timing MARK
                if(irdata == SPACE) // MARK ended, record time
                {   
                    irparams[i].rawbuf[irparams[i].rawlen++] = irparams[i].timer;
                    irparams[i].timer = 0;
                    irparams[i].rcvstate = STATE_SPACE;
                }
            break;
            
            case STATE_SPACE: // timing SPACE
                if(irdata == MARK) // SPACE just ended, record it
                {
                    irparams[i].rawbuf[irparams[i].rawlen++] = irparams[i].timer;
                    irparams[i].timer = 0;
                    irparams[i].rcvstate = STATE_MARK;
                }
                else // SPACE
                { 
                    if(irparams[i].timer > GAP_TICKS)
                    {
                        // big SPACE, indicates gap between codes
                        // Mark current code as ready for processing
                        // Switch to STOP
                        // Don't reset timer; keep counting space width
                        irparams[i].rcvstate = STATE_STOP;
                    }
                }
            break;
            
            case STATE_STOP: // waiting, measuring gap
                if(irdata == MARK) // reset gap timer
                {
                    irparams[i].timer = 0;
                }
            break;
        }
    }
}
