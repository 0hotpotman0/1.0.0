
#include "InfraredEmitter.h"


static unsigned long codeTableA[21] = 
{
    0x00ffa25d, // 0 - 'Power'
    0x00ff629d, // 1 - 'Mode'
    0x00ffe21d, // 2 - 'Speaker'
    0x00ff22dd, // 3 - 'Play/Pause'
    0x00ff02fd, // 4 - 'Fast backward'
    0x00ffc23d, // 5 - 'Fast forward'
    0x00ffe01f, // 6 - 'EQ'
    0x00ffa857, // 7 - 'VOL-'
    0x00ff906f, // 8 - 'VOL+'
    0x00ff6897, // 9 - '0'
    0x00ff9867, // 10 - 'RPT'
    0x00ffb04f, // 11 - 'U/SD'
    0x00ff30cf, // 12 - '1'
    0x00ff18e7, // 13 - '2'
    0x00ff7a85, // 14 - '3'
    0x00ff10ef, // 15 - '4'
    0x00ff38c7, // 16 - '5'
    0x00ff5aa5, // 17 - '6'
    0x00ff42bd, // 18 - '7'
    0x00ff4ab5, // 19 - '8'
    0x00ff52ad, // 20 - '9'
};

static unsigned long codeTableB[24] = 
{
    0x00f700ff, // light up
    0x00f7807f, // light down
    0x00f740bf, // OFF
    0x00f7c03f, // ON
    
    0x00f720df, // R
    0x00f7a05f, // G
    0x00f7609f, // B
    0x00f7e01f, // W
    
    0x00f710ef, // 
    0x00f7906f, // 
    0x00f750af, //
    0x00f7d02f, // FLASH
    
    0x00f730cf, // 
    0x00f7b04f, //
    0x00f7708f, //
    0x00f7f00f, // STROBE
    
    0x00f708f7, //
    0x00f78877, //
    0x00f748b7, //
    0x00f7c837, // FADE
    
    0x00f728d7, //
    0x00f7a857, //
    0x00f76897, //
    0x00f7e817, // SMOOTH
};

void delayUs(uint32_t usec)
{
    if (usec == 0) return;
    uint32_t n = usec * (VARIANT_MCK / 8000000);
    asm volatile(
        "L_%=_delayMicroseconds:"       "\n\t"
        "sub    %0, #1"                 "\n\t"
        "bne    L_%=_delayMicroseconds" "\n"
        : "+r" (n) :
    );
}

InfraredEmitter::InfraredEmitter(int senPin)
{
	_pin = senPin;
}

void InfraredEmitter::Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    
    if(_pin == PA6)
    {
        GPIO_PinAFConfig(GPIOA, 6, GPIO_AF_1); // // PA6 -> TIM3 -> CH1
        
        GPIO_InitStructure.GPIO_Pin = (1 << 6);
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
    else if(_pin == PA7)
    {
        GPIO_PinAFConfig(GPIOA, 7, GPIO_AF_1); // PA7 -> TIM3 -> CH2
        
        GPIO_InitStructure.GPIO_Pin = (1 << 7);
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    uint16_t timerARP = (uint16_t)(1000000 / 38000) - 1;
    
    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = timerARP;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
    // TIM_OCInitStructure.TIM_Pulse = 13; // 50% duty
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    
    if(_pin == PA6)
    {
        // PWM1 Mode configuration: Channel1
        TIM_OC1Init(TIM3, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    }
    else if(_pin == PA7)
    {
        // PWM2 Mode configuration: Channel1
        TIM_OC2Init(TIM3, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    }

    TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	// TIM3 counter enable
	TIM_Cmd(TIM3, ENABLE);

	// TIM3 Main Output Enable
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
}

void InfraredEmitter::InfraredEmitterWriteCodeA(int code)
{
    unsigned char dtaSend[4] = {0};
    
    if(code >= 21)return;
#if __DEBUG    
    Serial.print("code: ");
    Serial.println(codeTableA[code], HEX);
#endif    
    dtaSend[0] = (codeTableA[code] >> 24) & 0xff;
    dtaSend[1] = (codeTableA[code] >> 16) & 0xff;
    dtaSend[2] = (codeTableA[code] >> 8) & 0xff;
    dtaSend[3] = (codeTableA[code]) & 0xff;
#if __DEBUG    
    Serial.println(dtaSend[0]);
    Serial.println(dtaSend[1]);
    Serial.println(dtaSend[2]);
    Serial.println(dtaSend[3]);
    Serial.println();
#endif     
    InfraredEmitterWrite(dtaSend, 4);
}

void InfraredEmitter::InfraredEmitterWriteCodeB(int code)
{
    unsigned char dtaSend[4] = {0};
    
    if(code >= 24)return;
#if __DEBUG    
    Serial.print("code: ");
    Serial.println(codeTableB[code], HEX);
#endif    
    dtaSend[0] = (codeTableB[code] >> 24) & 0xff;
    dtaSend[1] = (codeTableB[code] >> 16) & 0xff;
    dtaSend[2] = (codeTableB[code] >> 8) & 0xff;
    dtaSend[3] = (codeTableB[code]) & 0xff;
#if __DEBUG    
    Serial.println(dtaSend[0]);
    Serial.println(dtaSend[1]);
    Serial.println(dtaSend[2]);
    Serial.println(dtaSend[3]);
    Serial.println();
#endif     
    InfraredEmitterWrite(dtaSend, 4);
}

void InfraredEmitter::InfraredEmitterWrite(unsigned char *idata, unsigned char num)
{
	unsigned char datalen   = num;
    unsigned int start_high = 179;
    unsigned int start_low  = 90;
    unsigned int nshort     = 11;
    unsigned int nlong      = 33;
    
    unsigned int *pSt = (unsigned int *)malloc((4 + datalen * 16) * sizeof(unsigned int));

    if(NULL == pSt)
    {
#if __DEBUG
        Serial.println("not enough place!!\r\n");
#endif
        exit(1);
    }

    pSt[0] = start_high * 50;
    pSt[1] = start_low * 50;

    for(int i = 0; i < datalen; i ++)
    {
        for(int j = 0; j < 8; j ++)
        {
            if(idata[i] & 0x01<<(7 - j))
            {
                pSt[16 * i + 2 * j + 2] = nshort * 50;
                pSt[16 * i + 2 * j + 3] = nlong * 50;
            }
            else
            {
                pSt[16 * i + 2 * j + 2] = nshort * 50;
                pSt[16 * i + 2 * j + 3] = nshort * 50;
            }
        }
    }

    pSt[2 + datalen * 16] = nshort * 50;
    pSt[2 + datalen * 16 + 1] = nshort * 50;

#if __DEBUG
    for(int i = 0; i < 4 + datalen * 16; i ++)
    {
        Serial.print("pSt is ");
        Serial.print(pSt[i]);
        Serial.print("\t");
    }
    Serial.println();
#endif
    
    sendRaw(pSt, 4 + datalen * 16);
    free(pSt);
}

void InfraredEmitter::sendRaw(unsigned int buf[], int len)
{
	enableIROut();

	for(int i = 0; i < len; i++)
	{
		if(i & 1)space(buf[i]);
		else mark(buf[i]);
	}
	
	space(0); // Just to be sure
}

void InfraredEmitter::mark(int time)
{
	// Sends an IR mark for the specified number of microseconds.
	// The mark output is modulated at the PWM frequency.
	if(_pin == PA6)TIM_SetCompare1(TIM3, 13);
	else if(_pin == PA7)TIM_SetCompare2(TIM3, 13);
	delayUs(time);
}

/* Leave pin off for time (given in microseconds) */
void InfraredEmitter::space(int time)
{
	// Sends an IR space for the specified number of microseconds.
	// A space is no output, so the PWM output is disabled.
	if(_pin == PA6)TIM_SetCompare1(TIM3, 0);
	else if(_pin == PA7)TIM_SetCompare2(TIM3, 0);
	delayUs(time);
}

void InfraredEmitter::enableIROut(void)
{
	// Enables IR output.  The khz value controls the modulation frequency in kilohertz.
	// The IR output will be on pin 3 (OC2B).
	// This routine is designed for 36-40KHz; if you use it for other values, it's up to you
	// to make sure it gives reasonable results.  (Watch out for overflow / underflow / rounding.)
	// TIMER2 is used in phase-correct PWM mode, with OCR2A controlling the frequency and OCR2B
	// controlling the duty cycle.
	// There is no prescaling, so the output frequency is 16MHz / (2 * OCR2A)
	// To turn the output on and off, we leave the PWM running, but connect and disconnect the output pin.
	// A few hours staring at the ATmega documentation and this will all make sense.
	// See my Secrets of Arduino PWM at http://arcfn.com/2009/07/secrets-of-arduino-pwm.html for details.

	// Disable the Interrupt (which is used for receiving IR)
    
    // pinMode(_pin, OUTPUT);
    // digitalWrite(_pin, LOW);
    
	if(_pin == PA6)TIM_SetCompare1(TIM3, 13);
	else if(_pin == PA7)TIM_SetCompare2(TIM3, 13);
}
