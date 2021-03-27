
#include "LowPower.h"


void LowPower::begin(unsigned int inter_pin, voidFuncPtr callback, unsigned int pin_mode)
{	
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC |
							RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF , ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA |RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC |
							RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF, DISABLE);
	
	attachInterrupt(inter_pin, callback, pin_mode, INPUT_PULLUP);	
}

void LowPower::standby(bool flag)
{		
	ADC_Cmd(ADC1, DISABLE);   
	
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
    
    // After wake-up from STOP reconfigure the system clock
    if(flag)
    {
        // Enable HSE
        RCC_HSEConfig(RCC_HSE_ON);
        // Wait till HSE is ready
        while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET){}
    }
    else
    {
        // Enable HSI
        RCC_HSICmd(ENABLE);
        // Wait till HSI is ready
        while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET){}
    }

	// Enable PLL
	RCC_PLLCmd(ENABLE);
	// Wait till PLL is ready
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}
    
	// Select PLL as system clock source
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	// Wait till PLL is used as system clock source
	while(RCC_GetSYSCLKSource() != 0x08){}
	// Enable ADC
	ADC_Cmd(ADC1, ENABLE); 
}
