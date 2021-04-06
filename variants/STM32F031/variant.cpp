/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"

#ifdef __cplusplus
extern "C" {
#endif


// Pins descriptions
extern const PinDescription g_APinDescription[]=
{	
	{ GPIOA,  3,  RCC_AHBPeriph_GPIOA, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, GPIO_AF_1 }, // 0  : RX 
	{ GPIOA,  2,  RCC_AHBPeriph_GPIOA, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, GPIO_AF_1 }, // 1  : TX
	
	{ GPIOA,  9,  RCC_AHBPeriph_GPIOA, GPIO_OType_OD, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, GPIO_AF_4 }, // 2  : SCL
	{ GPIOA, 10,  RCC_AHBPeriph_GPIOA, GPIO_OType_OD, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, GPIO_AF_4 }, // 3  : SDA
	
	{ GPIOA,  0,  RCC_AHBPeriph_GPIOA, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz,         0 }, // 4  : ADC 
	{ GPIOA,  1,  RCC_AHBPeriph_GPIOA, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz,         0 }, // 5  : ADC
	
	{ GPIOA, 13,  RCC_AHBPeriph_GPIOA, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz,         0 }, // 6  :  
	{ GPIOA, 14,  RCC_AHBPeriph_GPIOA, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz,         0 }, // 7  :
	{ GPIOF,  0,  RCC_AHBPeriph_GPIOF, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz,         0 }, // 8  :  
	{ GPIOF,  1,  RCC_AHBPeriph_GPIOF, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz,         0 }, // 9  :
	
	{ GPIOA,  4,  RCC_AHBPeriph_GPIOA, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, GPIO_AF_4 }, // 10 : PWM/SS
	{ GPIOA,  5,  RCC_AHBPeriph_GPIOA, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, GPIO_AF_0 }, // 11 : ADC/SCK
	{ GPIOA,  6,  RCC_AHBPeriph_GPIOA, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, GPIO_AF_0 }, // 12 : ADC/MISO
	{ GPIOA,  7,  RCC_AHBPeriph_GPIOA, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, GPIO_AF_0 }, // 13 : ADC/MOSI
	{ GPIOB,  1,  RCC_AHBPeriph_GPIOB, GPIO_OType_PP, GPIO_PuPd_NOPULL, GPIO_Speed_50MHz, GPIO_AF_1 }, // 14 : PWM

	// END
	{ NULL,  0,                 NULL,          NULL,             NULL,            NULL,         0 }
};

// ADC Channel
extern const unsigned int pin_ADC_Channel[]=
{
	NONE,          // 0
	NONE,          // 1
	NONE,          // 2
	NONE,          // 3	
	ADC_Channel_0, // 4
	ADC_Channel_1, // 5
	NONE,          // 6
	NONE,          // 7
	NONE,          // 8
	NONE,          // 9
	NONE,          // 10
	ADC_Channel_5, // 11
	ADC_Channel_6, // 12
	ADC_Channel_7, // 13
	ADC_Channel_9, // 14
	NONE
};

// TIMER PWM
extern const TIM_TypeDef* pin_TIM[]=
{
	NULL,          // 0
	NULL,          // 1
	NULL,          // 2
	NULL,          // 3	
	NULL,          // 4
	NULL,          // 5
	NULL,          // 6
	NULL,          // 7
	NULL,          // 8
	NULL,          // 9
	TIM14,         // 10
	NULL,          // 11
	NULL,          // 12
	NULL,          // 13
	TIM3,          // 14
	NULL
};

extern const uint16_t pin_TIM_Channel[]=
{
	NONE,          // 0
	NONE,          // 1
	NONE,          // 2
	NONE, 		   // 3	
	NONE, 		   // 4
	NONE,          // 5
	NONE,          // 6
	NONE, 		   // 7
	NONE, 		   // 8
	NONE, 		   // 9
	TIM_Channel_1, // 10
	NONE, 		   // 11
	NONE, 		   // 12
	NONE, 		   // 13
	TIM_Channel_4, // 14
	NONE
};

#ifdef __cplusplus
}
#endif


// USART objects
RingBuffer rx_buffer1;
USARTClass Serial(USART1, USART1_IRQn, 1, &rx_buffer1);
void serialEvent() __attribute__((weak));
void serialEvent() { }

// IT handlers
void USART1_IRQHandler(void) 
{
	Serial.IrqHandler();
}

void serialEventRun(void)
{
	if(Serial.available())serialEvent();
}


#ifdef __cplusplus
extern "C" {
#endif

void __libc_init_array(void);

void init( void )
{
	SystemInit();
	
	RCC->CR &= 0xFFFEFFFF;

	// Set Systick to 1ms interval, common to all SAM3 variants
	if(SysTick_Config(SystemCoreClock / 1000))
	{
		// Capture error
		while(true);
	}
	// Configure the SysTick Handler Priority: Preemption priority and subpriority
	NVIC_SetPriority(SysTick_IRQn, 15);	

	// Initialize C library
	__libc_init_array();

	// Initialize Analog Controller
	ADC_InitTypeDef     ADC_InitStructure;
		
	// ADC1 DeInit 
	ADC_DeInit(ADC1);
  
	// ADC1 Periph clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	// Initialize ADC structure
	ADC_StructInit(&ADC_InitStructure);
  
	// Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
	ADC_Init(ADC1, &ADC_InitStructure);

	// ADC Calibration
	ADC_GetCalibrationFactor(ADC1);
  
	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);     
  
	// Wait the ADCEN falg
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN));	
}


#define BOOTLOADER_ADDRESS    0x1FFFEC00

typedef  void (*pFunction)(void);
pFunction Jump_To_Bootloader;

void writeBootFlag(uint8_t flag)
{
	FLASH_Unlock();
	FLASH_OB_Unlock();
	FLASH_OB_RDPConfig(OB_RDP_Level_0);
	if(flag)
	{
		FLASH_OB_ProgramData(0x1FFFF804, 0x42);
		FLASH_OB_ProgramData(0x1FFFF806, 0x4C);
	}
	else
	{
		FLASH_OB_ProgramData(0x1FFFF804, 0xff);
		FLASH_OB_ProgramData(0x1FFFF806, 0xff);
	}
	FLASH_OB_Lock();
	FLASH_Lock(); 
}

uint8_t readBootFlag(void)
{	
	if(*(__IO uint32_t *)(0x1FFFF804) == 0xB34CBD42)return true;
	return false;
}

void jumpToBootloader(void)
{
	writeBootFlag(true); // BOOT
	FLASH_OB_Launch();
}

void bootloader(void)
{
	if(readBootFlag()) // BOOT
	{
		writeBootFlag(false);
		
		uint32_t JumpAddress;
		
		// Remap SRAM at 0x00000000
		SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_Flash);

		// Jump to user application
		JumpAddress = *(__IO uint32_t*) (BOOTLOADER_ADDRESS + 4);
		Jump_To_Bootloader = (pFunction) JumpAddress;

		// Initialize user application's Stack Pointer
		__set_MSP(*(__IO uint32_t*) BOOTLOADER_ADDRESS);

		// Jump to application
		Jump_To_Bootloader();
	}
	
}

#ifdef __cplusplus
}
#endif
