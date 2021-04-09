
#include "Timer3.h"


void Timer3Class::init(uint16_t microseconds)
{
	TIM_DeInit(TIM3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	setPeriod(microseconds);
}

void Timer3Class::setPeriod(uint16_t microseconds)
{
	if(microseconds < 1)microseconds = 1;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = microseconds * 48;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM3, 0, TIM_PSCReloadMode_Immediate);
}

void Timer3Class::start()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}
void Timer3Class::stop()
{
	TIM_Cmd(TIM3, DISABLE);
}

void Timer3Class::attachInterrupt(void (*isr)())
{
	isrCallback = isr;
	start();
}

void Timer3Class::detachInterrupt()
{
	stop();
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM3_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	Timer3.isrCallback();
}


Timer3Class Timer3;
