
#include "Timer14.h"


void Timer14Class::init(uint16_t microseconds)
{
	TIM_DeInit(TIM14);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	setPeriod(microseconds);
}

void Timer14Class::setPeriod(uint16_t microseconds)
{
	if(microseconds < 1)microseconds = 1;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = microseconds;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM14, 47, TIM_PSCReloadMode_Immediate);
}

void Timer14Class::start()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM14, TIM_IT_CC1, ENABLE);
	TIM_Cmd(TIM14, ENABLE);
}
void Timer14Class::stop()
{
	TIM_Cmd(TIM14, DISABLE);
}

void Timer14Class::attachInterrupt(void (*isr)())
{
	isrCallback = isr;
	start();
}

void Timer14Class::detachInterrupt()
{
	stop();
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM14_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);
	Timer14.isrCallback();
}


Timer14Class Timer14;
