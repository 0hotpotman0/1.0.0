
#include "RTCTimer.h"


void RTCTimerClass::begin(uint16_t times, void(*function)(void), bool clockRenew)
{
	RTC_InitTypeDef  RTC_InitStructure;
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_AlarmTypeDef RTC_AlarmStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	if(times <= 0)times = 1; // Min time is 1 ms
	else if(times > 8000)times = 8000; // Max time is 8000 ms
	
	clockRenewCmd = clockRenew;
	
	/* Enable the PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Allow access to RTC */
	PWR_BackupAccessCmd(ENABLE);

	/* Reset back up registers */
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);

	/* Enable the LSI OSC */
	RCC_LSICmd(ENABLE);

	/* Wait till LSE is ready */  
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();

	/* Configure the RTC data register and RTC prescaler */
	/* CLK = RTCCLK(LSI) /(AsynchPrediv + 1)*(SynchPrediv + 1)*/
	RTC_InitStructure.RTC_AsynchPrediv = 9;
	RTC_InitStructure.RTC_SynchPrediv  = (times << 2) - 1; // x = (40000 * t / 1000 - 10) / 10 = 4 * t - 1
	RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;
	RTC_Init(&RTC_InitStructure);

	/* Set the time to 00h 00mn 00s AM */
	RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
	RTC_TimeStructure.RTC_Hours   = 0x00;
	RTC_TimeStructure.RTC_Minutes = 0x00;
	RTC_TimeStructure.RTC_Seconds = 0x00;  
	RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);

	/* EXTI configuration */
	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable the RTC Alarm Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = 0x00;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = 0x00;
	RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = 0x01;
	RTC_AlarmStructure.RTC_AlarmDateWeekDay = 0x31;
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;
	RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);

	/* Enable AlarmA interrupt */
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);

	/* Enable the alarm */
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
	
	rtcCallback = function;
}

void RTCTimerClass::setNewPeriod(uint16_t times)
{
	RTC_InitTypeDef  RTC_InitStructure;
	
	if(times <= 0)times = 1; // Min time is 1 ms
	else if(times > 8000)times = 8000; // Max time is 8000 ms

	/* Disable the alarm */
	RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
	
	/* Configure the RTC data register and RTC prescaler */
	/* CLK = RTCCLK(LSI) /(AsynchPrediv + 1)*(SynchPrediv + 1)*/
	RTC_InitStructure.RTC_AsynchPrediv = 9;
	RTC_InitStructure.RTC_SynchPrediv  = (times << 2) - 1; // x = (40000 * t / 1000 - 10) / 10 = 4 * t - 1
	RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;
	RTC_Init(&RTC_InitStructure);

	/* Enable the alarm */
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
}

void RTCTimerClass::end(void)
{
	RTC_DeInit();
	rtcCallback = NULL;
}


RTCTimerClass RTCTimer;


void RTC_IRQHandler(void)
{
	if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
	{
		RTC_TimeTypeDef RTC_TimeStructure;

		if(RTCTimer.clockRenewCmd)
		{
			// After wake-up from STOP reconfigure the system clock
			// Enable HSI
			RCC_HSICmd(ENABLE);
			// Wait till HSI is ready
			while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)	{}
			// Enable PLL
			RCC_PLLCmd(ENABLE);
			// Wait till PLL is ready
			while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)	{}
			// Select PLL as system clock source
			RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
			// Wait till PLL is used as system clock source
			while (RCC_GetSYSCLKSource() != 0x08) {}
			// Enable ADC
			ADC_Cmd(ADC1, ENABLE); 	
		}

		// Set the time to 00h 00mn 00s AM
		RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
		RTC_TimeStructure.RTC_Hours   = 0x00;
		RTC_TimeStructure.RTC_Minutes = 0x00;
		RTC_TimeStructure.RTC_Seconds = 0x00;  
		RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);

		if(RTCTimer.rtcCallback)RTCTimer.rtcCallback();
		
		RTC_ClearITPendingBit(RTC_IT_ALRA);
		EXTI_ClearITPendingBit(EXTI_Line17);
	} 
}
