
#include "WatchDog.h"


void WWDGClass::begin(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

	WWDG_SetPrescaler(WWDG_Prescaler_8);

	WWDG_Enable(127);
}

void WWDGClass::reset(void)
{
    WWDG_SetCounter(127);
}

void WWDGClass::end(void)
{
	WWDG_DeInit();
}


WWDGClass wwdg;
