

#define TIM_FOR_TIMER            TIM14
#define CHANNEL_FOR_TIM          1
#define IRQn_FOR_TIM             TIM14_IRQn
#define HANDLER_FOR_TIM          TIM14_IRQHandler
#define RCC_PeriphClock_EN_TIM   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE)

typedef enum {
	_timer,
	_Nbr_16timers 
} timer16_Sequence_t ;

