#include <stdio.h>
#include <stdbool.h>

#include "Board_LED.h"                  // ::Board Support:LED
#include "Board_Buttons.h"              // ::Board Support:Buttons

#include "stm32f4xx.h"                  // Device header

#include 	"misc.h"
#include 	"stm32f4xx_tim.h"

void SQ_TIM3_Init(void);


extern int stdout_init (void);

volatile uint32_t msTicks;                            /* counts 1ms timeTicks */
/*----------------------------------------------------------------------------
 * SysTick_Handler:
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;
}

/*----------------------------------------------------------------------------
 * Delay: delays a number of Systicks
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) { __NOP(); }
}

/*----------------------------------------------------------------------------
 * SystemCoreClockConfigure: configure SystemCoreClock using HSI
                             (HSE is not populated on Nucleo board)
 *----------------------------------------------------------------------------*/
void SystemCoreClockConfigure(void) {

  RCC->CR |= ((uint32_t)RCC_CR_HSION);                     /* Enable HSI */
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                  /* Wait for HSI Ready */

  RCC->CFGR = RCC_CFGR_SW_HSI;                             /* HSI is system clock */
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  /* Wait for HSI used as system clock */

  FLASH->ACR  = (FLASH_ACR_PRFTEN     |                    /* Enable Prefetch Buffer */
                 FLASH_ACR_ICEN       |                    /* Instruction cache enable */
                 FLASH_ACR_DCEN       |                    /* Data cache enable */
                 FLASH_ACR_LATENCY_5WS );                  /* Flash 5 wait state */

  RCC->CFGR |= (RCC_CFGR_HPRE_DIV1  |                      /* HCLK = SYSCLK */
                RCC_CFGR_PPRE1_DIV2 |                      /* APB1 = HCLK/2 */
                RCC_CFGR_PPRE2_DIV1  );                    /* APB2 = HCLK/1 */

  RCC->CR &= ~RCC_CR_PLLON;                                /* Disable PLL */

  /* PLL configuration:  VCO = HSI/M * N,  Sysclk = VCO/P */		//	Configure for SYSCLK of 100 MHz
  RCC->PLLCFGR = ( 16ul                   |                /* PLL_M =  16 */
                 (200ul <<  6)            |                /* PLL_N = 200 */
                 (  0ul << 16)            |                /* PLL_P =   2 */
                 (RCC_PLLCFGR_PLLSRC_HSI) |                /* PLL_SRC = HSI */
                 (  7ul << 24)            |                /* PLL_Q =   7 */
                 (  2ul << 28)             );              /* PLL_R =   2 */

  RCC->CR |= RCC_CR_PLLON;                                 /* Enable PLL */
  while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           /* Wait till PLL is ready */

  RCC->CFGR &= ~RCC_CFGR_SW;                               /* Select PLL as system clock source */
  RCC->CFGR |=  RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  /* Wait till PLL is system clock src */
}


volatile	bool	Print_Flag	=	false;

/*----------------------------------------------------------------------------
 * main: blink LED and check button state
 *----------------------------------------------------------------------------*/
int main (void) {
  int32_t max_num = LED_GetCount();
  int32_t num = 0;

  SystemCoreClockConfigure();                              /* configure HSI as System Clock */
  SystemCoreClockUpdate();

  LED_Initialize();
  Buttons_Initialize();
  stdout_init();                                           /* Initializ Serial interface */
printf ("Hello World\n\r");

  SysTick_Config(SystemCoreClock / 1000);                  /* SysTick 1 msec interrupts */

	SQ_TIM3_Init();
	
  for (;;) {
    LED_On(num);                                           /* Turn specified LED on */
    Delay(500);                                            /* Wait 500ms */
    while (Buttons_GetState() & (1 << 0));                 /* Wait while holding USER button */
    LED_Off(num);                                          /* Turn specified LED off */
    Delay(500);                                            /* Wait 500ms */
    while (Buttons_GetState() & (1 << 0));                 /* Wait while holding USER button */

    num++;                                                 /* Change LED number */
    if (num >= max_num) {
      num = 0;                                             /* Restart with first LED */
    }

		printf ("Hello All!\n\r");

		if(Print_Flag)
		{
			Print_Flag	=	false;
			printf ("Hello Flag\n\r");
		}
		else
		{
			//	...
		}
  }
}


void SQ_TIM3_Init(void) 
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_CAN1, ENABLE);
	
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  // Time Base Configuration
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  // TIM3 generates a 1 Msec tick for a 1 mSec ISR.
#if		001
  // 100MHz clock / 2 = 50MHz.  50MHz / 5 (prescaler) = 10MHz.  10MHz / 10000 = 1KHz (1mS).
  TIM_TimeBaseStructure.TIM_Period 		= (100000-1);
  TIM_TimeBaseStructure.TIM_Prescaler = (5-1);
#else
  // 180MHz clock / 2 = 90MHz.  90MHz / 9 (prescaler) = 10MHz.  10MHz / 10000 = 1KHz (1mS).
  TIM_TimeBaseStructure.TIM_Period 		= (10000-1);
  TIM_TimeBaseStructure.TIM_Prescaler = (9-1);
#endif

  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

	// Timer 3 Interrupt Handling
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

		NVIC_Init(&NVIC_InitStructure);
	}

	/* Enable Timer 3 Update Event Interrupt */
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	TIM_Cmd(TIM3, ENABLE);
}


void	XX_mSec_Tasks(void);
uint16_t	Print_Counter	=	5000;

void TIM3_IRQHandler(void) 
{
	// Clear the interrupt pending flag
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	
	if(Print_Counter)
	{
		Print_Counter--;
	}
	else
	{
		Print_Counter	=	5000;
		Print_Flag	=	true;
		XX_mSec_Tasks();
	}
}



volatile	uint16_t	State	=	0;

void	XX_mSec_Tasks(void)
{
	if(State	==	0)
	{
		State	=	1;
		LED_On(0);
	}
	else
	{
		State	=	0;
		LED_Off(0);
	}
}



