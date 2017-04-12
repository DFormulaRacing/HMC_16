#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <iso646.h>

#include "Board_LED.h"                  // ::Board Support:LED
#include "Board_Buttons.h"              // ::Board Support:Buttons

#include "stm32f4xx.h"                  // Device header

#include 	"misc.h"
#include 	"stm32f4xx_tim.h"
//#include	"stm32f4xx_dma.h"
//#include	"stm32f4xx_usart.h"
//#include 	"stm32f4xx_adc.h"
#include 	"stm32f4xx_can.h"
#include 	"stm32f4xx_gpio.h"
//#include 	"stm32f4xx_flash.h"
//#include 	"stm32f4xx_rcc.h"
//#include 	"stm32f4xx_tim.h"
#include "HMC_state_machine.h"

#include <RTE_components.h>

// a small helper... 
#define	until(arg)	while(!(arg))
#define ENABLE_CAN_INTERRUPTS (1)

volatile	uint32_t	JLM_Debug	=	0;

// Setting Up Message Table
const uint32_t STD = CAN_Id_Standard;
const uint32_t EXT = CAN_Id_Extended;
 uint32_t msgTableSize = 20;

uint32_t shaun_debug;

//#pragma pack

enum HMC_states{
	off,
	start,
	init,
	brake,
	clutch,
	prop_torqe,
}; 

enum HMC_states state = off;
float engine_torque;
float motor_torque;
float kP;

// Structures for BAMOCAR data
typedef struct Bamo_data_16_s{
	uint8_t REGID; // change back to uint8_t
	uint16_t data;
} Bamo_data_16_t;

/* typedef struct Bamo_data_32_s {
	uint8_t REGID;
	uint32_t data;
} Bamo_data_32_t; */

// union for CAN Data
typedef union CAN_data_u {
	uint64_t   _64;
	uint8_t    _8[8];
  uint16_t   _16[4];
  uint32_t   _32[2];
  float      _float[2];
  double     _double;

	Bamo_data_16_t Bamo_data_16;

	//Bamo_data_32_t Bamo_data_32;

} CAN_data_t;

// create structure for message tble
typedef struct CAN_msg_s {
	uint32_t messageID;
	uint32_t messageType;
	uint32_t REGID;
 	CAN_data_t data;
	uint32_t update;
	int row;
	int col;
	char name[10];
	int msg_count;
	int old_count;

} CAN_msg_t;


input_vector_t input_vector;

const int column = 12;
	// create message table
volatile CAN_msg_t msgTable[] =
{	
	// PE3 messages
	{0x0CFFF048, EXT, 0, 0, 1, 1, column, "PE3 1",0,0}, //PE3 1
	{0x0CFFF148, EXT, 0, 0, 1, 2, column, "PE3 2",0,0}, //PE3 2
	{0x0CFFF548, EXT, 0, 0, 1, 3, column, "PE3 6",0,0}, //PE3 6
	{0x0CFFFB48, EXT, 0, 0, 1, 4, column, "PE3 12",0,0}, //PE3 12

	// Orion messages
	{0x03B, STD, 0, 0, 1, 5, column, "BMS1",0,0}, // BMS 1
	{0x3CB, STD, 0, 0, 1, 6, column, "BMS2",0,0}, // BMS 2
	{0x6B2, STD, 0, 0, 1, 7, column, "BMS3",0,0}, // BMS 3
	{0x623, STD, 0, 0, 1, 8, column, "BMS4",0,0}, // BMS 4
	{0x190, STD, 0, 0, 1, 9, column, "BMS5",0,0}, // BMS 5
	//{0x00, STD, 0} // BMS 6

	// BAMOCAR messages
	{0x180, STD, 0x30, 0, 1,10, column, "BAMO1",0,0}, // BAMOCAR 1 - RPM
	{0x180, STD, 0x20, 0, 1,11, column, "BAMO2",0,0}, // BAMOCAR 2 - Motor Current
	{0x180, STD, 0xA0, 0, 1,12, column, "BAMO3",0,0}, // BAMOCAR 3 - Motor Torque
	{0x180, STD, 0x84, 0, 1,13, column, "BAMO4",0,0}, // BAMOCAR 4 - Motor Fault
	{0x180, STD, 0x49, 0, 1,14, column, "BAMO5",0,0}, // BAMOCAR 5 - Motor Temp

	// NOTE: Bamocar transmits on ONE CAN message ID
	// The REGID data field signifies what kind of
	// info it holds. This will be delt in parsing...

};

// Ring Buffer Stuff

const int BUFFER_SIZE = 100;
CanRxMsg buffer[BUFFER_SIZE] = {0} ;
volatile int readIdx = 0;
volatile int writeIdx = 0;
volatile int ringCounter = 0;


void addToRing (CanRxMsg x) {
	if(ringCounter >= BUFFER_SIZE) {
		//while(1);
		// BUFFER IS FULL!
		printf("Buffer is full\n");
		return;
	}
	
	buffer[writeIdx] = x;
	writeIdx++;
	if (writeIdx >= BUFFER_SIZE){
		writeIdx = 0;
	}
	ringCounter++;	
}

bool readFromRing (volatile CAN_msg_t *msgTable) {
	if (ringCounter ==0) {
		//printf("Buffer is empty\n");
		return false;
	}

	memcpy((void*)&msgTable->data, (void*)&buffer[readIdx].Data, sizeof(msgTable->data));
			// msgTable[i].data = *((uint64_t *)&(My_RX_message.Data[0]));
			// memcpy(&msgTable[i].data, &My_RX_message.Data[0], sizeof(msgTable[i].data));
	msgTable->update = 1;
	msgTable->msg_count++;
//printf("%llu\n\r", msgTable->data);

	ringCounter--;
	if(ringCounter<0)
	{
		ringCounter=0;
	}
	else
	{
		// do nothing
	}
	
	readIdx++;
	if (readIdx >= BUFFER_SIZE)
	{
		readIdx = 0;
	}

	return true;
}

bool isEmpty () {
	if (ringCounter == 0){
		return true;
	} 
	else{
		return false;
	}
}

int i = 0;

// CanRxMsg RxMessage; // HOW DO WE GET THE CAN MESSAGE TO FILL RxMessage????*****
CanRxMsg ringBuffer[50];
uint32_t readCount = 0;
uint32_t writeCount = 0;


void DFR_TIM3_Init(void);
void DFR_CAN_Init(uint32_t u32SensorID);

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
#if		001
  /* PLL configuration:  VCO = HSI/M * N,  Sysclk = VCO/P */		//	Configure for SYSCLK of 180 MHz
  RCC->PLLCFGR = ( 16ul                   |                /* PLL_M =   8 */
                 (200ul <<  6)            |                /* PLL_N = 180 */
                 (  0ul << 16)            |                /* PLL_P =   2 */
                 (RCC_PLLCFGR_PLLSRC_HSI) |                /* PLL_SRC = HSI */
                 (  7ul << 24)            |                /* PLL_Q =   7 */
                 (  2ul << 28)             );              /* PLL_R =   2 */
#else
  /* PLL configuration:  VCO = HSI/M * N,  Sysclk = VCO/P */		//	Configure for SYSCLK of 100 MHz
  RCC->PLLCFGR = ( 16ul                   |                /* PLL_M =  16 */
                 (200ul <<  6)            |                /* PLL_N = 200 */
                 (  0ul << 16)            |                /* PLL_P =   2 */
                 (RCC_PLLCFGR_PLLSRC_HSI) |                /* PLL_SRC = HSI */
                 (  7ul << 24)            |                /* PLL_Q =   7 */
                 (  2ul << 28)             );              /* PLL_R =   2 */
#endif
  RCC->CR |= RCC_CR_PLLON;                                 /* Enable PLL */
  while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           /* Wait till PLL is ready */

  RCC->CFGR &= ~RCC_CFGR_SW;                               /* Select PLL as system clock source */
  RCC->CFGR |=  RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  /* Wait till PLL is system clock src */
}



CanTxMsg	My_TX_message	=	
{
	0,								//  uint32_t StdId;  /*!< Specifies the standard identifier.
										//                        This parameter can be a value between 0 to 0x7FF. */
										//
	0x0CFFF048,					//  uint32_t ExtId;  /*!< Specifies the extended identifier.
										//                        This parameter can be a value between 0 to 0x1FFFFFFF. */
										//
	CAN_Id_Extended,	//  uint8_t IDE;     /*!< Specifies the type of identifier for the message that 
										//                        will be transmitted. This parameter can be a value 
										//                        of @ref CAN_identifier_type */
										//
	CAN_RTR_Data,			//  uint8_t RTR;     /*!< Specifies the type of frame for the message that will 
										//                        be transmitted. This parameter can be a value of 
										//                        @ref CAN_remote_transmission_request */
										//
	8,								//  uint8_t DLC;     /*!< Specifies the length of the frame that will be 
										//                        transmitted. This parameter can be a value between 
										//                        0 to 8 */
										//
										//  uint8_t Data[8]; /*!< Contains the data to be transmitted. It ranges from 0 
										//                        to 0xFF. */
	{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}

	// 0CFFF048
};

CanRxMsg	My_RX_message;
CanRxMsg *pointer_to_msg;

volatile	bool	Print_Flag	=	false;

volatile	int	My_Time = 0;

/*----------------------------------------------------------------------------
 * main: blink LED and check button state
 *----------------------------------------------------------------------------*/
 int main (void) {
  int32_t max_num = LED_GetCount();

	//SystemCoreClockConfigure();                              /* configure HSI as System Clock */
  SystemCoreClockUpdate();
  LED_Initialize();
  Buttons_Initialize();
  stdout_init();                                           /* Initialize Serial interface */
  SysTick_Config(SystemCoreClock / 1000);                  /* SysTick 1 msec interrupts */
	DFR_TIM3_Init();				//	Initialize TIM3 for a 1 mSec Interrupt  (TIM3 ISR)
	DFR_CAN_Init(0xFFFF12);
	 
	

  for (;;) {


  input_vector.engine_rpm = msgTable[0].data._16[0];
	 
	input_vector.engine_MAP = msgTable[1].data._16[1];
	
	input_vector.engine_temp = msgTable[2].data._16[1];

	//input_vector.motor_rpm = msgTable[9].data.Bamo_data_16.data; // change back
	input_vector.motor_rpm = (uint16_t)(msgTable[9].data._8[1]+msgTable[9].data._8[2]*256); // change back
	memcpy((void*)&input_vector.motor_rpm, (void*)&msgTable[9].data._8[1], sizeof(uint16_t));

	input_vector.motor_torque_rdval = msgTable[11].data.Bamo_data_16.data;

		if(Print_Flag)		//	TIM3_IRQHandler() ISR sets Print_Flag, we clear it here...
		{
			Print_Flag	=	false;
			My_Time	+=	5;						//	add another 5 seconds to My_Time
			CAN_Transmit(CAN1, &My_TX_message);		//	Let's send a message...
			JLM_Debug	=	CAN_TransmitStatus(CAN1, 0);
		}
		else
		{
			//	...
		}
		// CAN_Transmit(CAN1, &My_TX_message); // test
		//	If there's a CAN message, let's collect it...
		{
			#if	defined(ENABLE_CAN_INTERRUPTS)
			if(isEmpty())
			{
				// do nothing ring is empty
			}
			else 
			{ 	
				bool done = false;
				
				// parse new message
				
				i = 0;
				msgTableSize = (sizeof(msgTable)/sizeof(CAN_msg_t));
				
				while((done == false) && (i < msgTableSize)) {
/* change back to 0x180*/
					if( 
						((buffer[readIdx].StdId == msgTable[i].messageID) && (buffer[readIdx].IDE == msgTable[i].messageType) && buffer[readIdx].StdId != 0x180) ||  // Regular
						((buffer[readIdx].StdId == msgTable[i].messageID) && (buffer[readIdx].Data[0] == msgTable[i].REGID )) || // Bamocar
						((buffer[readIdx].ExtId == msgTable[i].messageID) && (buffer[readIdx].IDE == msgTable[i].messageType)) // Extended
					) {
						readFromRing(&msgTable[i]); // Fill in DOUBLE CHECK*****
						done = true;
					} else {
						i++; // increment
					}
				}  
shaun_debug = 0x123;
			}
			#else
			if(CAN_MessagePending(CAN1, CAN_FIFO0)	)
			{ 	
				bool done = false;
				
				CAN_Receive(CAN1, CAN_FIFO0, &My_RX_message);

				My_TX_message.Data[0]++; // if we get a ring, we visually acknowledge it by incrementing

				addToRing(My_RX_message);

				CAN_FIFORelease(CAN1, CAN_FIFO0);
				
				
				
				// put message handling code here
				
				// parse new message
				
				i = 0;
				msgTableSize = (sizeof(msgTable)/sizeof(CAN_msg_t));
				
				while((done == false) && (i < msgTableSize)) {
/* change back to 0x180*/
					if( 
						((buffer[readIdx].StdId == msgTable[i].messageID) && (buffer[readIdx].IDE == msgTable[i].messageType) && buffer[readIdx].StdId != 0x180) ||  // Regular
						((buffer[readIdx].StdId == msgTable[i].messageID) && (buffer[readIdx].Data[0] == msgTable[i].REGID )) || // Bamocar
						((buffer[readIdx].ExtId == msgTable[i].messageID) && (buffer[readIdx].IDE == msgTable[i].messageType)) // Extended
					) {
						readFromRing(&msgTable[i]); // Fill in DOUBLE CHECK*****
						done = true;
					} else {
						i++; // increment
					}
				}  
shaun_debug = 0x123;
			}
			else
			{
				//	...
			}
			#endif	// defined(ENABLE_CAN_INTERRUPTS)
			
		}
  }

}


void DFR_TIM3_Init(void) 
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  // Time Base Configuration
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  // TIM3 generates a 1 Msec tick for a 1 mSec ISR.
#if		000
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

uint16_t	Print_Counter	=	10;

void updateTerminal(void);

void updateTerminal(void){
	int i;
	int c = 1;
	for(i = 0; i < msgTableSize; i++){
	
		if(msgTable[i].update == 1)
		{
			printf("\033[%d;%dH %s",msgTable[i].row,c,msgTable[i].name);
			printf("\033[%d;%dH %llu", msgTable[i].row, msgTable[i].col, msgTable[i].data._64 );
			//printf("%llu",msgTable[i].data);
			msgTable[i].update = 0;
		}
	}
	
	printf("\033[%d;%dH %s",15,c,"Engine RPM");
	printf("\033[%d;%dH %d", 15, 15, input_vector.engine_rpm);
	
	printf("\033[%d;%dH %s",16,c,"Engine MAP");
	printf("\033[%d;%dH %d", 16, 15, input_vector.engine_MAP);
	
	printf("\033[%d;%dH %s",17,c,"Engine Temp");
	printf("\033[%d;%dH %d", 17, 15, input_vector.engine_temp);
	
	printf("\033[%d;%dH %s",18,c,"Motor RPM");
	printf("\033[%d;%dH %d", 18, 15, input_vector.motor_rpm);
	
	printf("\033[%d;%dH %s",19,c,"Motor RPM REGID"); // change back to Motor Torque
	printf("\033[%d;%dH %d", 19, 15,input_vector.motor_torque_rdval); //msgTable[9].data.Bamo_data_16.REGID
	
}


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
		Print_Counter	=	10;
		Print_Flag	=	true;
		XX_mSec_Tasks();
	}
}


void msg_safety_chk (void);

void msg_safety_chk(void){

	int n;
	int msg_error_count[msgTableSize-1];

	for (n = 0; n < msgTableSize; n++){
		if(msgTable[n].msg_count == msgTable[n].old_count)
		{
			msg_error_count[n]++;
		}
		else //if(msgTable[n].msg_count != msgTable[n].old_count)
		{
			msg_error_count[n] = 0;
			msgTable[n].old_count = msgTable[n].msg_count;
			// request msg function here
		}

		if(msg_error_count[n] == 20)
		{
			printf("\033[%d;%dH Crit. Msg Error for %s, shut down",15,25, msgTable[n].name );
			// shutdown function
		}
	
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
	
	//updateTerminal();
	//msg_safety_chk();
	
	state = prop_torqe; // need to figure out how we will change states
	
	switch(state){
		case off:
			kP = 0;
			break;
		case start:
			kP = 0;
			break;
		case init:
			kP = 0;
			break;
		case brake:
			kP = 0;
			break;
		case clutch:
			kP = 0;
			break;
		case prop_torqe:
 			kP = desired_kP();
			engine_torque = get_engine_torque(); 
			motor_torque = kP*engine_torque;
			torque_command(motor_torque);
			break;
	}
}



void ConfigureGPIO(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, GPIOMode_TypeDef GPIO_Mode, GPIOSpeed_TypeDef GPIO_Speed, GPIOOType_TypeDef GPIO_OType, GPIOPuPd_TypeDef GPIO_PuPd)
{
GPIO_InitTypeDef GPIO_InitStructure;

GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
GPIO_InitStructure.GPIO_OType = GPIO_OType;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd;

GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void DFR_CAN_Init(uint32_t u32SensorID)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
ConfigureGPIO(GPIOB, GPIO_Pin_8, GPIO_Mode_AF, GPIO_Speed_2MHz, GPIO_OType_OD, GPIO_PuPd_UP); //  PB8   Pin 61  CAN_RX/SER_RXD  Input
ConfigureGPIO(GPIOB, GPIO_Pin_9, GPIO_Mode_AF, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL); //  PB9   Pin 62  CAN_TX/SER_TXD  Output


	
#if		000
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);		//  PA11   Pin ??  CAN_RX  Input	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);		//  PA12   Pin ??  CAN_TX  Output
#else
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);		//  PB8   Pin 61  CAN_RX  Input	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);		//  PB9   Pin 62  CAN_TX  Output
#endif

	/* CAN register init */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_CAN1, ENABLE);
	#if		defined(USE_CAN2)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_CAN2, ENABLE);
	#endif
	CAN_DeInit(CAN1);		//	Always De-Init CAN1 even if we're not using it.
	#if		defined(USE_CAN2)
	CAN_DeInit(CAN2);
	#endif

	//	CAN Receive Message Filter Initialization
	{
		CAN_FilterInitTypeDef CAN_FilterInitStructure;

JLM_Debug	=	u32SensorID;

		// Prepare the extended ID for the filters list
		u32SensorID = ((u32SensorID<<3) | CAN_Id_Extended);	//CAN_ID_EXT;
JLM_Debug	=	u32SensorID;

#if		001
		CAN_FilterInitStructure.CAN_FilterNumber	=	0;		//	Filter  0 is the default for CAN1
		CAN_FilterInitStructure.CAN_FilterMode		=	CAN_FilterMode_IdMask;	//CAN_FilterMode_IdList;
		CAN_FilterInitStructure.CAN_FilterScale		=	CAN_FilterScale_32bit;

		CAN_FilterInitStructure.CAN_FilterIdHigh	=	0;	//((u32SensorID & 0xffff0000)>>16);
		CAN_FilterInitStructure.CAN_FilterIdLow		= 0;	// (u32SensorID & 0x0000ffff);
			CAN_FilterInitStructure.CAN_FilterMaskIdHigh	=	0;	//CAN_FilterInitStructure.CAN_FilterIdHigh;
			CAN_FilterInitStructure.CAN_FilterMaskIdLow		=	0;	//CAN_FilterInitStructure.CAN_FilterIdLow;
#else
		
		//	NOTE:	Note: CAN 2 start filter bank number n is configurable by writing to
		//				the CAN2SB[5:0] bits in the CAN_FMR register.
		
		/* CAN filter init */
	#if		defined(USE_CAN2)		
		CAN_SlaveStartBank(0x0E);													//	Shouldn't need this, it's the default value for CAN2

		CAN_FilterInitStructure.CAN_FilterNumber	=	0x0E;	//	Filter 14 is the default for CAN2
	#else
		CAN_FilterInitStructure.CAN_FilterNumber	=	0;		//	Filter  0 is the default for CAN1
	#endif
	
		CAN_FilterInitStructure.CAN_FilterMode		=	CAN_FilterMode_IdList;
		CAN_FilterInitStructure.CAN_FilterScale		=	CAN_FilterScale_32bit;

		//	NOTE:	We have 2 ID Filters active in this Mode (CAN_FilterMode_IdList)
		//	Set MSGID_2 into 1st filter
		CAN_FilterInitStructure.CAN_FilterIdHigh	=	((u32SensorID & 0xffff0000)>>16);
		CAN_FilterInitStructure.CAN_FilterIdLow		=  (u32SensorID & 0x0000ffff);

		{
			//	Make 2nd ID Filter match 1st Filter
			CAN_FilterInitStructure.CAN_FilterMaskIdHigh	=	CAN_FilterInitStructure.CAN_FilterIdHigh;
			CAN_FilterInitStructure.CAN_FilterMaskIdLow		=	CAN_FilterInitStructure.CAN_FilterIdLow;
		}
#endif
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment	=	0;
		CAN_FilterInitStructure.CAN_FilterActivation			=	ENABLE;

		CAN_FilterInit(&CAN_FilterInitStructure);
	}

	//	CAN Unit Initialization
	{
		CAN_InitTypeDef    		CAN_InitStructure;

		CAN_StructInit(&CAN_InitStructure);

		/* CAN cell init */
		CAN_InitStructure.CAN_TTCM			=	DISABLE;	//"...0: 	Time Triggered Communication mode disabled..."
		CAN_InitStructure.CAN_ABOM			=	ENABLE;		//"...1: 	The Bus-Off state is left automatically by hardware once 
																								//				128 occurrences of 11 recessive bits have been monitored.
		CAN_InitStructure.CAN_AWUM			=	DISABLE;	//"...0: 	The Sleep mode is left on software request by clearing the 
																								//				SLEEP bit of the CAN_MCR register...."
		CAN_InitStructure.CAN_NART			=	DISABLE;	//"...0: 	The CAN hardware will automatically retransmit the message until 
																								//				it has been successfully transmitted according to the CAN standard...."
		CAN_InitStructure.CAN_RFLM			=	DISABLE;	//"...0: 	Receive FIFO not locked on overrun. Once a receive FIFO is full the next 
																								//				incoming message will overwrite the previous one...."	
		CAN_InitStructure.CAN_TXFP			=	DISABLE;	//"...0: 	Priority driven by the identifier of the message..."

// #define		CAN_LOOP_BACK_MODE	(1)

#if		defined(CAN_LOOP_BACK_MODE)		
		CAN_InitStructure.CAN_Mode			=	CAN_Mode_LoopBack;
#else
		CAN_InitStructure.CAN_Mode			=	CAN_Mode_Normal;
#endif
#if 000
		//============
		//	250 Kbaud
		//============
		//	KBaud Rate = ((45 MHz/CAN_Prescaler)*(10^3))/(CAN_SJW+CAN_BS1+CAN_BS2)

		//	KBaud Rate = ((45 MHz/18)*(10^3))/(1+7+2) = ((2.5 MHz)*(10^3))/10 = (0.25 MHz)*(10^3) = 250 Kbaud

		CAN_InitStructure.CAN_SJW				=	CAN_SJW_1tq;
		CAN_InitStructure.CAN_BS1				=	CAN_BS1_7tq;		//	consider CAN_BS1_12tq
		CAN_InitStructure.CAN_BS2				=	CAN_BS2_2tq;		//	consider CAN_BS2_3tq, for 81.25% Sampling point  [(CAN_BS1_12tq+CAN_SJW_1tq)/(CAN_BS1_12tq+CAN_BS2_1tq+CAN_SJW_3tq)]
		CAN_InitStructure.CAN_Prescaler	=	18;     // 250 Kbaud
#else	
		//============
		//	500 Kbaud
		//============
		//	KBaud Rate = ((45 MHz/CAN_Prescaler)*(10^3))/(CAN_SJW+CAN_BS1+CAN_BS2)

		//	KBaud Rate = ((45 MHz/9)*(10^3))/(1+7+2) = ((5.0 MHz)*(10^3))/10 = (0.50 MHz)*(10^3) = 500 Kbaud

		CAN_InitStructure.CAN_SJW				=	CAN_SJW_1tq;
		CAN_InitStructure.CAN_BS1				=	CAN_BS1_7tq;		//	consider CAN_BS1_12tq
		CAN_InitStructure.CAN_BS2				=	CAN_BS2_2tq;		//	consider CAN_BS2_3tq, for 81.25% Sampling point  [(CAN_BS1_12tq+CAN_SJW_1tq)/(CAN_BS1_12tq+CAN_BS2_1tq+CAN_SJW_3tq)]
		CAN_InitStructure.CAN_Prescaler	=	9;     // 500 Kbaud
#endif
	#if		defined(USE_CAN2)
		CAN_Init(CAN2,	&CAN_InitStructure);
	#else
		CAN_Init(CAN1,	&CAN_InitStructure);

		#if		(defined (ENABLE_CAN_INTERRUPTS)	)
//void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState)
		
		  {
   NVIC_InitTypeDef NVIC_InitStructure;
   // CAN TX Interrupt Handling
   NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn; //USB_HP_CAN_TX_IRQChannel;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   // NVIC_Init(&NVIC_InitStructure); // bring it back later
   // CAN RX Interrupt Handling
   NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn; //USB_LP_CAN_RX0_IRQChannel;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
  }
			
		CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);
		
		
		#endif
	#endif
	}
}



#if	defined(ENABLE_CAN_INTERRUPTS)

void CAN1_TX_IRQHandler(void)
{
  // Check if there is anything in the CAN TX queue to send; if so, send it. 
 //[Note that CAN is not interrupt driven this leads to low bus utilization.]
 
// CAN_TX_Q_Read_Index++;  // point to the next message in the Q
// if(CAN_TX_Q_Read_Index >= CAN_TX_Q_LENGTH)
// {
//  CAN_TX_Q_Read_Index = 0;
// }
// else
// {
//  // ...
// }
// if(CAN_TX_Q_COUNT <= 0)
// {
//  // !!!LOGIC ERROR!!! we should never have a negative value for the Count!!!
//  CAN_TX_Q_COUNT = 0; // Sorry attempt at recovery...
// }
// else
// {
//  CAN_TX_Q_COUNT--;     // mark the completion of transmission of the previous message
//  if(CAN_TX_Q_COUNT > 0)
//  {
//   CAN_Transmit(CAN1, &CAN_TX_Q[CAN_TX_Q_Read_Index]);
//  }
//  else
//  {
//   // ...Q is empty, nothing more to send...wait for another transmission...
//  }
// }
 CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
}

void CAN1_RX0_IRQHandler(void)
{
	#if 001
	
			if(CAN_MessagePending(CAN1, CAN_FIFO0)	)
			{ 	
//				bool done = false;
				
				CAN_Receive(CAN1, CAN_FIFO0, &My_RX_message);

				My_TX_message.Data[0]++; // if we get a ring, we visually acknowledge it by incrementing

				addToRing(My_RX_message);

				CAN_FIFORelease(CAN1, CAN_FIFO0);
				
			}
			else
			{
				//	...
			}
	
	 CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);

	#else
	 
 // If there's a CAN message, let's collect it...
 if(CAN_MessagePending(CAN1, CAN_FIFO0) )
 {
  if(CAN_RX_Q_COUNT < (CAN_RX_Q_LENGTH-1) )
  {
   CAN_Receive(CAN1, CAN_FIFO0, &CAN_RX_Q[CAN_RX_Q_Write_Index]);
   CAN_RX_Q_Write_Index++;
   if(CAN_RX_Q_Write_Index >= CAN_RX_Q_LENGTH)
   {
    CAN_RX_Q_Write_Index = 0;
   }
   else
   {
    // ...
   }
   CAN_RX_Q_COUNT++;  // Indicate new message available
  }
  else //(CAN_RX_Q_COUNT >= CAN_RX_Q_LENGTH)
  {
   // Q is full, lose latest message...or rewrite this code to lose oldest message...
   CanRxMsg Lost_Message;
   CAN_Receive(CAN1, CAN_FIFO0, &Lost_Message);
  }
 }
 else
 {
  // ...LOGIC ERROR!!! This shouldn't happen...shouldn't get an Interrupt without a message...
 }
 CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
 #endif
}

#endif 
