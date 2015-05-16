/**
  ******************************************************************************
  * @file	 	GlobalInit.c
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		sourse
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "GlobalInit.h"
#include "motor.h"
#include "pumps.h"
#include "heater.h"
#include "selector.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t USART_RxBuffer[8] = {0};
__IO float TIM1_Frequency = 0;

/* Extern function -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void GPIO_Configuration(void);
static void RTC_UserConfiguration(void);
static void USART_DMA_DefaultConfiguration(void);
static void Timer1_init(void);
static void Timer4_Configuration(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Global configures the mcu peripheal.
  * @param  None
  * @retval None
  */
void GlobalInit(void)
{
    __disable_irq();

    GPIO_Configuration();

    CU_Selector_Configuration();
    CU_Motor_Configuration();
    CU_Pumps_Configuration();
    CU_Program_ReadUserTempControl();
    CU_Heater_Configuration();

    RTC_UserConfiguration();
    USART_DMA_DefaultConfiguration();

    Timer1_init();
    Timer4_Configuration();

    Debug_Send_String("\r\n");
    delay_ms(2000);
    /* Load CU program, not change later */
    CU_Program_Configuration();

    __enable_irq();
}


/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIO/AFIO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);


    /* ---------------------------------------- OUTPUTS ---------------------------------------- */

    /* --------- port C --------- */
    GPIO_InitStructure.GPIO_Pin = GPIO_EN_HV_LINE_PIN | GPIO_ON_OFF_LAMP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinLockConfig(GPIOC, GPIO_EN_HV_LINE_PIN | GPIO_ON_OFF_LAMP);

    /* Global ENABLE ~HV line*/
    GPIO_ResetBits(GPIOC, GPIO_EN_HV_LINE_PIN);

    /* ---------------------------------------- INPUTS ---------------------------------------- */
    GPIO_InitStructure.GPIO_Pin = GPIO_FAST_MODE_BUTTON_PIN | GPIO_ECONOM_MODE_BUTTON_PIN | \
                                  GPIO_LOW_MOTOR_SPEED_BUTTON_PIN | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIO_BUTTONS_PORT, &GPIO_InitStructure);
    GPIO_PinLockConfig(GPIO_BUTTONS_PORT, GPIO_FAST_MODE_BUTTON_PIN | GPIO_ECONOM_MODE_BUTTON_PIN | \
                       GPIO_LOW_MOTOR_SPEED_BUTTON_PIN | GPIO_Pin_7);

//    if((GPIO_BUTTONS_PORT->IDR & GPIO_ECONOM_MODE_BUTTON_PIN) != 0)
//    {
//        Jump_To_Bootloader();
//    }
}


/**
  * @brief  Configures the RTC.
  * @param  None
  * @retval None
  */
static void RTC_UserConfiguration(void)
{
    NVIC_EnableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, RTC_INTR_PRIORITY);

    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Backup Domain Reset */
//	BKP_DeInit();

    RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div128);

    /* RTC Enabled */
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForLastTask();

    /*Wait for RTC registers synchronisation */
    RTC_WaitForSynchro();
    RTC_WaitForLastTask();

    /* Setting RTC Interrupts-Seconds interrupt enabled */
    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    RTC_WaitForLastTask();

    /* Set RTC prescaler: set RTC period to 1 sec */
    RTC_SetPrescaler((62500) - 1); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
    RTC_WaitForLastTask();

    /* Change the current time */
    RTC_SetCounter(0);
    RTC_WaitForLastTask();
}


/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
static void USART_Configuration(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//
//	/* USART for debug */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
//
//	/* Configure USART1 Tx () as alternate function push-pull */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//	/* Configure USART1 Rx () as input floating */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//
//	/* USART1 configuration ------------------------------------------------------*/
//	/* USART1 configured as follow:
//	      - BaudRate = 115200 baud
//	      - Word Length = 8 Bits
//	      - One Stop Bit
//	      - No parity
//	      - Hardware flow control disabled (RTS and CTS signals)
//	      - Receive and transmit enabled
//	      - USART Clock disabled
//	      - USART CPOL: Clock is active low
//	      - USART CPHA: Data is captured on the middle
//	      - USART LastBit: The clock pulse of the last data bit is not output to
//	                       the SCLK pin
//	*/
//	USART_InitStructure.USART_BaudRate = 230400;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//
//	USART_Init(USART1, &USART_InitStructure);
//
//	/* Enable USART1 */
//	USART_Cmd(USART1, ENABLE);


    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* USART resources configuration (Clock, GPIO pins and USART/DMA registers) */

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART configured as follow:
    	        - BaudRate = 921600 baud
    	        - Word Length = 8 Bits
    	        - One Stop Bit
    	        - No parity
    	        - Hardware flow control disabled (RTS and CTS signals)
    	        - Receive and transmit enabled
     */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* Enable USART1 DMA Rx request */
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    /* Enable the USART1 */
    USART_Cmd(USART1, ENABLE);		/* Enable the USART1 */



    // устанавливаем приоритет и разрешаем прерывания USART1
    //NVIC_EnableIRQ(USART1_IRQn);
    //NVIC_SetPriority(USART1_IRQn, 1);
}


/**
  * @brief  USART_DMA_DefaultConfiguration.
  * @param  None
  * @retval None
  */
static void USART_DMA_DefaultConfiguration(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    DMA_DeInit(DMA1_Channel5);

    DMA1_Channel5->CCR &= ~DMA_CCR5_EN; 							// Disable
    DMA1_Channel5->CPAR = (uint32_t)&USART1->DR; 					// Periph addr
    DMA1_Channel5->CMAR = (uint32_t)USART_RxBuffer;					// Memory addr
    DMA1_Channel5->CCR &= ~DMA_CCR5_DIR; 							// Periph -> Memory
    DMA1_Channel5->CNDTR = (uint32_t)4;								// Data cnt
    DMA1_Channel5->CCR &= ~DMA_CCR1_PINC; 							// Perip addr not increment
    DMA1_Channel5->CCR |= DMA_CCR1_MINC; 							// Memory addr increment
    DMA1_Channel5->CCR &= ~DMA_CCR1_PSIZE; 							// Periph data 8 bit
    DMA1_Channel5->CCR &= ~DMA_CCR1_MSIZE; 							// Memory data 8 bit
    DMA1_Channel5->CCR |= DMA_CCR1_CIRC; 							// Circular mode
    DMA1_Channel5->CCR |= DMA_Priority_Medium; 						// Medium priority

    USART_Configuration();

    /* Enable USART1 TX/RX DMA1 Channel */
    DMA_Cmd(DMA1_Channel5, ENABLE);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    NVIC_SetPriority(DMA1_Channel5_IRQn, DMA_USART_INTR_PRIORITY);
}


/**
  * @brief  Timer4_Configuration
  * @param  None
  * @retval None
  */
static void Timer1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;		//
    TIM1->PSC = 240 - 1;					//
    TIM1->ARR = 5000;						//
    TIM1->DIER |= TIM_DIER_UIE;
    TIM1->CR1 |=  TIM_CR1_CEN | TIM_CR1_ARPE;	//

    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, TIM1_INTR_PRIORITY);

    TIM1_Frequency = (float)SystemCoreClock / ((float)(TIM1->PSC) * (float)(TIM1->ARR));
}


/**
  * @brief  Timer4_Configuration
  * @param  None
  * @retval None
  */
void Timer1_ReConfig(float cnt_div)
{
	if(cnt_div >= 10) cnt_div = 10;

	TIM1->CR1 &= ~TIM_CR1_CEN;
	TIM1->ARR = (uint16_t)(5000.0F / cnt_div);
    TIM1->CR1 |= TIM_CR1_CEN;

    TIM1_Frequency = (float)SystemCoreClock / ((float)(TIM1->PSC) * (float)(TIM1->ARR));
}


/**
  * @brief  Timer4_Configuration
  * @param  None
  * @retval None
  */
float Timer1_GetFrequency(void)
{
	return TIM1_Frequency;
}


/**
  * @brief  Timer4_Configuration
  * @param  None
  * @retval None
  */
void Timer4_Configuration(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;			// разрешаем татирование модуля
    TIM4->PSC = 24 - 1;							// устанавливаем предделитель
    TIM4->ARR = 1000 - 1;						// значение до которого будет считать таймер
    TIM4->CR1 = TIM_CR1_ARPE;					// разрешаем автоперезагрузку
    TIM4->DIER = TIM_DIER_UIE;					// прерывание по обновлению

    /* устанавливаем приоритет */
    NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_SetPriority(TIM4_IRQn, TIM4_INTR_PRIORITY);
}


/**
  * @brief  Jump_To_Bootloader
  * @param  None
  * @retval None
  */
void Jump_To_Bootloader(void)
{
    Debug_Send_String("\r\n SWITCH TO BOOTLOADER...");

    FLASH_Unlock();
    FLASH_ErasePage(0x08020000-0x200);
    FLASH_ProgramWord(0x08020000-0x200, 0xA5FDDF5A);
    FLASH_Lock();

    /* Reset and jump to bootloader */
    NVIC_SystemReset();
}





/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
