/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

typedef unsigned char bool;
#define true 1
#define false 0

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
volatile uint32_t debouncer;


void setup_USART(void){
	
	// Enable USART1 Clock
	//PA9 TX, PA10 RX
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// Enable GPIOA Clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// Set alternate function mode 10
	GPIOA->MODER |= GPIO_MODER_MODER9;
	GPIOA->MODER |= GPIO_MODER_MODER10;
	GPIOA->MODER &= ~(GPIO_MODER_MODER9_0);
	GPIOA->MODER &= ~(GPIO_MODER_MODER10_0);
	
	// Set AFRH 0001
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9);
	GPIOA->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL9_Pos); 
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL10);
	GPIOA->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL10_Pos);
	
	USART1->BRR = HAL_RCC_GetHCLKFreq()/115200;
	USART1->CR1 |= USART_CR1_UE;
	USART1->CR1 |= USART_CR1_RE;
	USART1->CR1 |= USART_CR1_TE;
	
	// Push-pull
	GPIOA->OTYPER = 0x00000;
	
	//High Speed
	GPIOA->OSPEEDR = GPIO_OSPEEDR_OSPEEDR9;
	GPIOA->OSPEEDR = GPIO_OSPEEDR_OSPEEDR10;
	
	// No pull resistors
	GPIOA->PUPDR = 0x0;
}

void singleChar(char c){
	while((USART1->ISR & USART_ISR_TXE) == 0){}
	USART1->TDR = c;
}

void writeString(char* c){
	int i = 0;
	while(1){
		singleChar(c[i]);
		i++;
		if(c[i] == 0){
			break;
		}
	}
}

int turn_off_LED(char ch)
{
		switch(ch)
	{
		case 'r':
		case 'R':
			GPIOC->BSRR |= GPIO_BSRR_BR_6;
			break;
		case 'g':
		case 'G':
			GPIOC->BSRR |= GPIO_BSRR_BR_9;
			break;
		case 'b':
		case 'B':
			GPIOC->BSRR |= GPIO_BSRR_BR_7;
			break;
		case 'o':
		case 'O':
			GPIOC->BSRR |= GPIO_BSRR_BR_8;
			break;
		default:
			return 0;
	}
	return 1;
}

int turn_on_LED(char ch)
{
		switch(ch)
	{
		case 'r':
		case 'R':
			GPIOC->BSRR |= GPIO_BSRR_BS_6;
			break;
		case 'g':
		case 'G':
			GPIOC->BSRR |= GPIO_BSRR_BS_9;
			break;
		case 'b':
		case 'B':
			GPIOC->BSRR |= GPIO_BSRR_BS_7;
			break;
		case 'o':
		case 'O':
			GPIOC->BSRR |= GPIO_BSRR_BS_8;
			break;
		default:
			return 0;
	}
	return 1;
}

void initLeds(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //Enable GPIOC for LEDs
	
	//Set PC6-PC9 to General purpose in/out
	GPIOC->MODER &= ~(0xFF << GPIO_MODER_MODER6_Pos);
	GPIOC->MODER |= 0x01 << GPIO_MODER_MODER6_Pos;
	GPIOC->MODER |= 0x01 << GPIO_MODER_MODER7_Pos;
	GPIOC->MODER |= 0x01 << GPIO_MODER_MODER8_Pos;
	GPIOC->MODER |= 0x01 << GPIO_MODER_MODER9_Pos;
	
		//Set PC6-PC9 to push-pull 
	GPIOC->OTYPER &= ~(0xF << 6); //Clear PC6-PC9 for push-pull mode 
	
	//Set PC6-PC9 to low_speed mode
	GPIOC->OSPEEDR &= ~(0xFF << GPIO_OSPEEDR_OSPEEDR6_Pos); // Clear PC6-PC9 for low-speed mode
	
		//Set no pull up/down for PC6-PC9
	GPIOC->PUPDR &= ~(0xFF << GPIO_PUPDR_PUPDR6_Pos);
	
	// Initialize LED states
	turn_off_LED('o');
	turn_off_LED('b');
	turn_off_LED('r');
	turn_off_LED('g');

	
}

void Setup_ADC()
{
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //Enable GPIOC for ADC. Use PC0 for ADC input
	
	//Set PC0 to General purpose in/out
	GPIOC->MODER |= GPIO_MODER_MODER0;
	//Set PC3 to General purpose in/out
	GPIOC->MODER |= GPIO_MODER_MODER3;
	
	
		//Set no pull up/down for PC0
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR0;
	//Set no pull up/down for PC3
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR3;
	
	
	// Enable ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	
	// Set resolution to 8-bit
	ADC1->CFGR1 &= ~ADC_CFGR1_RES;
	ADC1->CFGR1 |= ADC_CFGR1_RES_1;
	
	// Clear Cont. Conversion mode
	ADC1->CFGR1 &= ~ADC_CFGR1_CONT;
	
	// Hardware triger disabled
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;
	
	// Disable all input channels
	ADC1->CHSELR &= ~ADC_CHSELR_CHSEL;
	
	// Calibrate
	ADC1->CR |= ADC_CR_ADCAL;
	while(ADC1->CR & ADC_CR_ADCAL){}
	
	//Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
		
	ADC1->ISR |= ADC_ISR_ADRDY;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0){}
			
}

int Read_ADC_PC0()
{
	// Disable all input channels
	ADC1->CHSELR &= ~ADC_CHSELR_CHSEL;
	// Select/Enable the input channel 10 for PC0
	ADC1->CHSELR |= ADC_CHSELR_CHSEL10;
	ADC1->CR |= ADC_CR_ADSTART;
	return ADC1->DR;
}

int Read_ADC_PC3()
{
		// Disable all input channels
	ADC1->CHSELR &= ~ADC_CHSELR_CHSEL;
	
	// Select/Enable the input channel 10 for PC0
	ADC1->CHSELR |= ADC_CHSELR_CHSEL13;
	ADC1->CR |= ADC_CR_ADSTART;
	return ADC1->DR;
}

void  button_init(void) {
    // Initialize PA0 for button input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}

/* Called by SysTick Interrupt
 * Performs button debouncing, changes wave type on button rising edge
 * Updates frequency output from ADC value
 */
void HAL_SYSTICK_Callback(void) {
    // Remember that this function is called by the SysTick interrupt
    // You can't call any functions in here that use delay

    debouncer = (debouncer << 1);
    if(GPIOA->IDR & (1 << 0)) {
        debouncer |= 0x1;
    }

    if(debouncer == 0x7FFFFFFF) {
				turn_on_LED('o');
    }
}

void setup_timer(void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	
	// 3k HZ
	//TIM6->PSC = 28; // TODO: Change this!
  //TIM6->ARR = 100; // TODO: Change this!
	
	TIM6->PSC = 7999; // TODO: Change this!
  TIM6->ARR = 400; // TODO: Change this!

  TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
  TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

  NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
  NVIC_SetPriority(TIM6_DAC_IRQn,2);
}

// Encoder interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void) {

		GPIOC->ODR ^= GPIO_ODR_9;   
	
    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

/* USER CODE BEGIN PFP */
/* Private function prototyp
es -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	debouncer = 0;
  HAL_Init();
  SystemClock_Config();

	initLeds();
	Setup_ADC();
	setup_USART();
	button_init();
	setup_timer();
	
	while(true)
	{
		HAL_Delay(500);
		writeString("hello");
	}
	
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
