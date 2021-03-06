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

typedef unsigned int bool;
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
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //Enable GPIOB for ADC. Use PB0 for ADC input
	
	//Set PB0 to General purpose in/out
	GPIOC->MODER |= GPIO_MODER_MODER0;
	
		//Set no pull up/down for PB0
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR0;
	
	// Enable ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	
	// Set resolution to 8-bit
	ADC1->CFGR1 &= ~ADC_CFGR1_RES;
	ADC1->CFGR1 |= ADC_CFGR1_RES_1;
	
	// Set Cont. Conversion mode
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	
	// Hardware triger disabled
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;
	
	// Select/Enable the input channel 8 for PB0
	ADC1->CHSELR &= ~ADC_CHSELR_CHSEL;
	ADC1->CHSELR |= ADC_CHSELR_CHSEL10;
	
	// Calibrate
	ADC1->CR |= ADC_CR_ADCAL;
	while(ADC1->CR & ADC_CR_ADCAL){}
	
	
	ADC1->ISR |= ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0){}
	
	ADC1->CR |= ADC_CR_ADSTART;
	
	/*if((ADC1->CR & ADC_CR_ADEN) != 0){
		ADC1->CR |= ADC_CR_ADDIS;
	}
	while((ADC1->CR & ADC_CR_ADEN) != 0){}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
	ADC1->CR |= ADC_CR_ADCAL;
	while((ADC1->CR & ADC_CR_ADCAL) != 0){}
		
	if((ADC1->ISR & ADC_ISR_ADRDY) != 0){
		ADC1->ISR |= ADC_ISR_ADRDY;
	}
	ADC1->CR |= ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0){}
	ADC1->CR |= ADC_CR_ADSTART;*/
	
}

void Setup_DAC(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
		//Set PB0 to General purpose in/out
	GPIOA->MODER |= GPIO_MODER_MODER4;
	
		//Set no pull up/down for PB0
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4;
	
	DAC->CR |= DAC_CR_TSEL1;
	//DAC->CR |= DAC_CR_TEN1;
	//DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
	DAC->CR |= DAC_CR_EN1;	

}

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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
  HAL_Init();
  SystemClock_Config();

	initLeds();
	//Setup_ADC();
	Setup_DAC();
	const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
	232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
	//uint8_t count = 0;
	while(true)
	{/*
		(ADC1->DR > 32) ? turn_on_LED('o') : turn_off_LED('o');
		(ADC1->DR > 64) ? turn_on_LED('b') : turn_off_LED('b');	
		(ADC1->DR > 128) ? turn_on_LED('g') : turn_off_LED('g');	
		(ADC1->DR > 200) ? turn_on_LED('r') : turn_off_LED('r');		*/
		//int i;
		for(int i = 0; i < 32; i++){
			HAL_Delay(1);
			DAC->DHR8R1 = sine_table[i];
		}
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
