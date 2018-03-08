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

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

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

	
}


int turn_off_LED(char ch)
{
		switch(ch)
	{
		case 'r':
			GPIOC->BSRR |= GPIO_BSRR_BR_6;
			break;
		case 'g':
			GPIOC->BSRR |= GPIO_BSRR_BR_9;
			break;
		case 'b':
			GPIOC->BSRR |= GPIO_BSRR_BR_7;
			break;
		case 'o':
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
			GPIOC->BSRR |= GPIO_BSRR_BS_6;
			break;
		case 'g':
			GPIOC->BSRR |= GPIO_BSRR_BS_9;
			break;
		case 'b':
			GPIOC->BSRR |= GPIO_BSRR_BS_7;
			break;
		case 'o':
			GPIOC->BSRR |= GPIO_BSRR_BS_8;
			break;
		default:
			return 0;
	}
	return 1;
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
	//PB11 -> I2C2_SDA AF1
	//PB13 -> I2C2_SCL AF5
	// Enable I2C2 clock
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// Enable GPIOB and GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	//RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set to open-drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
	
	// Set alternate function
	GPIOB->MODER &= ~(GPIO_MODER_MODER11);
	GPIOB->MODER |= 0x2 << GPIO_MODER_MODER11_Pos;
	
	GPIOB->MODER &= ~(GPIO_MODER_MODER13); // Fixed this
	GPIOB->MODER |= 0x2 << GPIO_MODER_MODER13_Pos;
	
	// 0001
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL11);
	GPIOB->AFR[1] |= 0x1 << GPIO_AFRH_AFSEL11_Pos;
	
	// 0101
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL13);
	GPIOB->AFR[1] |= 0x5 << GPIO_AFRH_AFSEL13_Pos;
	
	
	// Output mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER14);
	GPIOB->MODER |= 0x1 << GPIO_MODER_MODER14_Pos;
	
	GPIOC->MODER &= ~(GPIO_MODER_MODER0);
	GPIOC->MODER |= 0x1 << GPIO_MODER_MODER0_Pos;
	
	// Push-pull
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);
	GPIOB->BSRR |= GPIO_BSRR_BS_14;
	
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_0);
	GPIOC->BSRR |= GPIO_BSRR_BS_0;
	
	// Init I2C
	I2C2->TIMINGR &= ~(I2C_TIMINGR_PRESC);
	I2C2->TIMINGR |= 0x1 << I2C_TIMINGR_PRESC_Pos;
	
	I2C2->TIMINGR &= ~(I2C_TIMINGR_SCLL);
	I2C2->TIMINGR |= 0x13 << I2C_TIMINGR_SCLL_Pos;
	
	I2C2->TIMINGR &= ~(I2C_TIMINGR_SCLH);
	I2C2->TIMINGR |= 0xF << I2C_TIMINGR_SCLH_Pos;
	
	I2C2->TIMINGR &= ~(I2C_TIMINGR_SDADEL);
	I2C2->TIMINGR |= 0x2 << I2C_TIMINGR_SDADEL_Pos;
	
	I2C2->TIMINGR &= ~(I2C_TIMINGR_SCLDEL);
	I2C2->TIMINGR |= 0x4 << I2C_TIMINGR_SCLDEL_Pos;
	
	I2C2->CR1 |= I2C_CR1_PE;
	/*I2C2->CR2 = 0;
	I2C2->CR2 &= ~(I2C_CR2_SADD);//~(0xFE); // 0000 1111 1110
	I2C2->CR2 |= 0x6B << 1; 
	
	I2C2->CR2 &= ~(I2C_CR2_NBYTES);
	I2C2->CR2 |= 0x1 << (I2C_CR2_NBYTES_Pos);
	
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
	I2C2->CR2 |= I2C_CR2_START;

	while(1){
		if((I2C2->ISR & I2C_ISR_TXIS) != 0){break;}
		if((I2C2->ISR & I2C_ISR_NACKF) != 0){break;}
					//turn_on_LED('b');
	}
	turn_on_LED('o');
	
	if(I2C2->ISR & I2C_ISR_NACKF){
		//SOME ERROR
		turn_on_LED('r');
	}
	I2C2->TXDR = 0x0F;
	while(!(I2C2->ISR & I2C_ISR_TC)){}
		
	I2C2->CR2 &= ~(0xFE); // 0000 1111 1110
	I2C2->CR2 |= 0x6B << 1; 
	
	I2C2->CR2 &= ~(I2C_CR2_NBYTES);
	I2C2->CR2 |= 0x1 << (I2C_CR2_NBYTES_Pos);
	
	I2C2->CR2 |= I2C_CR2_RD_WRN;
	I2C2->CR2 |= I2C_CR2_START;
		
	while(1){		
		if(I2C2->ISR & I2C_ISR_RXNE){break;}
		if(I2C2->ISR & I2C_ISR_NACKF){break;}}
	if(I2C2->ISR & I2C_ISR_NACKF){
		//SOME ERROR
		turn_on_LED('r');
	}
	
	if(I2C2->RXDR == 0xD4){
		// SUCCESS
		turn_on_LED('g');
	}
	
	while(!(I2C2->ISR & I2C_ISR_TC)){}
	I2C2->CR2 |= I2C_CR2_STOP;
*/				
	// PART 2 - GYROSCOPE -------------------------------------------------------------------------------------------------------------------
	// 0000 1011
	I2C2->CR2 = 0;
	I2C2->CR2 &= ~(I2C_CR2_SADD);//~(0xFE); // 0000 1111 1110
	I2C2->CR2 |= 0x6B << 1; 
	
	I2C2->CR2 &= ~(I2C_CR2_NBYTES);
	I2C2->CR2 |= 0x2 << (I2C_CR2_NBYTES_Pos);
	
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
	I2C2->CR2 |= I2C_CR2_START;

	while(1){
		if((I2C2->ISR & I2C_ISR_TXIS) != 0){break;}
		if((I2C2->ISR & I2C_ISR_NACKF) != 0){break;}
					//turn_on_LED('b');
	}
	if(I2C2->ISR & I2C_ISR_NACKF){
		//SOME ERROR
		turn_on_LED('r');
	}
	I2C2->TXDR = 0x20; // Addres CR

	while(1){
		if((I2C2->ISR & I2C_ISR_TXIS) != 0){break;}
		if((I2C2->ISR & I2C_ISR_NACKF) != 0){break;}
	}
	if(I2C2->ISR & I2C_ISR_NACKF){
		//SOME ERROR
		turn_on_LED('r');
	}

	I2C2->TXDR = 0x0B;
	while(!(I2C2->ISR & I2C_ISR_TC)){}
	I2C2->CR2 |= I2C_CR2_STOP;
	

// X AND Y CHECKING ----------------------------------------------------------------------------------------------------
		
	int16_t x = 0; // x axis
	int16_t y = 0; // y axis
	while(1){
		HAL_Delay(100);
		
		I2C2->CR2 = 0;
		I2C2->CR2 &= ~(I2C_CR2_SADD);//~(0xFE); // 0000 1111 1110
		I2C2->CR2 |= 0x6B << 1; 
	
		I2C2->CR2 &= ~(I2C_CR2_NBYTES);
		I2C2->CR2 |= 0x1 << (I2C_CR2_NBYTES_Pos);
	
		I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
		I2C2->CR2 |= I2C_CR2_START;

		while(1){
			if((I2C2->ISR & I2C_ISR_TXIS) != 0){break;}
			if((I2C2->ISR & I2C_ISR_NACKF) != 0){break;}
					//turn_on_LED('b');
		}
		if(I2C2->ISR & I2C_ISR_NACKF){
			//SOME ERROR
			turn_on_LED('r');
		}
	I2C2->TXDR = 0xA8;
	
	while(!(I2C2->ISR & I2C_ISR_TC)){}
	I2C2->CR2 &= ~(0xFE); // 0000 1111 1110
	I2C2->CR2 |= 0x6B << 1; 
	
	I2C2->CR2 &= ~(I2C_CR2_NBYTES);
	I2C2->CR2 |= 0x2 << (I2C_CR2_NBYTES_Pos);
	
	I2C2->CR2 |= I2C_CR2_RD_WRN;
	I2C2->CR2 |= I2C_CR2_START;
		
	while(1){		
		if(I2C2->ISR & I2C_ISR_RXNE){break;}
		if(I2C2->ISR & I2C_ISR_NACKF){break;}}
	if(I2C2->ISR & I2C_ISR_NACKF){
		//SOME ERROR
		turn_on_LED('r');
	}
	// Check to make sure it gets here
				turn_on_LED('b');
	
	// Address is supposed to automatically change according to datasheets.
	x = I2C2->RXDR;
	while(1){		
		if(I2C2->ISR & I2C_ISR_RXNE){break;}
		if(I2C2->ISR & I2C_ISR_NACKF){break;}}
	if(I2C2->ISR & I2C_ISR_NACKF){
		//SOME ERROR
		turn_on_LED('r');
	}
		x |= I2C2->RXDR << 8; 
		while(!(I2C2->ISR & I2C_ISR_TC)){}
			I2C2->CR2 |= I2C_CR2_STOP;
			
		if(x > 0){
			turn_on_LED('o');
		}
		else {
			turn_on_LED('g');
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
