/*
	This file is the only file that should need to be edited. 	
	See the precompiler directives in the header file to see the pins used by the motor

*/
#include "motor_control.h"
#include "stm32f0xx_hal.h"
#include "stm_utils.h"

volatile int desired_steps_X;
volatile int desired_steps_Y;

void Setup_motor_system()
{
	setup_USART();
	Setup_GPIOs_for_PWM();
	
	// TODO Calibrate
	
	Setup_ADC();
	setup_timer_TIM6();
}

void ADC_callback()
{
	static uint16_t count_a = 0;
	if(100 == count_a++)
	{
	   desired_steps_X = calculate_desired_steps(Read_ADC_channel(ADC_CH_X));
		 desired_steps_Y = calculate_desired_steps(Read_ADC_channel(ADC_CH_Y));
		 count_a = 0;
	}
}

void PWM_control_callback(void)
{
	adjust_position_X();
	adjust_position_Y();
}

void adjust_position_X()
{
	static uint32_t count = 0;
	static int current_step_count_X = 0;
	if(desired_steps_X > current_step_count_X)
	{
		take_step_X(direction_forward);
		current_step_count_X++;
	}
	else if(desired_steps_X < current_step_count_X)
	{
		take_step_X(direction_backward);
		current_step_count_X--;
	}
	else
	{
		//If desired_steps == current_step_count, do nothing
	}
	
	if(10000 == count++)
	{
		static const int str_len = 16;
		char str[str_len];
		int_to_str(str, str_len, desired_steps_X);
		writeString("Desired steps X: ");
		writeString(str);
		writeString("\t\t");
		int_to_str(str, str_len, current_step_count_X);
		writeString("Current steps X: ");
		writeString(str);
		writeString("\r\n");
		count = 0;
	}
	
}

void adjust_position_Y()
{
	static uint32_t count = 0;
	static int current_step_count_Y = 0;
	if(desired_steps_Y > current_step_count_Y)
	{
		take_step_Y(direction_forward);
		current_step_count_Y++;
	}
	else if(desired_steps_Y < current_step_count_Y)
	{
		take_step_Y(direction_backward);
		current_step_count_Y--;
	}
	else
	{
		//If desired_steps == current_step_count, do nothing
	}
	
	if(10000 == count++)
	{
		static const int str_len = 16;
		char str[str_len];
		int_to_str(str, str_len, desired_steps_Y);
		writeString("Desired steps Y: ");
		writeString(str);
		writeString("\t\t");
		int_to_str(str, str_len, current_step_count_Y);
		writeString("Current steps Y: ");
		writeString(str);
		writeString("\r\n");
		count = 0;
	}
}

int calculate_desired_steps(int ADC_reading)
{
	//TODO calculate the steps from the ADC_reading
	return ADC_reading;
}

void take_step_X(direction dir)
{
	// Set direction of motor movement
	if(dir == direction_forward)
	{
		TURN_ON(GPIO_PWM_DIR_X, PWM_DIR_X_PIN_NUM);
	}
	else
	{
		TURN_OFF(GPIO_PWM_DIR_X, PWM_DIR_X_PIN_NUM);
	}
	
	// Send out pulse to move motor one step
	static volatile int count = 0;
	TURN_ON(GPIO_PWM_X, PWM_X_PIN_NUM);
	while(100 > count++){} // Short delay to give a pulse
	count=0;
	TURN_OFF(GPIO_PWM_X, PWM_X_PIN_NUM); 
}

void take_step_Y(direction dir)
{
	// Set direction of motor movement
	if(dir == direction_forward)
	{
		TURN_ON(GPIO_PWM_DIR_Y, PWM_DIR_Y_PIN_NUM);
	}
	else
	{
		TURN_OFF(GPIO_PWM_DIR_Y, PWM_DIR_Y_PIN_NUM);
	}
	
	// Send out pulse to move motor one step
	static volatile int count = 0;
	TURN_ON(GPIO_PWM_Y, PWM_Y_PIN_NUM);
	while(100 > count++){} // Short delay to give a pulse
	count=0;
	TURN_OFF(GPIO_PWM_Y, PWM_Y_PIN_NUM); 
}

void Setup_GPIOs_for_PWM()
{	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //Enable GPIOC for LEDs
	
	//Set PC6-PC9 to General purpose in/out
	GPIO_PWM_X->MODER = (GPIO_PWM_X->MODER & ~(0x3 << (2 * PWM_X_PIN_NUM)))
											| (0x01 << (2 * PWM_X_PIN_NUM));
	GPIO_PWM_DIR_X->MODER = (GPIO_PWM_DIR_X->MODER & ~(0x3 << (2 * PWM_DIR_X_PIN_NUM))) 
													| (0x01 << (2 * PWM_DIR_X_PIN_NUM));
	GPIO_PWM_Y->MODER = (GPIO_PWM_Y->MODER & ~(0x3 << (2 * PWM_Y_PIN_NUM))) 
											| (0x01 << (2 * PWM_Y_PIN_NUM));
	GPIO_PWM_DIR_Y->MODER = (GPIO_PWM_DIR_Y->MODER & ~(0x3 << (2 * PWM_DIR_Y_PIN_NUM))) 
													| (0x01 << (2 * PWM_DIR_Y_PIN_NUM));
	
		//Set PC6-PC9 to push-pull 
	GPIO_PWM_X->OTYPER &= ~(0x1 << PWM_X_PIN_NUM); 
	GPIO_PWM_DIR_X->OTYPER &= ~(0x1 << PWM_DIR_X_PIN_NUM); 
	GPIO_PWM_Y->OTYPER &= ~(0x1 << PWM_Y_PIN_NUM); 
	GPIO_PWM_DIR_Y->OTYPER &= ~(0x1 << PWM_DIR_Y_PIN_NUM);
	
	//Set PC6-PC9 to low_speed mode
	GPIO_PWM_X->OSPEEDR &= ~(0x3 << (2 * PWM_X_PIN_NUM));
  GPIO_PWM_DIR_X->OSPEEDR &= ~(0x3 << (2 * PWM_DIR_X_PIN_NUM));
  GPIO_PWM_Y->OSPEEDR &= ~(0x3 << (2 * PWM_Y_PIN_NUM));
  GPIO_PWM_DIR_Y->OSPEEDR &= ~(0x3 << (2 * PWM_DIR_Y_PIN_NUM));	
	
		//Set no pull up/down for PC6-PC9
	GPIO_PWM_X->PUPDR &= ~(0x3 << (2 * PWM_X_PIN_NUM));
	GPIO_PWM_DIR_X->PUPDR &= ~(0x3 << (2 * PWM_DIR_X_PIN_NUM));
	GPIO_PWM_Y->PUPDR &= ~(0x3 << (2 * PWM_Y_PIN_NUM));
	GPIO_PWM_DIR_Y->PUPDR &= ~(0x3 << (2 * PWM_DIR_Y_PIN_NUM));	
	
	// Initialize GPIO states
	TURN_OFF(GPIO_PWM_X, PWM_X_PIN_NUM);
	TURN_OFF(GPIO_PWM_DIR_X, PWM_DIR_X_PIN_NUM);
	TURN_OFF(GPIO_PWM_Y, PWM_Y_PIN_NUM);
	TURN_OFF(GPIO_PWM_DIR_Y, PWM_DIR_Y_PIN_NUM);
}

void Setup_ADC()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //Enable GPIOC for ADC. Use PC0 for ADC input
	
	//Set pins to General purpose in/out
	GPIO_ADC_X->MODER |= (0x3 << (2 * ADC_X_PIN_NUM));
	GPIO_ADC_Y->MODER |= (0x3 << (2 * ADC_Y_PIN_NUM));
	
	
	//Set no pull up/down
	GPIO_ADC_X->PUPDR &= ~(0x3 << (2 * ADC_X_PIN_NUM));
	GPIO_ADC_Y->PUPDR &= ~(0x3 << (2 * ADC_Y_PIN_NUM));
	
	
	// Enable ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	
	// Set resolution to 8-bit
	ADC1->CFGR1 &= ~ADC_CFGR1_RES;
	ADC1->CFGR1 |= ADC_CFGR1_RES_1;
	
	// Clear Cont. Conversion mode
	ADC1->CFGR1 &= ~ADC_CFGR1_CONT;
	ADC1->CFGR1 |= ADC_CFGR1_DISCEN;
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;
	
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

int Read_ADC_channel(uint32_t channel)
{
	int ret_val = 0;
	// Disable all input channels
	ADC1->CHSELR &= ~ADC_CHSELR_CHSEL;
		
	// Select/Enable the input channel for the X direction
	ADC1->CHSELR |= channel;
	ADC1->CR |= ADC_CR_ADSTART; // Start read
	while(!(ADC1->ISR & ADC_ISR_EOSEQ)); // Wait for read to end
	ret_val = ADC1->DR;
	ADC1->CHSELR &= ~ADC_CHSELR_CHSEL;
	return ret_val;
}
