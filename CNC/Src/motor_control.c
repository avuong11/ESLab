/*
	This file is the only file that should need to be edited. 
	
	Pins used by other peripherals include:
	USART_TX = PA9
	USART_RX = PA10
	
	See the precompiler directives in the header file to see the pins used by the motor

*/
#include "motor_control.h"
#include "stm32f0xx_hal.h"
#include "stm_utils.h"

#define GPIO_ADC_X 					GPIOC
#define ADC_X_PIN_NUM 			0
#define ADC_CH_X						ADC_CHSELR_CHSEL10

#define GPIO_ADC_Y 					GPIOC
#define ADC_Y_PIN_NUM 			3
#define ADC_CH_Y						ADC_CHSELR_CHSEL13

#define GPIO_PWM_X					GPIOC
#define PWM_X_PIN_NUM				9
#define GPIO_PWM_DIR_X			GPIOC
#define PWM_DIR_X_PIN_NUM		8

#define GPIO_PWM_Y					GPIOC
#define PWM_Y_PIN_NUM				7
#define GPIO_PWM_DIR_Y			GPIOC
#define PWM_DIR_Y_PIN_NUM		6

volatile int desired_steps_X;
volatile int desired_steps_Y;

void Setup_motor_system()
{
	Setup_ADC();
	setup_USART();
	Setup_GPIOs_for_PWM();
	setup_timer_TIM6();
}

void ADC_callback()
{
	static uint16_t count_a = 0;
	if(100 == count_a++)
	{
	   desired_steps_X = calculate_desired_steps(Read_ADC_X());
		 desired_steps_Y = calculate_desired_steps(Read_ADC_Y());
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
}

void adjust_position_Y()
{
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
		GPIOC->BSRR |= GPIO_BSRR_BS_8; // PC8 on
	}
	else
	{
		GPIOC->BSRR |= GPIO_BSRR_BR_8; // PC8 off
	}
	
	// Send out pulse to move motor one step
	static volatile int count = 0;
	GPIOC->BSRR |= GPIO_BSRR_BS_9; // PC9 on
	while(100 > count++){} // Short delay to give a pulse
	count=0;
	GPIOC->BSRR |= GPIO_BSRR_BR_9; // PC9 off
}

void take_step_Y(direction dir)
{
	// Set direction of motor movement
	if(dir == direction_forward)
	{
		GPIOC->BSRR |= GPIO_BSRR_BS_6; // PC6 on
	}
	else
	{
		GPIOC->BSRR |= GPIO_BSRR_BR_6; // PC6 off
	}
	
	// Send out pulse to move motor one step
	static volatile int count = 0;
	GPIOC->BSRR |= GPIO_BSRR_BS_7; // PC7 on
	while(100 > count++){} // Short delay to give a pulse
	count=0;
	GPIOC->BSRR |= GPIO_BSRR_BR_7; // PC7 off
}

void Setup_GPIOs_for_PWM()
{	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //Enable GPIOC for LEDs
	
	//Set PC6-PC9 to General purpose in/out
	GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER6) | (0x01 << GPIO_MODER_MODER6_Pos);
	GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER7) | (0x01 << GPIO_MODER_MODER7_Pos);
	GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER8) | (0x01 << GPIO_MODER_MODER8_Pos);
	GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER9) | (0x01 << GPIO_MODER_MODER9_Pos);
	
		//Set PC6-PC9 to push-pull 
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6); 
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7); 
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8); 
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_9);
	
	//Set PC6-PC9 to low_speed mode
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6);
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR7);
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR8);
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR9);	
	
		//Set no pull up/down for PC6-PC9
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9);
	
	// Initialize GPIO states
	GPIOC->BSRR |= GPIO_BSRR_BR_6;
	GPIOC->BSRR |= GPIO_BSRR_BR_7;
	GPIOC->BSRR |= GPIO_BSRR_BR_8;
	GPIOC->BSRR |= GPIO_BSRR_BR_9;
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

int Read_ADC_X()
{
	// Disable all input channels
	ADC1->CHSELR &= ~ADC_CHSELR_CHSEL;
	// Select/Enable the input channel 10 for PC0
	ADC1->CHSELR |= ADC_CHSELR_CHSEL10;
	ADC1->CR |= ADC_CR_ADSTART;
	return ADC1->DR;
}

int Read_ADC_Y()
{
		// Disable all input channels
	ADC1->CHSELR &= ~ADC_CHSELR_CHSEL;
	
	// Select/Enable the input channel 10 for PC3
	ADC1->CHSELR |= ADC_CHSELR_CHSEL13;
	ADC1->CR |= ADC_CR_ADSTART;
	return ADC1->DR;
}
