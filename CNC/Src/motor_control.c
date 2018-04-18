#include "motor_control.h"
#include "stm32f0xx_hal.h"
#include "stm_utils.h"

void Setup_motor_system()
{
	initLeds();
	Setup_ADC();
	setup_USART();
	button_init();
	setup_timer_TIM6();
}

void SysTick_callback()
{
	static uint16_t count_a = 0;
	count_a++;
	if(count_a == 1000){
		//(Read_ADC_PC0() > 64) ? turn_on_LED('o') : turn_off_LED('o');
		(Read_ADC_PC0() > 200) ? turn_on_LED('b') : turn_off_LED('b');	
		//(Read_ADC_PC3() > 64) ? turn_on_LED('g') : turn_off_LED('g');	
		(Read_ADC_PC3() > 200) ? turn_on_LED('r') : turn_off_LED('r');
		count_a = 0;
	}
}

void PWM_control_callback(void)
{
	(void)toggle_LED('g');
}

void Setup_GPIOs_for_PWM()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //Enable GPIOC for LEDs
	
	//Set PC1-PC2 to General purpose in/out
	GPIOC->MODER |= 0x01 << GPIO_MODER_MODER1_Pos;
	GPIOC->MODER |= 0x01 << GPIO_MODER_MODER2_Pos;
	
		//Set PC1-PC2 to push-pull 
	GPIOC->OTYPER &= ~(0x3 << 1); //Clear PC6-PC9 for push-pull mode 
	
	//Set PC1-PC2 to low_speed mode
	GPIOC->OSPEEDR &= ~(0x3 << GPIO_OSPEEDR_OSPEEDR1_Pos); // Clear PC1-PC2 for low-speed mode
	
		//Set no pull up/down for PC1-PC2
	GPIOC->PUPDR &= ~(0x3 << GPIO_PUPDR_PUPDR1_Pos);
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
