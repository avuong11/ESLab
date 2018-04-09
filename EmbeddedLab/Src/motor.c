/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "motor.h"

volatile int16_t error_integral = 0;    // Integrated error signal

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variables for Debug Viewing (no real purpose to be global otherwise)
 * -------------------------------------------------------------------------------------------------------------
 */
volatile int16_t motor_speed = 0;   // Measured motor speed
volatile uint8_t duty_cycle = 0;    // Output PWM duty cycle
volatile int16_t target_rpm = 0;    // Desired speed target
volatile int8_t adc_value = 0;      // ADC measured motor current
volatile int16_t error = 0;         // Speed error signal
volatile uint8_t Kp = 0;            // Proportional gain
volatile uint8_t Ki = 0;            // Integral gain

// Sets up the entire motor drive system
void motor_init(void) {
    pwm_init();
    encoder_init();
    ADC_init();
}

void init_GPIO_inout(void)
{
	// Setup PA4 for Timer 14 CH1
	GPIOA->MODER |= GPIO_MODER_MODER4_1;
	GPIOA->MODER &= ~(GPIO_MODER_MODER4_0);
	
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_4);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR4);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
	
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4);
	GPIOA->AFR[0] |= (0x4 << GPIO_AFRL_AFSEL4_Pos); // 0100
	
	// Input and Output pins for direction
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  
	GPIOC->MODER |= 0x01 << GPIO_MODER_MODER7_Pos;
  //GPIOC->MODER |= 0x01 << GPIO_MODER_MODER1_Pos;

	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_7);
	//GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_1);
	
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR7);
	//GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR1);
	
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7);
	//GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
	
	GPIOC->BSRR |= GPIO_BSRR_BS_7; 
	//GPIOC->ODR ^= GPIO_ODR_1;
	//GPIOC->ODR ^= GPIO_ODR_8; 
	
}

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void) {

    /// TODO: Set up a pin for H-bridge PWM output (TIMER 14 CH1)

    /// TODO: Set up a few GPIO output pins for direction control

    /// TODO: Initialize one direction pin to high, the other low

    /* Hint: These pins are processor outputs, inputs to the H-bridge
     *       they can be ordinary 3.3v pins.
     *       If you hook up the motor and the encoder reports you are
     *       running in reverse, either swap the direction pins or the
     *       encoder pins. (we'll only be using forward speed in this lab)
     */
	
	  init_GPIO_inout();

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->CR1 = 0;                         // Clear control registers
    TIM14->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM14->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM14->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    TIM14->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM14->PSC = 1;                         // Run timer on 24Mhz
    TIM14->ARR = 1200;                      // PWM at 20kHz
    TIM14->CCR1 = 0;                        // Start PWM at 0% duty cycle

    TIM14->CR1 |= TIM_CR1_CEN;              // Enable timer
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(uint8_t duty) {
    if(duty <= 100) {
        TIM14->CCR1 = ((uint32_t)duty*TIM14->ARR)/100;  // Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

void timer3_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// Pins PB4 and PB5
	GPIOB->MODER |= GPIO_MODER_MODER4_1;
	GPIOB->MODER &= ~(GPIO_MODER_MODER4_0);
	
	GPIOB->MODER |= GPIO_MODER_MODER5_1;
	GPIOB->MODER &= ~(GPIO_MODER_MODER5_0);
	
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_4);
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR4);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
	
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_5);
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR5);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);
	
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL4);
	GPIOB->AFR[0] |= (0x1 << GPIO_AFRL_AFSEL4_Pos); // 0001
	
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL5);
	GPIOB->AFR[0] |= (0x1 << GPIO_AFRL_AFSEL5_Pos); // 0001
	
}

// Sets up encoder interface to read motor speed
void encoder_init(void) {

    /// TODO: Set up encoder input pins (TIMER 3 CH1 and CH2)

		timer3_init();
	
    /* Hint: MAKE SURE THAT YOU USE 5V TOLERANT PINS FOR THE ENCODER INPUTS!
     *       You'll fry the processor otherwise, read the lab to find out why!
     */

    // Set up encoder interface (TIM3 encoder input mode)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CCMR1 = 0;    //Clear control registers
    TIM3->CCER = 0;
    TIM3->SMCR = 0;
    TIM3->CR1 = 0;

    TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    TIM3->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    TIM3->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
    TIM3->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
    // (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
    //  just another option, the mid-bias is a bit simpler to understand though.)
    TIM3->CR1 |= TIM_CR1_CEN;                               // Enable timer


    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    /// TODO: Select PSC and ARR values that give an appropriate interrupt rate

    /* Hint: See section in lab on sampling rate!
     *       Recommend choosing a sample rate that gives 2:1 ratio between encoder value
     *       and target speed. (Example: 200 RPM = 400 Encoder count for interrupt period)
     *       This is so your system will match the lab solution
     */
         TIM6->PSC = 5; // TODO: Change this!
         TIM6->ARR = 29000; // TODO: Change this!

    TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn,2);
}

// Encoder interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void) {
    /* Calculate the motor speed in raw encoder counts
     * Note the motor speed is signed! Motor can be run in reverse.
     * Speed is measured by how far the counter moved from center point
     */
    motor_speed = (TIM3->CNT - 0x7FFF);
    TIM3->CNT = 0x7FFF; // Reset back to center point

    // Call the PI update function
    PI_update();

    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

void ADC_init(void) {

    /// TODO: Configure a pin for ADC input (used for current monitoring)

		// PA0
		//Set PA0 to General purpose in/out
//		GPIOA->MODER |= GPIO_MODER_MODER0;
	
		//Set no pull up/down for PA0
//		GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;
	
	
	
	
	
	
    // Configure ADC to 8-bit continuous-run mode, (asynchronous clock mode)
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    ADC1->CFGR1 = 0;
    ADC1->CFGR1 |= (ADC_CFGR1_CONT);        // Set to continuous mode and 12-bit resolution

    /// TODO: Enable the proper channel for the ADC pin you are using
    ADC1->CHSELR |= ADC_CHSELR_CHSEL0; // Change this!

    ADC1->CR = 0;
    ADC1->CR |= ADC_CR_ADCAL;               // Perform self calibration
    while(ADC1->CR & ADC_CR_ADCAL);         // Delay until calibration is complete

    ADC1->CR |= ADC_CR_ADEN;                // Enable ADC
    while(!(ADC1->ISR & ADC_ISR_ADRDY));    // Wait until ADC ready
    ADC1->CR |= ADC_CR_ADSTART;             // Signal conversion start
}

void PI_update(void) {

    /* Run PI control loop
     *
     * Make sure to use the indicated variable names. This allows STMStudio to monitor
     * the condition of the system!
     *
     * target_rpm -> target motor speed in RPM
     * motor_speed -> raw motor speed in encoder counts
     * error -> error signal (difference between measured speed and target)
     * error_integral -> integrated error signal
     * Kp -> Proportional Gain
     * Ki -> Integral Gain
     * output -> raw output signal from PI controller
     * duty_cycle -> used to report the duty cycle of the system
     * adc_value -> raw ADC counts to report current
     *
     */

    /// TODO: calculate error signal and write to "error" variable
		// Convert encoder speeds to RPM.
		int motor_speed_rpm = motor_speed/2;
		error = target_rpm - motor_speed_rpm;
		//int target_rpm_count = target_rpm * 3200;
		//error = target_rpm_count - motor_speed;
	
    /* Hint: Remember that your calculated motor speed may not be directly in RPM!
     *       You will need to convert the target or encoder speeds to the same units.
     *       I recommend converting to whatever units result in larger values, gives
     *       more resolution.
     */


    /// TODO: Calculate integral portion of PI controller, write to "error_integral" variable
		error_integral = error_integral + Ki * error;

    /// TODO: Clamp the value of the integral to a limited positive range
		if(error_integral < 0){
			error_integral = 0;
		}
		else if(error_integral > 3200){
			error_integral = 3200;
		}

    /* Hint: The value clamp is needed to prevent excessive "windup" in the integral.
     *       You'll read more about this for the post-lab. The exact value is arbitrary
     *       but affects the PI tuning.
     *       Recommend that you clamp between 0 and 3200 (what is used in the lab solution)
     */

    /// TODO: Calculate proportional portion, add integral and write to "output" variable

    int16_t output = (Kp * error + error_integral); // Change this!

    /* Because the calculated values for the PI controller are significantly larger than
     * the allowable range for duty cycle, you'll need to divide the result down into
     * an appropriate range. (Maximum integral clamp / X = 100% duty cycle)
     *
     * Hint: If you chose 3200 for the integral clamp you should divide by 32 (right shift by 5 bits),
     *       this will give you an output of 100 at maximum integral "windup".
     *
     * This division also turns the above calculations into pseudo fixed-point. This is because
     * the lowest 5 bits act as if they were below the decimal point until the division where they
     * were truncated off to result in an integer value.
     *
     * Technically most of this is arbitrary, in a real system you would want to use a fixed-point
     * math library. The main difference that these values make is the difference in the gain values
     * required for tuning.
     */

     /// TODO: Divide the output into the proper range for output adjustment
		output = output/32;

     /// TODO: Clamp the output value between 0 and 100
	if(output < 0){
		output = 0;
	}
	else if(output > 100){
		output = 100;
	}

    pwm_setDutyCycle(output);
    duty_cycle = output;            // For debug viewing

    // Read the ADC value for current monitoring, actual conversion into meaningful units
    // will be performed by STMStudio
    if(ADC1->ISR & ADC_ISR_EOC) {   // If the ADC has new data for us
        adc_value = ADC1->DR;       // Read the motor current for debug viewing
    }
}
