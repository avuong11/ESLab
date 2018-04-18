#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
/* 
	Sets up the whole motor control system
*/
void Setup_motor_system(void);
/*
	Function that gets called during SysTick interrupt handler
*/
void SysTick_callback(void);
/*
	Function callback to adjust PWM
*/
void PWM_control_callback(void);
/*
	Sets up the GPIOs that will be turned on and off to control the motor speed.
*/
void Setup_GPIOs_for_PWM(void);
/*
	Sets up and calibrates the ADC peripheral
*/
void Setup_ADC(void);
/*
	Reads the input on PC0 using the ADC
*/
int Read_ADC_PC0(void);
/*
	Reads the input on PC3 using the ADC
*/
int Read_ADC_PC3(void);

#endif
