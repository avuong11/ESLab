#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

typedef enum
{
	direction_forward = 0,
	direction_backward = 1
}direction;

/* 
	Sets up the whole motor control system
*/
void Setup_motor_system(void);
/*
	Function that gets called during SysTick interrupt handler
*/
void ADC_callback(void);
/*
	Function callback to adjust PWM
*/
void PWM_control_callback(void);
/*
	Sets up the GPIOs that will be turned on and off to control the motor speed.
*/
void Setup_GPIOs_for_PWM(void);
/*
	Makes one step on the motor connected to PC9
*/
void take_step_PC9(direction dir);
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
