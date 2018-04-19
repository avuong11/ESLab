#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

typedef enum
{
	direction_forward = 0,
	direction_backward = 1
} direction;

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
	Adjusts the position in the X direction
*/
void adjust_position_X(void);
/*
	Adjusts the position in the Y direction
*/
void adjust_position_Y(void);
/*
	Makes one step on the motor connected to PC9, either forward or backward
*/
void take_step_X(direction dir);
/*
	Makes one step on the motor connected to PC9, either forward or backward
*/
void take_step_Y(direction dir);
/*
	Takes an ADC reading and converts it into the desired steps set point
*/
int calculate_desired_steps(int ADC_reading);
/*
	Sets up and calibrates the ADC peripheral
*/
void Setup_ADC(void);
/*
	Reads the input for the X axis using the ADC
*/
int Read_ADC_X(void);
/*
	Reads the input for the Y axis using the ADC
*/
int Read_ADC_Y(void);

#endif
