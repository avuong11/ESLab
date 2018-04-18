#ifndef STM_UTILS_H
#define STM_UTILS_H

typedef unsigned char bool;
#define true 1
#define false 0
	
void SystemClock_Config(void);

/*
	Sets up the GPIOC pins used for the LEDs and turns the LEDs off.
  Uses PC6-PC9
*/
void initLeds(void);
/*
	Sets up the USART peripheral to run using PA9 and PA10
*/
void setup_USART(void);
/*
	Turns on an LED with a character representation. Returns true
  if valid input, false otherwise. Valid inputs correspond to LED
  colors: R, r, G, g, B, b, O, o 
*/
bool turn_on_LED(char ch);
/*
	Turns off an LED with a character representation. Returns true
  if valid input, false otherwise. Valid inputs correspond to LED
  colors: R, r, G, g, B, b, O, o 
*/
bool turn_off_LED(char ch);
/*
	Toggles an LED with a character representation. Returns true
  if valid input, false otherwise. Valid inputs correspond to LED
  colors: R, r, G, g, B, b, O, o 
*/
bool toggle_LED(char ch);
/*
	Writes a string out the USART cable. Can be used for console debugging
*/
void writeString(char* c);
/*
	Sets up the User Button. Uses PA0.
*/
void  button_init(void);
/*
	Sets up the TIM6 timer peripheral.
*/
void setup_timer_TIM6(void);

#endif
