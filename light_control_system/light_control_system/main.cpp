/*
 * light_control_system.cpp
 *
 * Created: 12/22/2022 10:05:31
 * Author : Tanuki
 */ 

#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/eeprom.h>  /* Include AVR EEPROM header file */
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define EEPROM_ADDRESS 0 // address of the password in EEPROM
#define PASSWORD_LENGTH 20   // size of the password

//LIGHTS
#define LLT1 PA0
#define LLT2 PA1
#define LLT3 PA2
#define LLT4 PA3
#define SECURITY PA4

//LIGHT SWITCHES
#define LLT1_SWITCH PC0
#define LLT2_SWITCH PC1
#define LLT3_SWITCH PC2
#define LLT4_SWITCH PC3
#define SECURITY_SWITCH PA4

//SENSOR PINS AND PORTS
#define LDR PC5
#define PIR PC6


//PORTS PORTS AND PINS
#define LIGHTS_PORT PORTA
#define LIGHTS_SWITCH PINC



void store_password(char *user_password); 
void get_password(char password[PASSWORD_LENGTH]); 
void light_on(int pin);  
void light_off(int pin); 
void mcu_init(); 
void manual_light_switch(); //switch lights on and off manually
int  switch_pressed(int pin);
int check_light_intensity();
int detect_motion();

int main(void)
{
	mcu_init();
	
    while (1)
	{
		manual_light_switch();

		if(check_light_intensity())  //if darkness is detected, turn on security light
		{
			light_on(SECURITY);
		}

		if(detect_motion()) 
		{
			light_on(LLT2);
		}
		else
		{
			light_off(LLT2);
		}
		
		
	}
}


//initialize micro controller
void mcu_init()
{
	DDRA = 0xFF;//set lights port as output
	DDRC = 0x00;//set port C as input
	LIGHTS_PORT |= (1 << PIR); // Enable pull-up resistor
}

//return a 1 upon detecting motion else return a 0
int detect_motion()
{
	 if(LIGHTS_SWITCH & (1 << PIR))
        {
            return 1;
        }
	 else
		{
			return 0;
		}
}


//returns 1 for low light intensity and a 0 for high intensity
int check_light_intensity()
{
	if ( (LIGHTS_SWITCH & (1<<LDR)) == 1)
	{
		return 1;
	} 
	else
	{
		return 0;
	}
}

//returns a 1 if switch on a given pin is pressed or else returns a 0
int  switch_pressed(int pin)
{
	if ((LIGHTS_SWITCH & (1<<pin)) == 0)
	{
		return 1;
	} 
	else
	{
		return 0;
	}

	
}





//switch on light connected to pin 
void light_on(int pin)
{
	LIGHTS_PORT |= (1<<pin);
}

//switch on light connected to pin 
void light_off(int pin)
{
	LIGHTS_PORT &= ~(1<<pin);
}


//store user password in eeprom
void store_password(char *user_password)
{
	sei();  // Enable global interrupts
	eeprom_update_block((const void*)user_password, (void*)EEPROM_ADDRESS, strlen(user_password) + 1); //store password
}

//retrieve user password from eeprom
void get_password(char password[PASSWORD_LENGTH])
{
	eeprom_read_block((void*)password, (const void*)EEPROM_ADDRESS, PASSWORD_LENGTH);
}


void manual_light_switch()
{
	
	if (switch_pressed(LLT1_SWITCH))
	{
		light_on(LLT1);
	}
	else
	{
		light_off(LLT1);
	}
	
	
	if(switch_pressed(LLT2_SWITCH))
	{
		light_on(LLT2);
	}
	else
	{
		light_off(LLT2);
	}
	
	if(switch_pressed(LLT3_SWITCH))
	{
		light_on(LLT3);
	}
	else
	{
		light_off(LLT3);
	}
	
	if(switch_pressed(LLT4_SWITCH))
	{
		light_on(LLT4);
	}
	else
	{
		light_off(LLT4);
	}
	
	if(switch_pressed(SECURITY_SWITCH))
	{
		light_on(SECURITY);
	}
	else
	{
		light_off(SECURITY);
	}
}