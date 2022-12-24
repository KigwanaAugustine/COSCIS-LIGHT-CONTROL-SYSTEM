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

//PORTS PORTS AND PINS
#define LIGHTS_PORT PORTA
#define LIGHTS_SWITCH PINC



void store_password(char *user_password); 
void get_password(char password[PASSWORD_LENGTH]); 
void light_on(int pin);  
void light_off(int pin); 
void mcu_init(); 
void manual_light_switch(); //switch lights on and off manually

int main(void)
{
	mcu_init();
	
    while (1)
	{
		
		manual_light_switch();
		
	}
    
}

//initialize micro controller
void mcu_init()
{
	DDRA = 0xFF;//set lights port as output
	DDRC = 0x00;//set port C as input
}

void manual_light_switch()
{
	
	if ( (LIGHTS_SWITCH && (1<<LLT1_SWITCH)) == 0 )
	{
		light_on(LLT1);
	}
	else
	{
		light_off(LLT1);
	} 
	
	
	if ( (LIGHTS_SWITCH && (1<<LLT2_SWITCH)) == 0 )
	{
		light_on(LLT2);
	}
	else
	{
		light_off(LLT2);
	} 
	
	
	if ( (LIGHTS_SWITCH && (1<<LLT3_SWITCH)) == 0 )
	{
		light_on(LLT3);
	}
	else
	{
		light_off(LLT3);
	}
	
	
	
	if ( (LIGHTS_SWITCH && (1<<LLT4_SWITCH)) == 0 )
	{
		light_on(LLT4);
	}
	else
	{
		light_off(LLT4);
	} 

	
	
	if ( (LIGHTS_SWITCH && (1<<SECURITY_SWITCH)) == 0 )
	{
		light_on(SECURITY);
	}
	else
	{
		light_off(SECURITY);
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