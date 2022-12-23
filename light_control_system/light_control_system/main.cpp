/*
 * light_control_system.cpp
 *
 * Created: 12/22/2022 10:05:31
 * Author : Tanuki
 */ 

#include <avr/io.h>
#include <avr/eeprom.h>  /* Include AVR EEPROM header file */
#include <string.h>
#include <avr/interrupt.h>

#define EEPROM_ADDRESS 0 // address of the password in EEPROM
#define PASSWORD_LENGTH 20   // size of the password
#define LLT1 PA0
#define LLT2 PA1
#define LLT3 PA2
#define LLT4 PA3
#define SECURITY PA4
#define LIGHTS_PORT PORTA



void store_password(char *user_password); //store user password in eeprom
void get_password(char password[PASSWORD_LENGTH]); //retrieve user password form eeprom
void light_on(int pin);  //switch on light connected to pin 
void light_off(int pin); //switch on light connected to pin 
void mcu_init(); //initialize micro controller

int main(void)
{
    
}

void light_on(int pin)
{
	if ()
	{
	} 
	else
	{
	}
}

void store_password(char *user_password)
{
	sei();  // Enable global interrupts
	eeprom_update_block((const void*)user_password, (void*)EEPROM_ADDRESS, strlen(user_password) + 1); //store password
}


void get_password(char password[PASSWORD_LENGTH])
{
	eeprom_read_block((void*)password, (const void*)EEPROM_ADDRESS, PASSWORD_LENGTH);
}