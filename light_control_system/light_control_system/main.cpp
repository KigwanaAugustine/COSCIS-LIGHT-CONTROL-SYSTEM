/*
 * light_control_system.cpp
 *
 * Created: 12/22/2022 10:05:31
 * Author : Tanuki
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/eeprom.h>  /* Include AVR EEPROM header file */
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <ctype.h>

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

char rx_buffer[128];  // Buffer to store received string
int  rx_index = 0;  // Index to store received characters


void store_password(char *user_password); 
void store_username(char *username); 
void get_password(char password[PASSWORD_LENGTH]); 
void get_username(char username[PASSWORD_LENGTH]);
void light_on(int pin);  
void light_off(int pin); 
void mcu_init(); 
void USART_Transmit_String(char* string);
void USART_Transmit(unsigned char data);
void manual_light_switch(); //switch lights on and off manually
int  switch_pressed(int pin);
int check_light_intensity();
int detect_motion();
void login();
void register_user();
void USART_send_byte(unsigned char data);
void USART_send_array(char *array, int array_length);
char * receive_UDR_string();
void switch_on();
void switch_off();


int main(void)
{
	mcu_init(); //initialize microcontroller
	
    while (1)
	{
		//manual_light_switch();

		if(check_light_intensity())  //if darkness is detected, turn on security light
		{
			light_on(SECURITY);
		}
		else
		{
			light_off(SECURITY);
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


ISR(USART_RXC_vect) {
	
	light_on(SECURITY);
	
	char flag = UDR;  // Read the received character
	
	

	switch (flag)
	{
	case 'L': //login user 
		login();
		break;
	
	case 'R': //register user
		register_user();
		break;

	case 'S': //switch on light
		switch_on();
		break;

	case 'O': //switch off light
		switch_off();
		break;

	case 'D': //open door
		//open_door();
		break;
	
	case 'C': //reconfigure 
		//configure();
		break;
	
	default:
		break;
	}
	
	light_off(SECURITY);
	
}


//function that switches off desired light using mobile app
void switch_off()
{
	char  room_light = *(receive_UDR_string());
	
	if (room_light == '1')
	{
		light_off(LLT1);
	}
	else if (room_light == '2')
	{
		light_off(LLT2);
	}
	else if (room_light == '3')
	{
		light_off(LLT3);
	}
	else if (room_light == '4')
	{
		light_off(LLT4);
	}
	else if (room_light == '5')
	{
		light_off(SECURITY);
	}
}


//function that switches on desired light using mobile app
void switch_on()
{

	char  room_light = *(receive_UDR_string());
	
	
	if (room_light == '1')
	{
		light_on(LLT1);
		_delay_ms(100);
		light_off(LLT1);
	}
	else if (room_light == '2')
	{
		light_on(LLT2);
		_delay_ms(100);
		light_off(LLT2);
	}
	else if (room_light == '3')
	{
		light_on(LLT3);
		_delay_ms(100);
		light_off(LLT3);
	}
	else if (room_light == '4')
	{
		light_on(LLT4);
		_delay_ms(100);
		light_off(LLT4);
	}
	else if (room_light == '5')
	{
		light_on(SECURITY);
		_delay_ms(100);
		light_off(SECURITY);
	}
	
	
}


//function receives incoming characters from UDR to and returns the combined formed string
char * receive_UDR_string()
{
	for(int i = 0; i < 128; i++)// we don't expect to receive a string of more than 128 characters 
	{
		
		
		while(! (UCSRA & (1<<RXC))); //wait for entire character to be received in UDR
		
		
		
		char received_char = UDR; //we shall only get the correct character if the speeds are well synchronized

		if ( isspace(received_char) )
		{
			rx_buffer[rx_index] = '\0';  // Add null terminator to the string
			rx_index = 0;  // Reset the index
		
			return rx_buffer;
		}
		else
		{
			// Otherwise, store the received character in the buffer
			rx_buffer[rx_index] = received_char;
			rx_index++;
		}
		
	}
	
}

void register_user()
{

	char * password = receive_UDR_string();
	//char * username = receive_UDR_string();
	
	int result = strcmp(password , "august");
	
	if (result == 0) //D
	{
		light_on(LLT1);
		_delay_ms(500);
		
	}

	char buf[128] ; //D

	get_password(buf); //D

	if ( strcmp(buf , "august") == 0 ) //D
	{
		light_off(LLT1);
	}
	
	store_password(password);
	//store_username(username);

}

void USART_send_array(char *array, int array_length) {
  int i;
  for (i = 0; i < array_length; i++) {
    USART_send_byte(array[i]);
  }
}

void USART_send_byte(unsigned char data) {
  // Wait for empty transmit buffer
  while ( !( UCSRA & (1<<UDRE)) );
  // Put data into buffer, sends the data
  UDR = data;
}

//sends the password and username for verification to the mobile app at login
void login()
{
	char password[PASSWORD_LENGTH];
	char username[PASSWORD_LENGTH];

	get_password(password);
	get_username(username);

	// Enable transmitter
	UCSRB |= (1 << TXEN);

	//TO-DO , convert the arrays password and username into strings before sending them

	//transmit both string and password
	USART_Transmit_String(password);
	USART_Transmit_String(username);
}


//initialize micro controller
void mcu_init()
{
	DDRB=0xff;
	DDRA = 0xFF;//set lights port as output
	DDRC = 0x00;//set port C as input
	
	LIGHTS_PORT |= (1 << PIR); // Enable pull-up resistor
	
	UBRRL = 0X67; //Set baud rate to 9600 for 16MHz clock 
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0) | (URSEL); // Set frame format: 8 data bits, 1 stop bit
	UCSRB =(1<<RXEN)|(1<<TXEN)| (1 << RXCIE); //Enable serial reception and transmission
	sei();  // Enable global interrupts

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



void USART_Transmit(unsigned char data)
{
	// Wait for empty transmit buffer
	while (!(UCSRA & (1 << UDRE)));

	_delay_ms(100);
	
	// Put data into buffer, sends the data
	UDR = data;
}

void USART_Transmit_String(char* string)
{
	while (*string)
	{
		USART_Transmit(*string);
		string++;
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


//store user password in EEPROM
void store_password(char *user_password)
{
	sei();  // Enable global interrupts
	eeprom_update_block((const void*)user_password, (void*)EEPROM_ADDRESS, strlen(user_password) + 1); //store password
}

//store username in EEPROM
void store_username(char *username)
{
	sei();  // Enable global interrupts
	eeprom_update_block((const void*)username, (void*)(EEPROM_ADDRESS + 128), strlen(username) + 1); //store username
} 

//retrieve user password from EEPROM
void get_password(char password[PASSWORD_LENGTH])
{
	eeprom_read_block((void*)password, (const void*)EEPROM_ADDRESS, PASSWORD_LENGTH);
}

//retrieve username from EEPROM
void get_username(char username[PASSWORD_LENGTH])
{
	eeprom_read_block((void*)username, (const void*)(EEPROM_ADDRESS + 128), PASSWORD_LENGTH);
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

/* C CODE TO TRANSMIT STRING via UART atmega32

void usart_init(void)
{
	// Set baud rate to 9600
	UBRRL = 0X67; // for 16MHz clock or 0x33 for 8MHz clock

	// Enable transmitter
	UCSRB = (1 << TXEN); OR UCSRB = (1 << RXEN); // for either transmission or reception
	
	
	// Set frame format: 8 data bits, 1 stop bit
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0) | (1 << URSEL);
}

char USART_receive(void)
{
	// Wait for the entire byte to be received
	while(!(UCSRA & (1<<UDRE) ));
	_delay_ms(100); //random delay
	
	data = UDR;
	
	return data;
	
}


void USART_Transmit(unsigned char data)
{
	// Wait for empty transmit buffer
	while (!(UCSRA & (1 << UDR)));

	_delay_ms(100);
	
	// Put data into buffer, sends the data
	UDR = data;
}

void USART_Transmit_String(char* string)
{
	while (*string)
	{
		USART_Transmit(*string);
		string++;
	}
}

//USE THIS TO RECEIVE A STRING 
	/*char received_char = UDR;  // Read the received character
	if (received_char == '\n') {  // If end of string is received
		rx_buffer[rx_index] = '\0';  // Add null terminator to the string
		rx_index = 0;  // Reset the index
		// Process the received string here
	}
	else {  // Otherwise, store the received character in the buffer
		rx_buffer[rx_index] = received_char;
		rx_index++;
	} */

