/*
 * lighting_system_code.c
 *
 * Created: 1/5/2023 15:40:37
 * Author : Tanuki
 */ 


// #define F_CPU 16000000UL
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/eeprom.h>  /* Include AVR EEPROM header file */
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <ctype.h>
#include <stdlib.h>

#define EEPROM_ADDRESS 0 // address of the password in EEPROM
#define PASSWORD_LENGTH 20   // size of the password

//RTC
#define RTC_PORT PORTC 


//ALARM
#define ALARM_PORT PORTD 
#define ALARM_PIN PD2


//DOORS
#define DOOR_PORT PORTB
#define DOOR_PIN  PINB
#define LLT2_DOOR_OPEN_BUTTON   PB1
#define LLT2_DOOR_CLOSE_BUTTON  PB0
#define LLT1_DOOR_OPEN_BUTTON   PB2
#define LLT1_DOOR_CLOSE_BUTTON  PB3
#define LLT2_DOOR_PIN2 PB4
#define LLT2_DOOR_PIN1 PB5
#define LLT1_DOOR_PIN2 PB6
#define LLT1_DOOR_PIN1 PB7
#define STEP_DELAY_MS 10 //time delay between each step
#define NUM_STEPS 23 //number of steps to make motor rotate 90 degrees


//LIGHTS
#define LLT1 PA0
#define LLT2 PA1
#define LLT3 PA2
#define LLT4 PA3
#define SECURITY PA4
#define APP_LED PA5
#define DATA_IN_LED PA6
#define DATA_OUT_LED PA7

//LIGHT SWITCHES
#define LIGHT_SWITCH_PORT
#define LLT1_SWITCH PD7
#define LLT2_SWITCH PD6
#define LLT3_SWITCH PD5
#define LLT4_SWITCH PD4
#define SECURITY_SWITCH PD3

//SENSOR PINS AND PORTS
#define LDR_PORT PORTC
#define LDR PC5
#define LLT1_PIR PC6
#define LLT2_PIR PC7
#define LIGHTS_PORT PORTA
#define PIR_PIN PINC
#define LDR_PIN PINC
#define LIGHTS_SWITCH PIND
#define LIGHTS_BUTTON_PORT PORTD


//FUNCTIONS

void store_password(char *user_password); 
void store_username(char *username); 
void get_password(char password[PASSWORD_LENGTH]); 
void get_username(char username[PASSWORD_LENGTH]);
void light_on(int pin);  
void light_off(int pin); 
void mcu_init(); 
void USART_Transmit_String(char* string);
void USART_Transmit(unsigned char data);
void activate_manual_light_switch(); //switch lights on and off manually
int  switch_pressed(int pin);
int check_light_intensity();
int detect_motion(int PIR_pin);
void login();
void register_user();
void USART_send_byte(unsigned char data);
void USART_send_array(char *array, int array_length);
char * receive_UDR_string();
void switch_on();
void switch_off();
void all_lights_off();
void all_lights_on();
void open_door(int door_pin1, int door_pin2);
void close_door(int door_pin1, int door_pin2);
void activate_manual_doors();
void sound_alarm();
void rtc_getTime(unsigned char *h,unsigned char *m,unsigned char *s);
void rtc_setTime(unsigned char h,unsigned char m,unsigned char s);
void rtc_init(void);
void i2c_stop();
unsigned char i2c_read(unsigned char val);
void i2c_write(unsigned char data);
void i2c_init(void);
void i2c_start(void);
int _night_time();
void door_open();
void door_close();
void activate_switch();
void deactivate_switch();
char* get_current_time();
void eeprom_store_string(char * string); 
void send_stored_timestamps();





		//GLOBAL VARIABLES
char rx_buffer[1000];  // Buffer to store received string
int  rx_index = 0;  // Index to store received characters
int start_address = 300;
int next_address = 300;
int last_address = 1005;
int eeprom_string_length = 15;
char * LLT1_time = " ";
char * LLT2_time = " ";
char * LLT3_time = " ";
char * LLT4_time = " ";
char * SECURITY_time = " ";
char * timer = " ";
int counter = 1;

//-------------------------------------------------MAIN----------------------------------------------- 
int main(void)
{
		
	
	mcu_init(); //initialize microcontroller

	 while (1)
	 {	
		
		if(_night_time())
		{	
			
			activate_manual_doors();
			activate_manual_light_switch(); 

			light_on(SECURITY);

			if(detect_motion(LLT1_PIR)) //if motion is detected in LLT1
			{
				light_on(LLT1);
				close_door(LLT1_DOOR_PIN1, LLT1_DOOR_PIN2);
				DDRB &= ~ ( (1<<LLT1_DOOR_PIN1) | ( (1<<LLT1_DOOR_PIN2)) ); //Deactivate manual opening for LLT1 door
				sound_alarm();   //raise alarm for 10s
			}
			else
			{
				light_off(LLT1); 
			}
			
			
			if(detect_motion(LLT2_PIR)) //if motion is detected in LLT2
			{
				light_on(LLT2);
				close_door(LLT2_DOOR_PIN2, LLT2_DOOR_PIN1);
				DDRB &= ~ ( (1<<LLT2_DOOR_PIN1) | ( (1<<LLT2_DOOR_PIN2)) ); //Deactivate manual opening for LLT2 door
				sound_alarm();   //raise alarm for 10s
			}
			else
			{
				light_off(LLT2);
			}
		}
		else  //DAY TIME
		{

			activate_manual_doors();
			activate_manual_light_switch(); 


			if (check_light_intensity())
			{
				light_on(SECURITY);


				//Motion detected in LLT1 with low light intensity , switch on LLT1 light
				if (detect_motion(LLT1_PIR))
				{
					light_on(LLT1);
				}
				else
				{
					light_off(LLT1);
				}

				//Motion detected in LLT2 with low light intensity , switch on LLT2 light
				if (detect_motion(LLT2_PIR))
				{
					light_on(LLT2);
				}
				else
				{
					light_off(LLT2);
				}


			}
			else
			{
				light_off(SECURITY);
			}
		}
		
		_delay_ms(50);
	 }
	

}


ISR(USART_RXC_vect) {
	
	light_on(APP_LED);
	
	
	char flag = UDR;  // Read the received character
	
	

	switch (flag)
	{
	case 'L': //login user 
		login();
		break;
	
	case 'R': //register user
		register_user();
		break;

	case 'O': //switch on light
		switch_on();
		break;

	case 'F': //switch off light
		switch_off();
		break;

	case 'D': //open door
		door_open();
		break;
	
	case 'C': //close door
		door_close();
		break;

	case 'G': //deactivate room switch
		deactivate_switch();
		break;

	case 'A': //activate room switch
		activate_switch();
		break;

	case 'P': //send reports data
		send_stored_timestamps();
		break;
	
	default:
		break;
	}
	

	light_off(APP_LED);
	
	_delay_ms(200);
	
}

//activate room switch from mobile app
void activate_switch()
{
	char  chosen_switch = *(receive_UDR_string()); //get the choice of switch user would want to activate

	//activate switch by making it output from DDR
	if (chosen_switch == '1') //LLT1
	{
		DDRA |= (1<<LLT1);
	}
	else if (chosen_switch == '2') //LLT2
	{
		DDRA |= (1<<LLT2);
	}
	else if (chosen_switch == '3') //LLT3
	{
		DDRA |= (1<<LLT3);
	}
	else if (chosen_switch == '4') //LLT4
	{
		DDRA |= (1<<LLT4);
	}
	else if (chosen_switch == '5') //LLT5
	{
		DDRA |= (1<<SECURITY);
	}
}


//deactivate room switch from mobile app
void deactivate_switch()
{
	char  chosen_switch = *(receive_UDR_string()); //get the choice of switch user would want to deactivate

	//deactivate switch by making it input from DDR
	if (chosen_switch == '1') //LLT1
	{
		DDRA &= ~(1<<LLT1);
	}
	else if (chosen_switch == '2') //LLT2
	{
		DDRA &= ~(1<<LLT2);
	}
	else if (chosen_switch == '3') //LLT3
	{
		DDRA &= ~(1<<LLT3);
	}
	else if (chosen_switch == '4') //LLT4
	{
		DDRA &= ~(1<<LLT4);
	}
	else if (chosen_switch == '5') //LLT5
	{
		DDRA &= ~(1<<SECURITY);
	}
	
}

//open a door from the mobile app
void door_open()
{
	char  door_choice = *(receive_UDR_string());//get the user's choice of door to open

	if (door_choice == '1')
	{
		DDRB |= ( (1<<LLT1_DOOR_PIN1) | ( (1<<LLT1_DOOR_PIN2)) ); //activate opening/closing for LLT1 door
		open_door(LLT1_DOOR_PIN1, LLT1_DOOR_PIN2);  //open LLT1
	}

	if (door_choice == '2')
	{
		DDRB |= ( (1<<LLT2_DOOR_PIN1) | ( (1<<LLT2_DOOR_PIN2)) ); //activate  opening/closing for LLT2 door
		open_door(LLT2_DOOR_PIN2, LLT2_DOOR_PIN1);  //open LLT2
	}
	
}

//close a door from the mobile app
void door_close()
{
	char  door_choice = *(receive_UDR_string());//get the user's choice of door to close

	if (door_choice == '1')
	{
		DDRB |= ( (1<<LLT1_DOOR_PIN1) | ( (1<<LLT1_DOOR_PIN2)) ); //activate opening/closing for LLT1 door
		close_door(LLT1_DOOR_PIN1, LLT1_DOOR_PIN2);  //close LLT1
	}

	if (door_choice == '2')
	{
		DDRB |= ( (1<<LLT2_DOOR_PIN1) | ( (1<<LLT2_DOOR_PIN2)) ); //activate  opening/closing for LLT2 door
		close_door(LLT2_DOOR_PIN2, LLT2_DOOR_PIN1);  //close LLT2
	}
	
}

//function returns the current time as a string hhmmss
char* get_current_time()
{

    char digit1=0;
	 char digit2=0;
	 char digit3=0;
	 char digit4=0;
	 char digit5=0;
	 char digit6=0;

	 char i,j,k;

     char time[7] ;

    rtc_getTime(&i,&j,&k);

	// HOURS
	digit1=((i >> 4)+'0');
	
	_delay_ms(1);
	
	digit2=((i & 0x0F)+'0');
	
	// MINUTES
	digit3=((j >> 4)+'0');

	_delay_ms(1);
	
	digit4=((j & 0x0F)+'0');
	
	// seconds
	digit5=((k >> 4)+'0');
	
	_delay_ms(1);
	
	digit6=((k & 0x0F)+'0');



    time[0] = digit1;
    time[1] = digit2;
    time[2] = digit3;
    time[3] = digit4;
    time[4] = digit5;
    time[5] = digit6;
    time[6] = '\0';

	timer = time;

   // USART_Transmit_String(timer);
	_delay_ms(10); //random delay

	return time;

}

//returns a 1 if it's night time(21:30 to 06:59:59) else, returns a 0
int _night_time()
{
	
	unsigned char digit1=0;
	unsigned char digit2=0;
	unsigned char digit3=0;
	unsigned char digit4=0;
	unsigned char digit5=0;
	unsigned char digit6=0;

	unsigned char i,j,k;

    unsigned char time[7] ;

    rtc_getTime(&i,&j,&k);

	// HOURS
	digit1=((i >> 4)+'0');
	
	_delay_ms(1);
	
	digit2=((i & 0x0F)+'0');
	
	// MINUTES
	digit3=((j >> 4)+'0');

	_delay_ms(1);
	
	digit4=((j & 0x0F)+'0');
	
	// seconds
	digit5=((k >> 4)+'0');
	
	_delay_ms(1);
	
	digit6=((k & 0x0F)+'0');



    time[0] = digit1;
    time[1] = digit2;
    time[2] = digit3;
    time[3] = digit4;
    time[4] = digit5;
    time[5] = digit6;
    time[6] = '\0';

    unsigned char * night_time = "213000";
    unsigned char * midnight = "235959";

    if( (( (strcmp(time, night_time)) > 0) && (strcmp(time, midnight) < 0) ) || (((strcmp(time, "000000") > 0) && strcmp(time, "065959") < 0)) || (strcmp(time, "000000") == 0) || (strcmp(time, night_time) == 0))
    {
        return 1;
    }
    else
    {
        return 0;
    }

}

//initializes communication with the RTC
void i2c_init(void)
{
	TWSR = 0X00;
	TWBR = 0X48; //bit rate register, set clock to 100Khz crystal frequency= 16Mhz
	TWCR = 0X04;
}

//starts the communication with RTC
void i2c_start(void)
{
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while((TWCR & (1<<TWINT)) == 0);
}

//writes time from the RTC
void i2c_write(unsigned char data)
{
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while((TWCR & (1<<TWINT)) == 0);
}

//reads data from the RTC
unsigned char i2c_read(unsigned char val)
{
	TWCR = (1<<TWINT)
	| (1<<TWEN) | (val<<TWEA);
	while((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}

//halts communication
void i2c_stop()
{
	int x;
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	for(x=0;x<100;x++); //wait for seconds
}

//initializes the RTC
void rtc_init(void)
{
	i2c_init();
	i2c_start();
	i2c_write(0XD0);
	i2c_write(0X07); //command register address
	i2c_write(0X00);
	i2c_stop();
}

//sets RTC time
void rtc_setTime(unsigned char h,unsigned char m,unsigned char s)
{
	i2c_start();
	i2c_write(0XD0);
	i2c_write(0X00);
	i2c_write(s);
	i2c_write(m);
	i2c_write(h);
	i2c_stop();
}

//gets current time from RTC
void rtc_getTime(unsigned char *h,unsigned char *m,unsigned char *s)
{
	i2c_start();
	i2c_write(0XD0);
	i2c_write(0X00);
	i2c_stop();

	i2c_start();
	i2c_write(0XD1);
	*s = i2c_read(1);
	*m = i2c_read(1);
	*h = i2c_read(0);
	i2c_stop();
}


//Sounds alarm for 10s
void sound_alarm()
{

	for(int i = 0; i < 4; i++)

	{
		ALARM_PORT |= (1<<ALARM_PIN);
		_delay_ms(100);


		ALARM_PORT &= ~(1<<ALARM_PIN);
		_delay_ms(100);
	}

	

}


void open_door(int door_pin1, int door_pin2)
{

	for(int i = 0; i < NUM_STEPS ; i++)
	{
		//set the forward pin to high and the backward pin to low
		DOOR_PORT |= (1<<door_pin1); 
	    DOOR_PORT &= ~(1<<door_pin2); 
		//Delay for a short time
		_delay_ms(STEP_DELAY_MS);
	}

	//Stop the mortor by setting both control pins to low
	DOOR_PORT &= ~(1<<door_pin1);
	DOOR_PORT &= ~(1<<door_pin2);
}

void close_door(int door_pin1, int door_pin2)
{
	for(int i = 0; i < NUM_STEPS ; i++)
	{
		//set the forward pin to high and the backward pin to low
		DOOR_PORT |= (1<<door_pin2); 
		DOOR_PORT &= ~(1<<door_pin1); 

		//Delay for a short time
		_delay_ms(STEP_DELAY_MS);
	}

	//Stop the mortor by setting both control pins to low
	DOOR_PORT &= ~(1<<door_pin1);
	DOOR_PORT &= ~(1<<door_pin2);
}


void activate_manual_doors()
{
    //opening doors
		//LLT1
	if (! ((DOOR_PIN) & (1<<LLT1_DOOR_OPEN_BUTTON)) )
	{
		open_door(LLT1_DOOR_PIN1, LLT1_DOOR_PIN2);
	}
		//LLT2
	if (! ((DOOR_PIN) & (1<<LLT2_DOOR_OPEN_BUTTON)) )
	{
		open_door(LLT2_DOOR_PIN1, LLT2_DOOR_PIN2);
	}

	//closing door
		//LLT1
	if ( !((DOOR_PIN) & (1<<LLT1_DOOR_CLOSE_BUTTON)) )
	{
		close_door(LLT1_DOOR_PIN1, LLT1_DOOR_PIN2);
	}
		//LLT2
	if ( !((DOOR_PIN) & (1<<LLT2_DOOR_CLOSE_BUTTON)) )
	{
		close_door(LLT2_DOOR_PIN1, LLT2_DOOR_PIN2);
	}
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
		
		if ((DDRA & (1<<LLT1))) //if room light active
		{
			light_on(LLT1);
		}
		else //if room light was deactivated
		{
			DDRA |= (1<<LLT1); 	//activate it 
			light_on(LLT1);    //switch on light
			_delay_ms(100);
			DDRA &= ~(1<<LLT1);  	//deactivate it again 
		}
		
	
	}
	else if (room_light == '2')
	{
		if ((DDRA & (1<<LLT2))) //if room light active
		{
			light_on(LLT2);
		}
		else //if room light was deactivated
		{
			DDRA |= (1<<LLT2); 	//activate it 
			light_on(LLT2);    //switch on light
			_delay_ms(100);
			DDRA &= ~(1<<LLT2);  	//deactivate it again 
		}
	}
	else if (room_light == '3')
	{
		if ((DDRA & (1<<LLT3))) //if room light active
		{
			light_on(LLT3);
		}
		else //if room light was deactivated
		{
			DDRA |= (1<<LLT3); 	//activate it 
			light_on(LLT3);    //switch on light
			_delay_ms(100);
			DDRA &= ~(1<<LLT3);  	//deactivate it again 
		}
	}
	else if (room_light == '4')
	{
		if ((DDRA & (1<<LLT4))) //if room light active
		{
			light_on(LLT4);
		}
		else //if room light was deactivated
		{
			DDRA |= (1<<LLT4); 	//activate it 
			light_on(LLT4);    //switch on light
			_delay_ms(100);
			DDRA &= ~(1<<LLT4);  	//deactivate it again 
		}
		
	}
	else if (room_light == '5')
	{
		if ((DDRA & (1<<SECURITY))) //if room light active
		{
			light_on(SECURITY);
		}
		else //if room light was deactivated
		{
			DDRA |= (1<<SECURITY); 	//activate it 
			light_on(SECURITY);    //switch on light
			_delay_ms(100);
			DDRA &= ~(1<<SECURITY);  	//deactivate it again 
		}
	}
	
	
}


//function receives incoming characters from UDR to and returns the combined formed string
char * receive_UDR_string()
{

	for(int i = 0; i < 128; i++)// we don't expect to receive a string of more than 128 characters 
	{
		
		while(! (UCSRA & (1<<RXC))); //wait for entire character to be received in UDR
		
		char received_char = UDR; //we shall only get the correct character if the speeds are well synchronized

	    light_on(DATA_IN_LED);

		if ( isspace(received_char) )
		{
			rx_buffer[rx_index] = '\0';  // Add null terminator to the string
			rx_index = 0;  // Reset the index

			light_off(DATA_IN_LED);
			
			return rx_buffer;
		}
		else
		{
			// Otherwise, store the received character in the buffer
			rx_buffer[rx_index] = received_char;
			rx_index++;
		}

	}
	
	light_off(DATA_IN_LED);	
	
}

void register_user()
{
	//receive password and store it in eeprom 
	char * username = receive_UDR_string();
	store_username(username);

	//receive username and store it in eeprom 
	char * password = receive_UDR_string();
	store_password(password);
	
}


//sends the password and username for verification to the mobile app at login
void login()
{
	char password[PASSWORD_LENGTH];
	char username[PASSWORD_LENGTH];

	//retrieve username and password
	get_password(password);
	get_username(username);


	//transmit both username and password
	USART_Transmit_String(username);
	USART_Transmit_String(password);
	
}


//initialize micro controller
void mcu_init()
{
	//PORTA INIT
	DDRA = 0xFF; //set as output (LEDS CONNECTED)

	//PORTB INIT
	DDRB = 0XF0;//first 4 bits are for  input(motor buttons) , last 4 bits for output(motors)
		//activate button internal pull up resistors
	PORTB |= (1 << LLT1_DOOR_OPEN_BUTTON);
	PORTB |= (1 << LLT1_DOOR_CLOSE_BUTTON);
	PORTB |= (1 << LLT2_DOOR_OPEN_BUTTON);
	PORTB |= (1 << LLT2_DOOR_CLOSE_BUTTON);

	//PORTC INIT
	DDRC &= ~((1<<LDR) | (1 << LLT1_PIR) | (1 << LLT2_PIR) ); //set sensor pins as input


	

		//activate button internal pull up resistors
	LIGHTS_BUTTON_PORT |= (1 << LLT1_SWITCH);
	LIGHTS_BUTTON_PORT |= (1 << LLT2_SWITCH);
	LIGHTS_BUTTON_PORT |= (1 << LLT3_SWITCH);
	LIGHTS_BUTTON_PORT |= (1 << LLT4_SWITCH);
	LIGHTS_BUTTON_PORT |= (1 << SECURITY_SWITCH);
	
	
	//PORTD INIT
	DDRD  &= ~((1<<LLT1_SWITCH)|(1<<LLT2_SWITCH)|(1<<LLT3_SWITCH)|(1<<LLT4_SWITCH)|(1<<SECURITY_SWITCH));
	DDRD |= (1<<ALARM_PIN); //set alarm pin as output 
	
	//USART INIT
	UBRRL = 0X67; //Set baud rate to 9600 for 16MHz clock and we have set U2X = 0 for normal speed
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0) | (URSEL); // Set frame format: 8 data bits, 1 stop bit
	UCSRB =(1<<RXEN)|(1<<TXEN)| (1 << RXCIE); //Enable serial reception and transmission, activating interrupt upon serial reception
	sei();  // Enable global interrupts
    //UCSRA |= (1 << U2X); // Set the U2X bit in the UCSRA register to double transmission speed


   //RTC INITIALIZATION
   rtc_init();
   rtc_setTime(0X23, 0X20, 0X59);

}

//return a 1 upon detecting motion else return a 0
int detect_motion(int PIR_pin)
{
	 if(PIR_PIN & (1 << PIR_pin))
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
	if ( (LDR_PIN & (1<<LDR)) == 0)
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
		light_on(DATA_OUT_LED);

		USART_Transmit(*string);

		light_off(DATA_OUT_LED);

		string++;
	}
	
}


//switch on light connected to pin 
void light_on(int pin)
{

	//check if light is already on , then don't execute this code
	if( (LIGHTS_PORT & (1<<pin)) )
	{
		return;
	}
	
	else
	{
		
		LIGHTS_PORT |= (1<<pin); //switch on light

		//no records should be kept about these leds
		if( (pin != APP_LED) & (pin != DATA_IN_LED) & (pin != DATA_OUT_LED) )
		{
			
			//convert pin to string
			char str[100];  //the 100 could cause potential error
			itoa(pin, str, 10);

			//check which light is being turn on
			if (pin == LLT1)
			{
				get_current_time();
				LLT1_time = str; 
				strcat(LLT1_time, timer);

				//D
				// USART_Transmit_String(" ST ");
				// USART_Transmit_String(LLT1_time);
			}
			else if(pin == LLT2)
			{
				get_current_time();
				LLT2_time = str; 
				strcat(LLT2_time, timer);
			}
			else if(pin == LLT3)
			{
				get_current_time();
				LLT3_time = str; 
				strcat(LLT3_time, timer);
			}
			else if(pin == LLT4)
			{
				get_current_time();
				LLT4_time = str; 
				strcat(LLT4_time, timer);
			}
			else if(pin == SECURITY)
			{
				get_current_time();
				SECURITY_time = str; 
				strcat(SECURITY_time, timer);
			}
			
		}
	}


}

//switch on light connected to pin 
void light_off(int pin)
{
	//if light is already off, don't switch off again
	if (!(LIGHTS_PORT & (1<<pin)))
	{
		return;
	}
	else
	{
			/* code */
		LIGHTS_PORT &= ~(1<<pin);

		
		//no records should be kept about these leds
		if( (pin != APP_LED) & (pin != DATA_IN_LED) & pin != DATA_OUT_LED)
		{
			//concatenate end time to the time string and store to eeprom
			if(pin == LLT1)
			{
				get_current_time(); //change value of timer 
				strcat(LLT1_time, timer);
				//  USART_Transmit_String(" LLT1_str_2 ");
				//  USART_Transmit_String(LLT1_time);
				eeprom_store_string(LLT1_time); 
			}
			else if (pin == LLT2)
			{
				get_current_time(); //change value of timer 
				strcat(LLT2_time, timer);
				//  USART_Transmit_String(" LLT1_str_2 ");
				//  USART_Transmit_String(LLT1_time);
				eeprom_store_string(LLT2_time); 
			}
			else if (pin == LLT3)
			{
				get_current_time(); //change value of timer 
				strcat(LLT3_time, timer);
				//  USART_Transmit_String(" LLT1_str_2 ");
				//  USART_Transmit_String(LLT1_time);
				eeprom_store_string(LLT3_time); 
			}
			else if (pin == LLT4)
			{
				get_current_time(); //change value of timer 
				strcat(LLT4_time, timer);
				//  USART_Transmit_String(" LLT1_str_2 ");
				//  USART_Transmit_String(LLT1_time);
				eeprom_store_string(LLT4_time); 
			}
			else if (pin == SECURITY)
			{
				get_current_time(); //change value of timer 
				strcat(SECURITY_time, timer);
				//  USART_Transmit_String(" LLT1_str_2 ");
				//  USART_Transmit_String(LLT1_time);
				eeprom_store_string(SECURITY_time); 
			}
			
		} 
	}
	
}

//store string in EEPROM at the given address
void eeprom_store_string(char * string)
{
	// USART_Transmit_String("str ");
	// USART_Transmit_String(string);

	//check if eeprom full and reset storage address
    if(next_address > last_address)
    {
        next_address = start_address; //reset next_address
	    eeprom_update_block((const void*)string, (void*)(next_address), strlen(string) + 1); //store string
    }
    else
    {
	    eeprom_update_block((const void*)string, (void*)(next_address), strlen(string) + 1); //store string
    }
    
	next_address += (eeprom_string_length + 1); //increment next address
}


//store user password in EEPROM
void store_password(char *user_password)
{
	eeprom_update_block((const void*)user_password, (void*)EEPROM_ADDRESS, strlen(user_password) + 1); //store password
}

//store username in EEPROM
void store_username(char *username)
{
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

//retrieves string from eeprom address passed to it
void retrieve_eeprom_string(char str[eeprom_string_length + 1], int address)
{
	eeprom_read_block((void*)str, (const void*)(address), eeprom_string_length);
}


//SENDING ALL THE TIMESTAMPS
void send_stored_timestamps()
{
    char str[eeprom_string_length + 1]; //buffer to store retrieved eeprom string
    
	//retrieve all time strings stored in eeprom and send them to mobile app
    for(int address = start_address; address < next_address; address += (eeprom_string_length + 1)) //has null character been considered
    {   
        retrieve_eeprom_string(str, address);
        USART_Transmit_String(str);
		USART_Transmit_String("*");
    }

	next_address = start_address; //reset address to overwrite sent reports
    
}

void activate_manual_light_switch()
{
	
	// if (switch_pressed(LLT1_SWITCH))
	// {
	// 	light_on(LLT1);
	// }
	// else
	// {
	// 	light_off(LLT1);
	// }
	
	
	// if(switch_pressed(LLT2_SWITCH))
	// {
	// 	light_on(LLT2);
	// }
	// else
	// {
	// 	light_off(LLT2);
	// }
	
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
	
	// if(switch_pressed(SECURITY_SWITCH))
	// {
	// 	light_on(SECURITY);
	// }

	// if(!switch_pressed(SECURITY_SWITCH))
	// {
	// 	light_off(SECURITY);
	// }
	// else
	// {
	// 	light_off(SECURITY);
	// }

}


//turns all lights on
void all_lights_on()
{
	light_on(LLT1);
	light_on(LLT2);
	light_on(LLT3);
	light_on(LLT4);
}

//turns all lights off
void all_lights_off()
{
	light_off(LLT1);
	light_off(LLT2);
	light_off(LLT3);
	light_off(LLT4);
}






