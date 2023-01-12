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

//RTC
#define RTC_PORT PORTC 


//ALARM
#define ALARM_PORT PORTD 
#define ALARM_PIN PD4


//DOORS
#define DOOR_PORT PORTB
#define DOOR_PIN  PINB
#define LLT1_DOOR_PIN1 PB0
#define LLT1_DOOR_PIN2 PB1
#define LLT2_DOOR_PIN1 PB2
#define LLT2_DOOR_PIN2 PB3
#define LLT2_DOOR_OPEN_BUTTON   PB4
#define LLT2_DOOR_CLOSE_BUTTON  PB5
#define LLT1_DOOR_OPEN_BUTTON   PB6
#define LLT1_DOOR_CLOSE_BUTTON  PB7

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
#define LLT1_SWITCH PC0
#define LLT2_SWITCH PC1
#define LLT3_SWITCH PC2
#define LLT4_SWITCH PC3
#define SECURITY_SWITCH PA4

//SENSOR PINS AND PORTS
#define LDR PC5
#define LLT1_PIR PC6
#define LLT2_PIR PC7


//PORTS PORTS AND PINS
#define LIGHTS_PORT PORTA
#define PIR_PIN PINC
#define LDR_PIN PINC
#define LIGHTS_SWITCH PINC
#define LIGHTS_BUTTON_PORT PORTC

char rx_buffer[1000];  // Buffer to store received string
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


int main(void)
{
		
	
	mcu_init(); //initialize microcontroller

	// unsigned char digit1=0;
	// unsigned char digit2=0;
	// unsigned char digit3=0;
	// unsigned char digit4=0;
	// unsigned char digit5=0;
	// unsigned char digit6=0;
	
	// unsigned char tim1='2';
	// unsigned char tim2='1';
	// unsigned char tim3='3';
	// unsigned char tim4='0';
	// unsigned char tim5='0';
	// unsigned char tim6='0';
	
	// unsigned char i,j,k;

	 while (1)
	 {
		
		if(_night_time())
		{
			light_on(LLT1);
		}
		else
		{
			light_off(LLT1);
		}
		
		_delay_ms(100);
	}
	

    while (!1)
	{
		
		activate_manual_doors();

	
		//turn on security light when low light intensity is detected
		// if(check_light_intensity())
		// {
		// 	light_on(SECURITY);
		// }
		// else
		// {
		// 	light_off(SECURITY);
		// }
		
		// while(!1)  //if night time(9:30pm to 6:59am) 
		// {	
		// 	_delay_ms(20);

		// 	activate_manual_light_switch();

		// 	light_on(SECURITY);


		// 	if(detect_motion(LLT1_PIR)) //if motion is detected in LLT1
		// 	{
		// 		light_on(LLT1);
		// 		close_door(LLT1_DOOR_PIN1, LLT1_DOOR_PIN2);
		// 		sound_alarm();   //raise alarm for 10s
		// 	}
			
		// 	if(detect_motion(LLT2_PIR)) //if motion is detected in LLT2
		// 	{
		// 		light_on(LLT2);
		// 		sound_alarm();   //raise alarm for 10s
		// 		close_door(LLT2_DOOR_PIN1, LLT2_DOOR_PIN2);
		// 	}
			

		// }

		// while(!1)  //if day time(7am to 7pm) 
		// {
		// 	_delay_ms(20);
		// 	activate_manual_doors();
		// 	activate_manual_light_switch(); 

		// 	light_off(SECURITY);

		// 	//Motion detected in LLT1 with low light intensity , switch on LLT1 light
		// 	if (check_light_intensity() & detect_motion(LLT1_PIR))
		// 	{
		// 		light_on(LLT1);
		// 	}
		// 	else
		// 	{
		// 		light_off(LLT1);
		// 	}

		// 	//Motion detected in LLT2 with low light intensity , switch on LLT2 light
		// 	if (check_light_intensity() & detect_motion(LLT2_PIR))
		// 	{
		// 		light_on(LLT2);
		// 	}
		// 	else
		// 	{
		// 		light_off(LLT2);
		// 	}

		// }
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
		//open_door();
		break;
	
	case 'C': //reconfigure 
		//configure();
		break;
	
	default:
		break;
	}
	
	_delay_ms(100);

	light_off(APP_LED);
	
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

    if( (( (strcmp(time, night_time)) > 0) && (strcmp(time, midnight) < 0) ) || (((strcmp(time, "000000") > 0) && strcmp(time, "065959") < 0)) || (strcmp(time, "000000") == 0))
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

	for(int i = 0; i < 10; i++)

	{
		ALARM_PORT |= (1<<ALARM_PIN);
		light_on(APP_LED);
		_delay_ms(100);


		ALARM_PORT &= ~(1<<ALARM_PIN);
		light_off(APP_LED);
		_delay_ms(100);
	}

	

}


void open_door(int door_pin1, int door_pin2)
{
	DOOR_PORT |= (1<<door_pin1); 
	DOOR_PORT &= ~(1<<door_pin2); 
}

void close_door(int door_pin1, int door_pin2)
{
	DOOR_PORT |= (1<<door_pin2); 
	DOOR_PORT &= ~(1<<door_pin1); 
}


void activate_manual_doors()
{
    //opening door
	if (! ((DOOR_PIN) & (1<<LLT1_DOOR_OPEN_BUTTON)) )
	{
		open_door(LLT1_DOOR_PIN1, LLT1_DOOR_PIN2);
	}

	//closing door
	if ( !((DOOR_PIN) & (1<<LLT1_DOOR_CLOSE_BUTTON)) )
	{
		close_door(LLT1_DOOR_PIN1, LLT1_DOOR_PIN2);
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
		light_on(LLT1);
		// _delay_ms(100);
		// light_off(LLT1);
	}
	else if (room_light == '2')
	{
		light_on(LLT2);
		// _delay_ms(100);
		// light_off(LLT2);
	}
	else if (room_light == '3')
	{
		light_on(LLT3);
		// _delay_ms(100);
		// light_off(LLT3);
	}
	else if (room_light == '4')
	{
		light_on(LLT4);
		// _delay_ms(100);
		// light_off(LLT4);
	}
	else if (room_light == '5')
	{
		light_on(SECURITY);
		// _delay_ms(100);
		// light_off(SECURITY);
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
	DDRB = 0X0F;//first 4 bits are for output(motors) , last 4 bits for input(motor buttons)
		//activate button internal pull up resistors
	PORTB |= (1 << LLT1_DOOR_OPEN_BUTTON);
	PORTB |= (1 << LLT1_DOOR_CLOSE_BUTTON);



	//PORTC INIT
	//DDRC = 0x00; //set as input (BUTTONS , INPUT SENSORS CONNECTED)

		//activate button internal pull up resistors
	LIGHTS_BUTTON_PORT |= (1 << LLT1_SWITCH);
	LIGHTS_BUTTON_PORT |= (1 << LLT2_SWITCH);
	LIGHTS_BUTTON_PORT |= (1 << LLT3_SWITCH);
	LIGHTS_BUTTON_PORT |= (1 << LLT4_SWITCH);
	LIGHTS_BUTTON_PORT |= (1 << SECURITY_SWITCH);
	
	
	//PORTD INIT
	DDRD |= (1<<ALARM_PIN); //set alarm pin as output 
	
	//USART INIT
	UBRRL = 0X67; //Set baud rate to 9600 for 16MHz clock 
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0) | (URSEL); // Set frame format: 8 data bits, 1 stop bit
	UCSRB =(1<<RXEN)|(1<<TXEN)| (1 << RXCIE); //Enable serial reception and transmission
	sei();  // Enable global interrupts
   // UCSRA |= (1 << U2X); // Set the U2X bit in the UCSRA register to double transmission speed


   //RTC INITIALIZATION
   rtc_init();
   rtc_setTime(0X21, 0X29, 0X50);

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
	if ( (LIGHTS_SWITCH & (1<<LDR)) == 0)
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


void activate_manual_light_switch()
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

//SEND AN ARRAY OF CHARACTERS SERIALLY
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



