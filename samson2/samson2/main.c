
#define  F_CPU 16000000UL
#include <avr/io.h>
#include<stdio.h>
#include<string.h>
const int SIZE=6;
#include<util/delay.h>


//initialize serial reception
void usart_init(void){
	DDRB=0xff;
	UCSRB =(1<<RXEN)|(1<<TXEN);
	UCSRC=(1<<UCSZ1)|(1<<UCSZ0)|(1<<URSEL);
	UBRRL=0X67;
}


//void usart_init_send(void){
//DDRB=0xff;
//UCSRB =(1<<TXEN);
//UCSRC=(1<<UCSZ1)|(1<<UCSZ0)|(1<<URSEL);
//UBRRL=0X67;
//}

void usart_send(unsigned char j){
	//DDRB=0xff
	while (!(UCSRA &(1<<UDRE)));
	_delay_ms(100);
	UDR=j;
	
}
//LIGHTS CONTROL
void switch_onlights(void){
	while (!(UCSRA &(1<<RXC)));
	_delay_ms(100);

	switch(UDR){
		case 'A':
		PORTB |=(1<<PB7);
		break;
		case 'B':
		PORTB |=(1<<PB6);
		break;
		case 'C':
		PORTB |=(1<<PB5);
		break;
		case 'D':
		PORTB |=(1<<PB4);
		break;
		case 'E':
		PORTB &=~(1<<PB7);
		break;
		case 'F':
		PORTB &=~(1<<PB6);
		break;
		case 'G':
		PORTB &=~(1<<PB5);
		break;
		case 'H':
		PORTB &=~(1<<PB4);
		break;
		default:
		PORTB ^=PORTB;
		break;
	}
	
}



//Main RECEPTION
//void usartRecieve(void){}



int main(void)
{
	
	DDRA =0x00;
	PORTA |=(1<<0);
	DDRC=0xff;
	if ((PINA &(1<<0))==0)
	{
		PORTB |=1<<PB4;
	}
	
	unsigned char x[]={"Smartsam125$%&*^%$#@"};
	unsigned char i=0;
	
	usart_init();
	
	
	while (1)
	{
		
		//usartRecieve();
		switch_onlights();
		
		while (!(UCSRA &(1<<RXC)));
		_delay_ms(100);
		
		
		switch(UDR){
			case 'X':
			
			switch_onlights();
			
			
			
			break;
			case 'Y':
			{
				for (int k=0;k<strlen(x);k++)
				{
					usart_send(x[k]);
					
				}
				//usart_send(x[i++]);
				//if(x[i]==0);
				//i=0;
			}
			
			
			break;
			case 'F':
			PORTB |=1<<PB3;
			break;
			
			default:
			//usart_init_send('t');
			break;
			
		}
		
		
		
	}
}
