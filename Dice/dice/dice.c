/*
 * dice.c
 *
 * Created: 27/12/2014 19:03:20
 *  Author: JONATHAN
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>




int contador;
int tempo;
int vector[]={0b11000000,0b10100100,0b11100100,0b10101101,0b11101101,0b10111111,0b11111111};

void puertos(void);
void interrupciones(void);

int main(void)
{
	puertos();
	interrupciones();
	tempo=random()%7;
	contador=0;
	
    while(1)
    {
		PORTB=vector[tempo];
		
		if (contador==1)    //hace q parezka q gira antes de detenerse en un numero
		{
				contador=0;
				PORTB=vector[random()%7];
				_delay_ms(100);
				PORTB=vector[random()%7];
				_delay_ms(100);
				PORTB=vector[random()%7];
				_delay_ms(100);
				PORTB=vector[random()%7];
				_delay_ms(100);
	
				tempo=random()%7;
				PORTB=vector[tempo];
		}
		   
    }
	
	return 0;
}


void puertos (void)
{
	DDRA=0;
	DDRD=0;
	DDRB=0b01111111;
	PORTA=0b111;
	PORTD=0b1111111;
	PORTB=0b10000000;
	

}

void interrupciones (void)
{
	MCUCR=0b00001010;
	GIMSK=0b11000000;
	sei();
}

ISR(INT0_vect)
{
	_delay_ms(20);  //ELIMINA REBOTES
	contador=1;
	
}

