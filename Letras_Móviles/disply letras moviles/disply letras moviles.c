/*
 * disply_letras_moviles.c
 *
 * Created: 09/04/2014 19:40:35
 *  Author: Jonathan
 */ 


#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include "lcd.h"
#include <util/delay.h>
#include <avr/pgmspace.h>    /*guarda tabla en la flash por su tamaño no en la eprom ya q es muy lenta y la ram muy pequeña*/

int dato;
const char letras[] PROGMEM={'M','I','E','D','O','_'};
int aux1;
int puntero;



void leer_analog(void)
{
	ADMUX=0B01000000;
	ADCSRA=0B11000011;
	ADCSRA |= (1<<ADSC);
	// wait until conversion complete ADSC=0 -> Complete
	while (ADCSRA & (1<<ADSC));
	// Get ADC the Result
	dato = ADCW;
}

void declaraion_puertos(void)
{
	DDRB=255;
	DDRA &=~(1<<0);
}

int main(void)
{	
	declaraion_puertos();
	
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	
	

	while(1)
	{
		
	aux1=0;
	
		while (aux1<39)
		{
			 leer_analog();
			 lcd_gotoxy(5,1);
			 lcd_write_value(dato,4);
			
			lcd_gotoxy(15-aux1,0);
			lcd_puts("M");
			lcd_gotoxy(16-aux1,0);
			lcd_puts("I");
			lcd_gotoxy(17-aux1,0);
			lcd_puts(" ");
			lcd_gotoxy(18-aux1,0);
			lcd_puts("P");
			lcd_gotoxy(19-aux1,0);
			lcd_puts("R");
			lcd_gotoxy(20-aux1,0);
			lcd_puts("I");
			lcd_gotoxy(21-aux1,0);
			lcd_puts("M");
			lcd_gotoxy(22-aux1,0);
			lcd_puts("E");
			lcd_gotoxy(23-aux1,0);
			lcd_puts("R");
			lcd_gotoxy(24-aux1,0);
			lcd_puts(" ");
			lcd_gotoxy(25-aux1,0);
			lcd_puts("P");
			lcd_gotoxy(26-aux1,0);
			lcd_puts("R");
			lcd_gotoxy(27-aux1,0);
			lcd_puts("O");
			lcd_gotoxy(28-aux1,0);
			lcd_puts("G");
			lcd_gotoxy(29-aux1,0);
			lcd_puts("R");
			lcd_gotoxy(30-aux1,0);
			lcd_puts("A");
			lcd_gotoxy(31-aux1,0);
			lcd_puts("M");
			lcd_gotoxy(32-aux1,0);
			lcd_puts("A");
			lcd_gotoxy(33-aux1,0);
			lcd_puts(" ");
			lcd_gotoxy(34-aux1,0);
			lcd_puts("E");
			lcd_gotoxy(35-aux1,0);
			lcd_puts("N");
			lcd_gotoxy(36-aux1,0);
			lcd_puts(" ");
			lcd_gotoxy(37-aux1,0);
			lcd_puts("C");
			lcd_gotoxy(38-aux1,0);
			lcd_puts(" ");
			_delay_ms(150);	
			aux1++;
			
		}
		
		lcd_gotoxy(0,0);
		lcd_puts("                        ");
				
				
		
		/*leer_analog();
		lcd_gotoxy(5,1);
		lcd_write_value(dato,4);*/
	}
}