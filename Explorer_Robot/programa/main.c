/*
 * programa.c
 *
 * Created: 7/30/2016 11:25:06 AM
 * Author : jon_r
 */ 


#include <avr/io.h>
#define F_CPU 8000000UL

#include <util/delay.h>
#include <avr/interrupt.h>

//Definicion de pwms
#define PWMA OCR0A
#define PWMB OCR0B

//Definicion de mot
#define PORT_MOTOR1 PORTB
#define PORT_MOTOR2 PORTA
#define PORT_MOTOR3 PORTA
#define PORT_MOTOR4 PORTA
#define PORT_MOTOR5 PORTA

#define M1_1	  0
#define M1_2      1
#define M2_1      0
#define M2_2      1
#define M3_1      2
#define M3_2      3
#define M4_1      4
#define M4_2      5
#define M5_1      6
#define M5_2      7


volatile uint8_t dato_recibido=0;

void declaraciones(void);
void conf_pwm(void);
void conf_serial_com(void);
void serial_transmit(unsigned char dato);
void timer1(void);
void movimiento(uint8_t val); 

int main(void)
{
	declaraciones();
	conf_pwm();
	conf_serial_com();
	timer1();
	sei();
    /* Replace with your application code */
    while (1) 
    {
    }
}


//////////////////////////////////////////////////////////////
void declaraciones(void)
{
	DDRA = 0B11111111;
	DDRB = 0B11111111;
	DDRD = 0B11110010;
	PORTD= 0B00001100;  //ACTIVA PULL/UP PULSADORES
	
}
//////////////////////////////////////////////////////////////
void conf_pwm(void)
{
	TCCR0A=0B10100011;  //FAST PWM
	TCCR0B=0B00000001;
	PWMA=255;
	PWMB=255;
}
/////////////////////////////////////////////////////////////
void timer1(void)
{
	TCCR1A=0B00000000;
	TCCR1B=0B00001010;
	OCR1A=65530;  //a f de 20hz cada 50ms 500
	OCR1B=25000;  //cada 5ms 78
	TIMSK1=0B00000110;
}

ISR (TIMER1_COMPA_vect) //cada 50ms
{
		PORTB&=~((1<<PINB0)|(1<<PINB1));
		PORTA=0;
		
}


ISR (TIMER1_COMPB_vect)  //cada 5ms
{
	
}

//////////////////////////////////////////////////////////////
void conf_serial_com(void)
{
	UCSR0A=0;
	UCSR0B=0b10011000; // transmite y recibe activa uint de recibido
	UCSR0C=0b00000110;
	UBRR0=51;           //baud rate inicializado a 9600
}

void serial_transmit(unsigned char dato)
{
	while ((UCSR0A & 0b100000)==0);  //mientras bit UDRE tenga dato
	UDR0=dato;    //SE ENVIA EL DATO
}

ISR (USART0_RX_vect)
{
	while ((UCSR0A & 0B10000000)==0)  ;//ESPERA A Q TERMINE DE RECIBIR
	dato_recibido=UDR0;

	movimiento(dato_recibido);

}


void movimiento(uint8_t val)  //mov_xxxx
{
	//pwm de 0 a 255
	switch (val)
	{
		case 'a': //MOTOR1_D
		PORT_MOTOR1|=(1<<M1_1);
		PORT_MOTOR1&=~(1<<M1_2);
		break;

		case 'b': //MOTOR1_I
		PORT_MOTOR1|=(1<<M1_2);
		PORT_MOTOR1&=~(1<<M1_1);
		break;
		
		case 'c': //MOTOR2_D
		PORT_MOTOR2|=(1<<M2_1);
		PORT_MOTOR2&=~(1<<M2_2);
		break;
		
		case 'd': //MOTOR2_I
		PORT_MOTOR2|=(1<<M2_2);
		PORT_MOTOR2&=~(1<<M2_1);
		break;
		
		case 'e': //MOTOR2_D
		PORT_MOTOR3|=(1<<M3_1);
		PORT_MOTOR3&=~(1<<M3_2);
		break;
		
		case 'f': //MOTOR2_I
		PORT_MOTOR3|=(1<<M3_2);
		PORT_MOTOR3&=~(1<<M3_1);
		break;

		case 'g': //MOTOR2_D
		PORT_MOTOR4|=(1<<M4_1);
		PORT_MOTOR4&=~(1<<M4_2);
		break;
		
		case 'h': //MOTOR2_I
		PORT_MOTOR4|=(1<<M4_2);
		PORT_MOTOR4&=~(1<<M4_1);
		break;
		
		case 'i': //MOTOR5_D
		PORT_MOTOR5|=(1<<M5_1);
		PORT_MOTOR5&=~(1<<M5_2);
		break;
		
		case 'j': //MOTOR5_I
		PORT_MOTOR5|=(1<<M5_2);
		PORT_MOTOR5&=~(1<<M5_1);
		break;
	}

}

