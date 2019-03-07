#include <avr/io.h>
#define F_CPU 8000000UL

#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>


volatile uint8_t dato_recibido=0;
char cadena[10];
uint8_t i=0;
uint8_t a=0;
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

void envia_cadena(char* cadena)
{	 //cadena de caracteres de tipo char
	while(*cadena !=0x00){				//mientras el último valor de la cadena sea diferente
		//a el caracter nulo
		serial_transmit(*cadena);	//transmite los caracteres de cadena
		cadena++;						//incrementa la ubicación de los caracteres en cadena
	}
}

ISR (USART_RX_vect)
{
	while ((UCSR0A & 0B10000000)==0)  ;//ESPERA A Q TERMINE DE RECIBIR
	dato_recibido=UDR0;
	
    if (dato_recibido==32)   //SI EL DATO DE ENTRADA ES ENTER
    {
	    cadena[i++]='\0';  //indica fin de cadena
	    i=0;                 //se resetea el contador
		envia_cadena(cadena);
		if(strcmp(cadena, "On")==0) PORTD|=(1<<7);       
		if(strcmp(cadena, "Off")==0) PORTD&=~(1<<7);	
    }
    else
    {
	    cadena[i++]=dato_recibido;   //SE GUARDA EL DATO EN UNA POSICION DEL VECTOR
    }

}

int main() {
	DDRD = 0B11110010;
	PORTD= 0B00001100;  //ACTIVA PULL/UP PULSADORES
	conf_serial_com();
	sei();

	while(1) {

				
	}

	return 0;
}
