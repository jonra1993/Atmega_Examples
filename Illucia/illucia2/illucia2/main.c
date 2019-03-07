/*
 * illucia.c
 *
 * Created: 4/23/2017 2:23:35 PM
 * Author : jon_r
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL

#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

uint8_t needsHandshake = 1;
volatile uint8_t dato_recibido=0;
char cadena[2];
char dataToSend[4];

uint8_t i=0;
uint8_t a=0;

//////////////////////////////////////////
uint8_t const CONTINUOUS_TYPE = 3;
uint8_t const DIGITAL_TYPE = 1;
uint8_t const JACK_TYPE = 2;


	
int const NUMBER_OF_CONTINUOUS_ELEMENTS = 6; // how many continuous controls?
int const NUMBER_OF_JACKS = 16; //how many jacks are on this device?

int const ANALOG_CHANGE_THRESHOLD = 6; //used for software smoothing of analogRead jitter
//For incoming data to microcontroller:
uint8_t const RESET_TYPE = 0; //used to tell the device to go back to waiting for a handshake
uint8_t const LED_TYPE = 1; //used to specify an LED's brightness
int const NUMBER_OF_LEDS = 4; //how many LEDs are on this device?



#define baddr DDRB
#define bbddr DDRC
#define bapor PORTB
#define bbpor PORTC
#define bapin PINB
#define bbpin PINC

int const NUMBER_OF_LEDS_ELEMENTS = 4; // how many buttons & switches
#define ledsport PORTD

	
int const NUMBER_OF_SWITCHES_ELEMENTS = 2; // how many buttons & switches
#define switchpin PIND



int const NUMBER_OF_BUTTON_ELEMENTS = 2; // how many buttons & switches?
#define buttonpin PINA

int llegada=5;

////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
struct mapasleds {
	uint8_t   _id;
	uint8_t value;
};

struct mapasleds leds[4]={{2,0},{3,0},{4,0},{5,0}};

uint8_t contador=0;

void analogWrite(int ledpin, uint8_t value, uint8_t conta)
{
	if (conta==0) ledsport |= (1<<ledpin);
	if (conta==value)ledsport&=~ (1<<ledpin);

}

void timer1(void)
{
	TCCR1A=0B00000000;  //modo ctc
	TCCR1B=0B00001001; //prescalador 1 
	OCR1A=7;  //a f de 500khz cada 2us
	TIMSK1=0B00000010;
}

ISR (TIMER1_COMPA_vect) //cada 2us
{
	contador++;
	for (int l=0; l<NUMBER_OF_LEDS_ELEMENTS; l++)
	{
		analogWrite(leds[l]._id,leds[l].value,contador);
	}
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

void envia_cadena(char* cadena)
{	 //cadena de caracteres de tipo char
	while(*cadena !=0x00){				//mientras el último valor de la cadena sea diferente
		//a el caracter nulo
		serial_transmit(*cadena);	//transmite los caracteres de cadena
		cadena++;						//incrementa la ubicación de los caracteres en cadena
	}
}

ISR (USART0_RX_vect)
{
	while ((UCSR0A & 0B10000000)==0)  ;//ESPERA A Q TERMINE DE RECIBIR
	dato_recibido=UDR0;
	
	if ((dato_recibido=='l')&&(llegada>1))   //SI EL DATO DE ENTRADA ES ENTER
	{
		llegada=0;
	}
	else if ((dato_recibido=='r')&&(llegada>1))
	{
		needsHandshake=1;
	}
	else
	{
		cadena[llegada++]=dato_recibido;   //SE GUARDA EL DATO EN UNA POSICION DEL VECTOR
	}
	
	if (llegada>1)
	{
		if (cadena[1]=='a') ledsport |= (1<<(cadena[0]-48));
		else ledsport&=~ (1<<(cadena[0]-48));
	}

}

//////////////////////////////////////////////////////////////

void pines()
{
	DDRD = 0B00111110;
	PORTD= 0B11000000;  //ACTIVA PULL/UP PULSADORES
	
	baddr=0;
	bapor=255;
	bbddr=0;
	bbpor=255;
	
	PORTA=0;
	PORTA=0b11000000;	
}


#define a1 0b01000000
#define a2 0b01000001
#define a3 0b01000010
#define a4 0b01000011
#define a5 0b01000100
#define a6 0b01000101


////////////////////////////////////////////////////////////
int lectura_analoga(uint8_t canal)
{
	ADMUX=canal;
	DIDR0=0B111111;
	ADCSRA=0B11000110;
	while (ADCSRA & (1<<ADSC)); //espera que complete la conversion
	return ADCW;
}


/////////////////////////////////////////////////////////
struct mapaswitches {
	uint8_t   _id;
	uint8_t beforeReading;

    //pack a byte array with data about the state change
};

struct mapaswitches switches[2]={{6,(1<<6)},{7,(1<<7)}};
	
void lectura_switches()
{
	for (int i= 0; i<2; i++)
	{
		uint8_t currentReading=(switchpin & (1<<switches[i]._id));
		if (switches[i].beforeReading != currentReading) { //there is a state change, or this is the very first time checking state	
			char valueToSend;
			if (currentReading == (1<<switches[i]._id)) valueToSend = 1;   //realmente es 0 y 1 abajo
			else  valueToSend = 2;
		
			//pack a byte array with data about the state change
			 dataToSend[0]=DIGITAL_TYPE;
			 dataToSend[1]=i+3;
			 dataToSend[2]=valueToSend;
			 dataToSend[3]=255;
			envia_cadena(dataToSend); //send it over serial.
			switches[i].beforeReading=currentReading;
		}
	}
}

/////////////////////////////////////////////////////////
struct mapabotones {
	uint8_t   _id;
	uint8_t beforeReading;

	//pack a byte array with data about the state change
};

struct mapabotones botones[2]={{6,(1<<6)},{7,(1<<7)}};

void lectura_botones()
{
	for (int i= 0; i<2; i++)
	{
		uint8_t currentReading=(buttonpin & (1<<botones[i]._id));
		if (botones[i].beforeReading != currentReading) { //there is a state change, or this is the very first time checking state
			char valueToSend;
			if (currentReading == (1<<botones[i]._id)) valueToSend = 1;   //realmente es 0 y 1 abajo
			else  valueToSend = 2;
			
			//pack a byte array with data about the state change
			dataToSend[0]=DIGITAL_TYPE;
			dataToSend[1]=i+1;
			dataToSend[2]=valueToSend;
			dataToSend[3]=255;
			envia_cadena(dataToSend); //send it over serial.
			botones[i].beforeReading=currentReading;
		}
	}
}

struct mapapotenciometros {
	uint8_t   _id;
	int  beforeReading;
};
struct mapapotenciometros potenciometros[6]={{a1,0},{a2,0},{a3,0},{a4,0},{a5,0},{a6,0}};

void lectura_potenciometros()
{
	for (int k=0; k<6;k++)
	{
		int lectura_adc=lectura_analoga(potenciometros[k]._id);
		int difference = potenciometros[k].beforeReading - lectura_adc;
        if (abs(difference) > ANALOG_CHANGE_THRESHOLD) 
	   { //simple software way of factoring out fluctations in reading due to noise/power
	        
	        //check for edge cases, so pot doesn't miss full CW or CCW positions given software filtering
	        if (lectura_adc <= ANALOG_CHANGE_THRESHOLD)
			{
		        lectura_adc = 0;
		        } else if (lectura_adc >= 1023 - ANALOG_CHANGE_THRESHOLD) {
		        lectura_adc = 1023;
	        }
	        
			uint8_t lsbyte = (uint8_t) lectura_adc;
			uint8_t msbyte = (uint8_t) (lectura_adc >> 8);
			
			dataToSend[0]=CONTINUOUS_TYPE;
			dataToSend[1]=k+1;
			dataToSend[2]=msbyte;  
			dataToSend[3]=lsbyte;     
			envia_cadena(dataToSend); //send it over serial	        
	        
	        //update internal state
	        potenciometros[k].beforeReading = lectura_adc;
        }		
	}
	
}

struct mapaconexiones {
	uint8_t   _id;
	uint8_t beforeReading[16];

	//pack a byte array with data about the state change
};
struct mapaconexiones conexion[16];
	
void conectar()
{	
	for (int i=0; i<8; i++)
	{
		conexion[i]._id=i;
		for (int h=0; h<16; h++)
		{
			if (h<8) conexion[i].beforeReading[h]=(1<<h); 
			else conexion[i].beforeReading[h]=(1<<(h-8));			
		}
		
	}	
	for (int i=8; i<16; i++)
	{
		conexion[i]._id=i;
		for (int h=0; h<16; h++)
		{
			if (h<8) conexion[i].beforeReading[h]=(1<<h);
			else conexion[i].beforeReading[h]=(1<<(h-8));			
		}		
	}			
}	
void determinarconeciones()
{
	
	int i;
	for (i=0; i<NUMBER_OF_JACKS; i++)
	{
		int x;
		//cambia a salida y le pone encer
		if (i<8)
		{
			x=i;
			baddr |= (1<<x);
			bapor&=~ (1<<x);;
		}
		else
		{ 
			x=i-8;
			bbddr |= (1<<x);
			bbpor&=~ (1<<x);
		}
		//_delay_ms(50);
		
		//conexion[i]._id es el que esta en cero
		for (int j = 0; j < NUMBER_OF_JACKS; j++)
		{
			if (j != i) 
			{ 
				int currentReading;
				if (j<8)  currentReading = (bapin & (1<<conexion[j]._id));
				else      currentReading = (bbpin & (1<<(conexion[j]._id-8)));
							
				//this just means "don't check this pin for connections with itself"				
				if (conexion[i].beforeReading[j] != currentReading) 
				{ //"is this reading different from what is already recorded as the state?" or, is it the first re
					char valueToSend;
					if (j<8)
					{
							if (currentReading == (1<<conexion[j]._id)) valueToSend = 1;   //realmente es 0 y 1 abajo
							else  valueToSend = 2;	
					} 
					else
					{
							if (currentReading == (1<<(conexion[j]._id-8))) valueToSend = 1;   //realmente es 0 y 1 abajo
							else  valueToSend = 2;
					}
					
					conexion[i].beforeReading[j]=currentReading;
									
				//	if (conexion[i].beforeReading[j]!=conexion[j].beforeReading[i])
				//	{
						//pack a byte array with data about the state change
						dataToSend[0]=JACK_TYPE;
						dataToSend[1]=conexion[i]._id;
						dataToSend[2]=conexion[j]._id;
						dataToSend[3]=valueToSend;
						envia_cadena(dataToSend); //send it over serial
											

				//	}													
				}
			}
			
		}
		
		
		//regresa EL PIN ha ser entrada y se activa el pull up
		if (i<8)
		{
			baddr&=~ (1<<x);
			bapor |= (1<<x);
		} 
		else
		{
			bbddr&=~ (1<<x);
			bbpor |= (1<<x);		
		}
		//_delay_ms(50);
	}
	
}

int main(void)
{
    /* Replace with your application code */
	
	pines();
	conf_serial_com();
	//timer1();
	sei();
    

   
	conectar();	

	while (1) 
    {
    if (needsHandshake) {
		//mensaje inicial
		dataToSend[0]='d';
	    dataToSend[1]='e';
	    dataToSend[2]='t';
	    dataToSend[3]='r';
		envia_cadena(dataToSend); //send it over serial		
	    _delay_ms(100);
	    needsHandshake = 0;
	    
	    for (int j = 0; j < 2; j++) {
		    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
				ledsport |= (1<<leds[i]._id);
			    _delay_ms(45); //init sequence
				ledsport&=~ (1<<leds[i]._id);
		    }
	    }
	    
	   }else {
	    
	    //update elements
		lectura_switches();
		lectura_botones();
		determinarconeciones();
		lectura_potenciometros();
			
	    	    
	    _delay_ms(1);
	    
	    }
	    
    }		

    
}

