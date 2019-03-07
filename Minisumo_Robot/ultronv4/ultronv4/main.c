/*
 * ULTROBNV1.c
 *
 * Created: 5/14/2016 3:57:09 PM
 * Author : Jona
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <string.h>
#include <avr/eeprom.h>

#include <util/delay.h>
#include <avr/interrupt.h>
char cadena[40];
char enviar[40];

volatile uint8_t control=0;
volatile uint8_t aux=0;
volatile uint16_t Vlip=0;
volatile uint16_t col=0;
volatile uint8_t negro=1;
volatile uint8_t dato_recibido=0;
volatile uint8_t i=0;
volatile uint8_t j=0;
volatile uint16_t lecoff[5]={0,0,0,0,0};
volatile uint16_t lecon=0;
volatile uint16_t medidas[7]={0,0,0,0,0,0,0};
volatile uint8_t dentrorango[5]={0,0,0,0,0};
volatile uint8_t u=0;
volatile uint8_t comenzar;
volatile uint8_t lala;

volatile uint8_t DI_anterior;
volatile int temp;

//definiciones del pid
volatile float ed=0;
volatile float eizq=0;
volatile float set_point=0;
volatile float I=0;
volatile float error=0;
volatile float error_anterior=0;
volatile float D=0;
volatile float target_power=0;
volatile float kp;
volatile float ki;
volatile float kd;
volatile float ley;
volatile int PWMDER;
volatile int PWMIZQ;
volatile int jonathan=0;

volatile uint8_t DD_anterior;
volatile uint8_t DI_anterior;

volatile int auxx1=0;
volatile int auxx2=0;
/*
LED1	PA7
LED2	PB1
LED3	PB2
LED4	PC5	
LED5	PC6	


SERIAL 0

PULSADORES
MARCHA	PD2
ESTRAT	PD3

PINES QUE MANEJAN LOS DOS MOTORES
PWMA	PB3   TIMER0A
PWMB	PB4   TIMER0B
BIN1	PD4
BIN2	PD5
AIN1	PD6
AIN2	PD7

VISTOS DESDE EL FENRE SENSORES 
ADC0	PA0	RECEPTOR DERECHO
ADC1	PA1	RECEPTOR FRONTAL_DERECHO
ADC2	PA2	RECEPTOR FRONTAL
ADC3	PA3	RECEPTOR FRONTAL_IZQUIERDO	
ADC4	PA4	RECEPTOR IZQUIERDO
COLOR	PA5	
ADCLIP	PA6


TODOS LOS EMISORES SE ENCIENDEN CON CERO
OUT1	PC0	EMISOR DERECHO
OUT2	PC1	EMISOR FRONTAL_DERECHO
OUT3	PC2	EMISOR CENTRAL
OUT4	PC3	EMISOR FRONTAL_IZQUIERDO
OUT5	PC4	EMISOR IZQUIERDO

*/

#define receptor_derecho 0
#define receptor_frontal_derecho 1
#define receptor_fontal 2
#define receptor_frontal_iz 3
#define receptor_izquierdo 4
#define color    5
#define Vlipo    6

//Definicion de leds
#define PORT_LED1 PORTA
#define LED1      7
#define PORT_LED2 PORTB
#define LED2      1
#define PORT_LED3 PORTB
#define LED3      2
#define PORT_LED4 PORTC
#define LED4      5
#define PORT_LED5 PORTC
#define LED5      6

//Definicion de pwms
#define PWMA OCR0A
#define PWMB OCR0B

//Definicion de emisores infrarojos
#define PORT_INFR PORTC
#define OUT_D      0
#define OUT_FD     1
#define OUT_F      2
#define OUT_FI     3
#define OUT_I      4

//Definiciones de los motores
#define PORT_MOTORES PORTD
#define MOTOR1A 6
#define MOTOR1B 7
#define MOTOR2A 4
#define MOTOR2B 5

 //
#define mov_avanzar	   2
#define mov_retroceder 1
#define mov_derecha    3
#define mov_izquierda  4
#define mov_parar      5


void declaraciones(void);
void conf_pwm(void);
void conf_adc(void);
int lectura_sensor(uint8_t canal);
void conf_serial_com(void);
void serial_transmit(unsigned char dato);
void timer1(void);
void int_pulsadores(void);
void movimiento(uint8_t mov,uint8_t velocidad_der,uint8_t velocidad_iz);
void leds(void);
void esperar5s (void);
void elegiraccion(void);
void SensarBatery (uint8_t a);
void primermov(uint8_t lala);
void pid(void);
void adc_init(void);

void envia_cadena(char* cadena);
void actualizar_variables(void);
void tostring(char [], int);

#define direccion_ref_d 0
#define direccion_ref_fd 2
#define direccion_ref_f 4
#define direccion_ref_fi 6
#define direccion_ref_i 8
#define direccion_ref_col 10

#define direccion_kp 20
#define direccion_ki 30
#define direccion_kd 40
#define direccion_set_point 50
#define direccion_target_power 60
// volatile uint16_t ref_d=5;
// volatile uint16_t ref_fd=2;
// volatile uint16_t ref_f=3;
// volatile uint16_t ref_fi=4;
// volatile uint16_t ref_i=4;
// volatile uint16_t ref_col=180;
volatile uint16_t ref_d;
volatile uint16_t ref_fd;
volatile uint16_t ref_f;
volatile uint16_t ref_fi;
volatile uint16_t ref_i;
volatile uint16_t ref_col;
volatile uint16_t ref_lipo=180;

int main(void)
{
	declaraciones();
	conf_pwm();
	conf_serial_com();
	timer1();
	int_pulsadores();
	adc_init();
	sei();
		
	lala=0;
	comenzar=0;

	do
	{
		movimiento(mov_parar,0,0);
		elegiraccion(); //FRENTE, IZQUIERDA, DERECHA, ATRAS
		
	} while (comenzar==0);

	esperar5s();	
	
	primermov(lala);
		
    while (1) 
    {
// 		if (medidas[0]>6) PORT_LED2|=(1<<LED2);
// 		else PORT_LED2&=~(1<<LED2);
// 
// 		if (medidas[1]>15 ) PORT_LED3|=(1<<LED3);
// 		else PORT_LED3&=~(1<<LED3);		
		
		if (medidas[0]>ref_d && auxx1==0)
		{ 
			auxx1=1;
			movimiento(mov_izquierda,254,254);
		}
		if (medidas[4]>ref_i && auxx2==0)
		{
			auxx2=1;
			 movimiento(mov_derecha,254,254);
		}
			
		if (medidas[3]>ref_fi)
		{
			 PORT_LED5|=(1<<LED5); //frontal derecho
			 auxx1=0;
		}
		else PORT_LED5&=~(1<<LED5);

		if (medidas[2]>ref_fd )
		{
			 PORT_LED4|=(1<<LED4); //frontal izquierdo
			 auxx2=0;
		}
		else PORT_LED4&=~(1<<LED4);
		
		if (auxx1==0 && auxx2==0)
		{
			pid();
		}
					
    }
}

void tostring(char str[], int num)
{
	int t, rem, len = 0, n;
	n = num;
	while (n != 0)
	{
		len++;
		n /= 10;
	}
	if (num==0)
	{
		str[3] = '0';
		str[2] = '0';
		str[1] = '0';
		str[0] = '0';
	}
	
	for (t = 0; t < len; t++)
	{
		rem = num % 10;
		num = num / 10;
		switch (len)
		{
			case 4:
			str[len - (t + 1)] = rem + '0';
			break;
			case 3:
			str[len+1 - (t+1)] = rem + '0';
			str[0] = '0';
			break;
			case 2:
			str[len+2 - (t+1)] = rem + '0';
			str[1] = '0';
			str[0] = '0';
			break;
			case 1:
			str[len+3 - (t+1)] = rem + '0';
			str[2] = '0';
			str[1] = '0';
			str[0] = '0';
			break;
		}
		//str[len] = '\0';
	}
}

void actualizar_variables(void)
{
	ref_d=eeprom_read_word((uint16_t*)direccion_ref_d);
	ref_fd=eeprom_read_word((uint16_t*)direccion_ref_fd);
	ref_f=eeprom_read_word((uint16_t*)direccion_ref_f);
	ref_fi=eeprom_read_word((uint16_t*)direccion_ref_fi);
	ref_i=eeprom_read_word((uint16_t*)direccion_ref_i);
	ref_col=eeprom_read_word((uint16_t*)direccion_ref_col);
	
	kp=eeprom_read_float((float*)direccion_kp);
	ki=eeprom_read_float((float*)direccion_ki);
	kd=eeprom_read_float((float*)direccion_kd);
	set_point=eeprom_read_float((float*)direccion_set_point);
	target_power=eeprom_read_float((float*)direccion_target_power);
}
//////////////////////////////////////////////////////////////
void declaraciones(void)
{
	DDRA = 0B10000000;
	DDRC = 0B11111111;
	DDRD = 0B11110010;
	PORTD= 0B00001100;  //ACTIVA PULL/UP PULSADORES
	DDRB = 0B00011110;	
}
//////////////////////////////////////////////////////////
// INTERRUPCION PULSADORES
void int_pulsadores(void)
{
	EICRA=0B00001010;
	EIMSK=0B00000011;
}

ISR(INT1_vect)
{
	_delay_ms(20);
	aux=PIND&(0B1000);
	if (aux==0)
	{
		lala++;
		if (lala>4) lala=0;
	}
}

ISR(INT0_vect)
{
	_delay_ms(20);
	aux=PIND&(0B100);
	if (aux==0)
	{
		comenzar=1;
		if (u==0)
		{
		PORT_LED3|=(1<<LED3);
		u=1;
		}
		else
		{
		PORT_LED3&=~(1<<LED3);
		u=0;
		}	
	}
		
}
//////////////////////////////////////////////////////////////
void conf_pwm(void)
{
	TCCR0A=0B10100011;  //FAST PWM
	TCCR0B=0B00000001;
	PWMA=240;
	PWMB=200;
}
/////////////////////////////////////////////////////////////
void timer1(void)
{
	TCCR1A=0B00000000;  //modo contador
	TCCR1B=0B00001010; //prescalador 8
	OCR1A=1000;  //a f de 500hz cada 2ms a 1000
	OCR1B=500;  //cada 1ms 500
	TIMSK1=0B00000110;
}

ISR (TIMER1_COMPA_vect) //cada 2ms
{
	Vlip=lectura_sensor(Vlipo);
	col=lectura_sensor(color);
	if (col<ref_col) negro=0; //esta en blanco
	else negro=1;         //esta en negro
	

		temp++;
		
		
		//1 significa dentro del rango, 0 fuera
		if (medidas[0]>ref_d) 
		{ 
			dentrorango[0]=-2; 
			DD_anterior=0; 
			DI_anterior=1; 
		}//iz
		else 
		{
			dentrorango[0]=0;
		}

		if (medidas[1]>ref_fd) 
		{
			dentrorango[1]=2; 
			DD_anterior=1; 
			DI_anterior=0;  
		}
		else 
		{
			dentrorango[1]=0;
		}
		
		if (medidas[2]>ref_f) //drontL DERECHO 
		{
			dentrorango[2]=1; 
			DD_anterior=1; 
			DI_anterior=0;
		}  //f_d
		else 
		{
			dentrorango[2]=0;
		}
		
		if (medidas[3]>ref_fi)  
		{
			dentrorango[3]=0; 
			DD_anterior=1; 
			DI_anterior=0;
		}  //f
		else 
		{
			dentrorango[3]=0;
		}
		
		if (medidas[4]>ref_i) //FRONT IZQUIERDO
		{
			 dentrorango[4]=-1; 
			 DD_anterior=0; 
			 DI_anterior=1;
		} //f_iz
		else 
		{
			dentrorango[4]=0;
		}

}

ISR (TIMER1_COMPB_vect)  //cada 1ms
{
	
	if (i==0)
	{
		PORT_INFR|=0b11111; //enciende todos los infraro=os
		//rec_der,rec_f_de,rec_f,rec_f_iz,rec_iz
		for (int k=0; k<5; k++ )
		{
			lecon=lectura_sensor(k);
			medidas[k]=lecoff[k]-lecon;
			//medidas[k]=lecon;
		}
		medidas[5]=col;
		medidas[6]=Vlip;
		i=1;
	}
	else
	{
		PORT_INFR&=~((1<<OUT_D)|(1<<OUT_FD)|(1<<OUT_F)|(1<<OUT_FI)|(1<<OUT_I)); //apaga todo
		for (int k=0; k<5; k++ )
		{
			lecon=lectura_sensor(k);
			lecoff[k]=lecon;
		}
		i=0;
	}
}

void adc_init(void)
{
	ADMUX = (1<<REFS0); // AREF = AVcc
	// ADC Enable and prescaler of 64
	// 8000000/64 = 125000
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
	DIDR0=0B11111;

}
////////////////////////////////////////////////////////////
int lectura_sensor(uint8_t canal)
{
	canal = canal & 0b00000111;
	ADMUX =(1<<REFS0)|canal;

	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC)); //espera que complete la conversion
	return ADC;
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
	int y=0;
	int casi=0;
	while(y==0){				//mientras el último valor de la cadena sea diferente
		//a el caracter nulo
		if (casi==1)
		{
			if (*cadena=='i') y=1;
		}
		else
		{
			if (*cadena=='f') casi=1;
			else casi=0;
		}
		
		serial_transmit(*cadena);	//transmite los caracteres de cadena
		cadena++;						//incrementa la ubicación de los caracteres en cadena
	}
}
ISR (USART0_RX_vect)
{
	while ((UCSR0A & 0B10000000)==0)  ;//ESPERA A Q TERMINE DE RECIBIR
	dato_recibido=UDR0;
    if (dato_recibido=='i' && control==1)   //SI EL DATO DE ENTRADA ES ENTER
    {
	    control=0;
	    j=0;                 //se resetea el contador
	    
	    if (cadena[0]=='1')
	    {
		    if (cadena[1]=='v') //envia los datos de sensores 1vfi
		    {
			    char str[4];
			    for (int k=0; k<7; k++ )
			    {
				    tostring(str, medidas[k]);
				    for (int x=0; x<4; x++)
				    {
					    enviar[4*k+x]=str[x];
				    }
				    
			    }
			    enviar[34]='v';  //indica fin de cadena
			    enviar[35]='a';  //indica fin de cadena
			    enviar[36]='r';  //indica fin de cadena
			    enviar[37]='f';  //indica fin de cadena
			    enviar[38]='i';  //indica fin de cadena
			    envia_cadena(enviar);
		    }
		    else if (cadena[1]=='h') //envia los valores de constante controlador 1hfi
		    {
			    
			    uint16_t ox;
			    
			    char str[4];
			    
			    ox=kp*100;
			    tostring(str, (int)ox);
			    int k=0;
			    for (int x=0; x<4; x++) enviar[4*k+x]=str[x];
			    
			    ox=ki*100;
			    tostring(str, (int)ox);
			    k=1;
			    for (int x=0; x<4; x++) enviar[4*k+x]=str[x];
			    
			    ox=kd*100;
			    tostring(str, (int)ox);
			    k=2;
			    for (int x=0; x<4; x++) enviar[4*k+x]=str[x];
			    
			    ox=set_point;
			    tostring(str, (int)ox);
			    k=3;
			    for (int x=0; x<4; x++) enviar[4*k+x]=str[x];
			    
			    ox=target_power;
			    tostring(str, (int)ox);
			    k=4;
			    for (int x=0; x<4; x++) enviar[4*k+x]=str[x];
			    

			    enviar[34]='c';  //indica fin de cadena
			    enviar[35]='o';  //indica fin de cadena
			    enviar[36]='n';  //indica fin de cadena
			    enviar[37]='f';  //indica fin de cadena
			    enviar[38]='i';  //indica fin de cadena
			    envia_cadena(enviar);
		    }

		    else if (cadena[1]=='j') //envia los datos de referencias sensores 1jfi
		    {

			    char str[4];
			    tostring(str, ref_d);
			    int k=0;
			    for (int x=0; x<4; x++) enviar[4*k+x]=str[x];

			    tostring(str, ref_fd);
			    k=1;
			    for (int x=0; x<4; x++) enviar[4*k+x]=str[x];

			    tostring(str, ref_f);
			    k=2;
			    for (int x=0; x<4; x++) enviar[4*k+x]=str[x];

			    tostring(str, ref_fi);
			    k=3;
			    for (int x=0; x<4; x++) enviar[4*k+x]=str[x];
			    
			    tostring(str, ref_i);
			    k=4;
			    for (int x=0; x<4; x++) enviar[4*k+x]=str[x];
			    
			    tostring(str, ref_col);
			    k=5;
			    for (int x=0; x<4; x++) enviar[4*k+x]=str[x];
			    
			    
			    enviar[34]='r';  //indica fin de cadena
			    enviar[35]='e';  //indica fin de cadena
			    enviar[36]='f';  //indica fin de cadena
			    enviar[37]='f';  //indica fin de cadena
			    enviar[38]='i';  //indica fin de cadena
			    envia_cadena(enviar);

		    }

		    else if (cadena[1]=='r') //recibe los datos de referencias sensores 1rxxxxxxxxxxxxfi
		    {
			    if (cadena[2]>57 || cadena[2]<48) cadena[2]='0';
			    if (cadena[3]>57 || cadena[3]<48) cadena[3]='0';
			    if (cadena[4]>57 || cadena[4]<48) cadena[4]='0';
			    ref_d=(cadena[2]-48)*1000+(cadena[3]-48)*100+(cadena[4]-48)*10+(cadena[5]-48);
			    eeprom_write_word((uint16_t*)direccion_ref_d,ref_d);
			    
			    if (cadena[6]>57 || cadena[6]<48) cadena[6]='0';
			    if (cadena[7]>57 || cadena[7]<48) cadena[7]='0';
			    if (cadena[8]>57 || cadena[8]<48) cadena[8]='0';
			    ref_fd=(cadena[6]-48)*1000+(cadena[7]-48)*100+(cadena[8]-48)*10+(cadena[9]-48);
			    eeprom_write_word((uint16_t*)direccion_ref_fd,ref_fd);
			    
			    if (cadena[10]>57 || cadena[10]<48) cadena[10]='0';
			    if (cadena[11]>57 || cadena[11]<48) cadena[11]='0';
			    if (cadena[12]>57 || cadena[12]<48) cadena[12]='0';
			    ref_f=(cadena[10]-48)*1000+(cadena[11]-48)*100+(cadena[12]-48)*10+(cadena[13]-48);
			    eeprom_write_word((uint16_t*)direccion_ref_f,ref_f);
			    
			    if (cadena[14]>57 || cadena[14]<48) cadena[14]='0';
			    if (cadena[15]>57 || cadena[15]<48) cadena[15]='0';
			    if (cadena[16]>57 || cadena[16]<48) cadena[16]='0';
			    ref_fi=(cadena[14]-48)*1000+(cadena[15]-48)*100+(cadena[16]-48)*10+(cadena[17]-48);
			    eeprom_write_word((uint16_t*)direccion_ref_fi,ref_fi);
			    
			    if (cadena[18]>57 || cadena[18]<48) cadena[18]='0';
			    if (cadena[19]>57 || cadena[19]<48) cadena[19]='0';
			    if (cadena[20]>57 || cadena[20]<48) cadena[20]='0';
			    ref_i=(cadena[18]-48)*1000+(cadena[19]-48)*100+(cadena[20]-48)*10+(cadena[21]-48);
			    eeprom_write_word((uint16_t*)direccion_ref_i,ref_i);
			    
			    if (cadena[22]>57 || cadena[22]<48) cadena[22]='0';
			    if (cadena[23]>57 || cadena[23]<48) cadena[23]='0';
			    if (cadena[24]>57 || cadena[24]<48) cadena[24]='0';
			    ref_col=(cadena[22]-48)*1000+(cadena[23]-48)*100+(cadena[24]-48)*10+(cadena[25]-48);
			    eeprom_write_word((uint16_t*)direccion_ref_col,ref_col);
		    }

		    else if (cadena[1]=='c') //recibe constantes de controlador 1cxxxxxxxxxxfi
		    {
			    kp=(cadena[2]-48)+(cadena[4]-48)*0.1+(cadena[5]-48)*0.01;
			    eeprom_write_float((float*)direccion_kp,kp);
			    ki=(cadena[6]-48)+(cadena[8]-48)*0.1+(cadena[9]-48)*0.01;
			    eeprom_write_float((float*)direccion_ki,ki);
			    kd=(cadena[10]-48)+(cadena[12]-48)*0.1+(cadena[13]-48)*0.01;
			    eeprom_write_float((float*)direccion_kd,kd);
			    set_point=(cadena[14]-48)+(cadena[16]-48)*0.1+(cadena[17]-48)*0.01;
			    eeprom_write_float((float*)direccion_set_point,set_point);
			    target_power=(cadena[18]-48)*100+(cadena[19]-48)*10+(cadena[20]-48);
			    eeprom_write_float((float*)direccion_target_power,target_power);
		    }
		    else if (cadena[1]=='a')
		    {
			    switch(cadena[2])
			    {
				    case 'a':
				    movimiento(mov_avanzar,254,254);
				    break;
				    case 'd':
				    movimiento(mov_derecha,200,200);
				    break;
				    case 'i':
				    movimiento(mov_izquierda,200,200);
				    break;
				    case 'r':
				    movimiento(mov_retroceder,200,200);
				    break;
				    case 'p':
				    movimiento(mov_parar,0,0);
				    comenzar=0;
				    break;
				    case 'c':
				    comenzar=1;
				    break;
				    case 't': //frente
				    lala=0;
				    break;
				    case 'f': //iz
				    lala=1;
				    break;
				    case 'h'://de
				    lala=2;
				    break;
				    case 'g': //detras
				    lala=3;
				    break;
				    case 'v': //voltaje
				    lala=4;
				    break;
			    }
		    }
	    }
    }
    else
    {
	    cadena[j++]=dato_recibido;   //SE GUARDA EL DATO EN UNA POSICION DEL VECTOR
	    control=0;
    }
    
    if (dato_recibido=='f' && control==0)
    {
	    control=1;
    }

}


void movimiento(uint8_t mov,uint8_t velocidad_der,uint8_t velocidad_iz)  //mov_xxxx
{
	//pwm de 0 a 255
	switch (mov)
	{
	case 1: //avanzar
		PORT_MOTORES|=(1<<MOTOR1A);
		PORT_MOTORES&=~(1<<MOTOR1B);
		PORT_MOTORES|=(1<<MOTOR2A);
		PORT_MOTORES&=~(1<<MOTOR2B);
		
	break;
	case 2: //retroceder
		PORT_MOTORES|=(1<<MOTOR1B);
		PORT_MOTORES&=~(1<<MOTOR1A);
		PORT_MOTORES|=(1<<MOTOR2B);
		PORT_MOTORES&=~(1<<MOTOR2A);
	break;
	case 3: //derecho
		PORT_MOTORES|=(1<<MOTOR1A);
		PORT_MOTORES&=~(1<<MOTOR1B);
		PORT_MOTORES|=(1<<MOTOR2B);
		PORT_MOTORES&=~(1<<MOTOR2A);
	
	break;
	case 4: //izquierdo
		PORT_MOTORES|=(1<<MOTOR1B);
		PORT_MOTORES&=~(1<<MOTOR1A);
		PORT_MOTORES|=(1<<MOTOR2A);
		PORT_MOTORES&=~(1<<MOTOR2B);
	break;		
	case 5: //parar
		PORT_MOTORES&=~(1<<MOTOR1A);
		PORT_MOTORES&=~(1<<MOTOR2A);
		PORT_MOTORES&=~(1<<MOTOR1B);
		PORT_MOTORES&=~(1<<MOTOR2B);
	break;	
	}
	PWMA=velocidad_der;
	PWMB=velocidad_iz;
}

//////////////////////////////////////////////////////////
void leds(void)
{
	if (ed==0)
	{
		PORT_LED1&=~(1<<LED1);
	}
	else
	{
		PORT_LED1|=(1<<LED1);
	}
	
	if (eizq==0)
	{
		PORT_LED2&=~(1<<LED2);
	}
	else
	{
		PORT_LED2|=(1<<LED2);
	}
	
	
}

void esperar5s (void)
{
	
	_delay_ms(400);
	PORT_LED5|=(1<<LED5);
	_delay_ms(400);
	PORT_LED5&=~(1<<LED5);
	_delay_ms(400);
	PORT_LED5|=(1<<LED5);
	_delay_ms(400);
	PORT_LED5&=~(1<<LED5);
	_delay_ms(400);
	PORT_LED5|=(1<<LED5);
	_delay_ms(300);
	PORT_LED5&=~(1<<LED5);
	_delay_ms(300);
	PORT_LED5|=(1<<LED5);
	_delay_ms(300);
	PORT_LED5&=~(1<<LED5);
	_delay_ms(300);
	PORT_LED5|=(1<<LED5);
	_delay_ms(310);
	PORT_LED5&=~(1<<LED5);	
}


void SensarBatery (uint8_t a)
{
	if (a<185)
	{
		PORT_LED1|=(1<<LED1);
		PORT_LED2|=(1<<LED2);
		PORT_LED3|=(1<<LED3);
		PORT_LED4|=(1<<LED4);
		PORT_LED5|=(1<<LED5);
		_delay_ms(100);
		PORT_LED1&=~(1<<LED1);
		PORT_LED2&=~(1<<LED2);
		PORT_LED3&=~(1<<LED3);
	    PORT_LED4&=~(1<<LED4);
		PORT_LED5&=~(1<<LED5);
		_delay_ms(100);
				
	}
	else
	{
		PORT_LED1|=(1<<LED1);
		PORT_LED2|=(1<<LED2);
		PORT_LED3|=(1<<LED3);
		PORT_LED4|=(1<<LED4);
		PORT_LED5|=(1<<LED5);		
		
	}
	
}
void primermov(uint8_t lala)
{
	switch (lala)
	{
		
		
		case 0: //esta de frente
		movimiento(mov_avanzar,254,254);
		DD_anterior=1;
		break;
		
		case 1: //esta derecha
		movimiento(mov_derecha,254,254);
		DD_anterior=1;
		break;
		
		case 2:  //esta a la iz
		movimiento(mov_izquierda,254,254);
		DI_anterior=1;
		break;
		
		case 3:  //esta detras
		movimiento(mov_derecha,254,254);
		DD_anterior=1;
		break;
		
		default:
		movimiento(mov_izquierda,254,254);
		DI_anterior=1;
	}	
}

void elegiraccion(void)
{
		switch (lala)
		{
			case 0: //esta de frente
			PORT_LED1|=(1<<LED1);
			PORT_LED2&=~(1<<LED2);
			PORT_LED3&=~(1<<LED3);
			PORT_LED4&=~(1<<LED4);
			PORT_LED5&=~(1<<LED5);
			break;
			
			case 1: //esta IZQUIERDA
			PORT_LED2|=(1<<LED2);
			PORT_LED1&=~(1<<LED1);
			PORT_LED3&=~(1<<LED3);
			PORT_LED4&=~(1<<LED4);
			PORT_LED5&=~(1<<LED5);
			break;
			
			case 2:  //esta a la DERECHA
			PORT_LED3|=(1<<LED3);
			PORT_LED2&=~(1<<LED2);
			PORT_LED1&=~(1<<LED1);
			PORT_LED4&=~(1<<LED4);
			PORT_LED5&=~(1<<LED5);
			break;
			
			case 3:  //esta detras
			PORT_LED4|=(1<<LED4);
			PORT_LED2&=~(1<<LED2);
			PORT_LED3&=~(1<<LED3);
			PORT_LED1&=~(1<<LED1);
			PORT_LED5&=~(1<<LED5);
			break;
			
			case 4:  //bateria  falta configurar						
			SensarBatery(Vlip);
			break;
		}	
}


void pid(void)
{
	//funciona solo si algun sensor detecta algo dentro de su rango
	if ((dentrorango[2]>0)||(dentrorango[3]>0)||(dentrorango[4]>0))
	{
		temp=0;
		
		error=(dentrorango[2]+dentrorango[3]+dentrorango[4])-set_point;
		I=I+error;
		D=error-error_anterior;
		ley=kp*error+ki*I+kd*D;
		PWMDER=target_power-ley;
		PWMIZQ=target_power+ley;
		
		if (PWMDER>254) PWMDER=255;
		if (PWMIZQ>254) PWMIZQ=255;
		if (PWMDER<0) PWMDER=0;
		if (PWMIZQ<0) PWMIZQ=0;
		error_anterior=error;
		
		if (negro==1) //esta en negro
		{
			movimiento(mov_avanzar,PWMDER,PWMIZQ);		
		}
		else
		{
			I=0;
			D=0;
			error_anterior=0;
			movimiento(mov_retroceder,250,250);
			_delay_ms(150);         //retrocede por 3s
			
		}
	}
	else
	//si ya no detecta, girar al ultimo lado que detecto
	{
		error_anterior=0;
		I=0;
		
		if (negro==1)
		{
			if (temp<3000)
			{
				if (DD_anterior==1) movimiento(mov_derecha,220,220);
				else if (DI_anterior==1) movimiento(mov_izquierda,220,220);
			}
			else movimiento(mov_avanzar,180,180);
				
		}
		else
		{
			movimiento(mov_retroceder,250,250);
			_delay_ms(150);
			temp=0;
		}		
					
	}

}
