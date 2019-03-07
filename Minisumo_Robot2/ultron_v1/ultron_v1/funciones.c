/*
 * funciones.c
 *
 * Created: 7/30/2017 3:51:50 PM
 *  Author: jon_r
 */ 

#include "funciones.h"

void declaraciones(void)
{
	DDRA = 0B10000000;
	DDRC = 0B11111111;
	DDRD = 0B11110010;
	PORTD= 0B00001100;  //ACTIVA PULL/UP PULSADORES
	DDRB = 0B00011110;
}

void conf_int_pulsadores(void)
{
	EICRA=0B00001010;
	EIMSK=0B00000011;
}

void conf_timer1(void)
{
	TCCR1A=0B00000000;  //modo contador
	TCCR1B=0B00001010; //prescalador 8
	OCR1A=500;  //a f de 1khz cada 1ms  a 100
	TIMSK1=0B00000010;
}

void conf_pwm(void)
{
	TCCR0A=0B10100011;  //FAST PWM
	TCCR0B=0B00000010;
	PWMA=0;
	PWMB=0;
}
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

void adc_init(void)
{
	ADMUX = (1<<REFS0); // AREF = AVcc
	// ADC Enable and prescaler of 64
	// 8000000/64 = 125000
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
	DIDR0=0B11111;

}

void set_one (ioport_port_t port, ioport_pin_t pin)
{
	port|=(1<<pin);
}

void set_zero(ioport_port_t port, ioport_pin_t pin)
{
	port&=~(1<<pin);
	
}

void set_port (ioport_port_t port, ioport_pin_t pines)
{
	port=pines;	
}

int lectura_sensor(uint8_t canal)
{
	canal = canal & 0b00000111;
	ADMUX =(1<<REFS0)|(1<<REFS1)|(1<<ADLAR)|canal; //REF 2.56V
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC)); //espera que complete la conversion
	return ADCH;   //confi 8bits
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

void SensarBatery (uint8_t vol_lipo,uint8_t ref_lipo)
{
	if (vol_lipo<ref_lipo)
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

void elegiraccion(uint8_t lala, uint8_t vol_lipo,uint8_t ref_lipo)
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
		//SensarBatery(Vlip);
		break;
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