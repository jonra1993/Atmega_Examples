/*
 * ultron_v1.c
 *
 * Created: 7/30/2017 3:50:15 PM
 * Author : jon_r
 */ 

#include <avr/io.h>

#include "funciones.h"

volatile uint8_t ref_d;
volatile uint8_t ref_fd;
volatile uint8_t ref_f;
volatile uint8_t ref_fi;
volatile uint8_t ref_i;
volatile uint8_t ref_col;
volatile uint8_t ref_lipo;

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

volatile uint8_t i=0;
volatile uint8_t negro=1;
volatile uint8_t dentrorango[4]={0,0,0,0};
volatile uint8_t lecoff[4]={0,0,0,0};
volatile uint8_t medidas[6]={0,0,0,0,0,0};
volatile int temp;

void pid(void);
	
int main(void)
{

	declaraciones();
	conf_pwm();
//	conf_serial_com();
	conf_int_pulsadores();
//	adc_init();
	sei();

	lala=0;
	comenzar=0;
	u=0;
//	conf_timer1();
		
	do
	{
		//movimiento(mov_parar,0,0);
		elegiraccion(lala, vol_lipo, ref_lipo); //FRENTE, IZQUIERDA, DERECHA, ATRAS
	} while (comenzar==0);	
	
	esperar5s();	
	movimiento(mov_avanzar,240,240);
	//primermov(lala);	
	
    /* Replace with your application code */
    while (1) 
    {
		if (i==1)
		{
		//	pid();
		}		
    }
}

ISR(INT1_vect)
{
	_delay_ms(20);
	uint8_t aux =PIND&(0B1000);
	if (aux==0)
	{
		lala++;
		if (lala>4) lala=0;
	}
}

ISR(INT0_vect)
{
	_delay_ms(20);
	uint8_t aux=PIND&(0B100);
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

ISR (TIMER1_COMPA_vect) //cada 1ms
{
	temp++;
	vol_lipo=lectura_sensor(Vlipo);
	col=lectura_sensor(color);
	if (col<ref_col) negro=0; //esta en blanco
	else negro=1;         //esta en negro
	
	if (i==0)
	{
		PORT_INFR|=0b11011; 
		//rec_der,rec_f_de,rec_f_iz,rec_iz
		for (int k=0; k<4; k++ )
		{
			uint8_t t=0;
			if (k>1) t=k+1;
			else t=k;
			
			medidas[k]=lectura_sensor(t)-lecoff[k];
		}
		medidas[4]=col;
		medidas[5]=vol_lipo;
		i=1;
	}
	else
	{
		PORT_INFR&=~((1<<OUT_D)|(1<<OUT_FD)|(1<<OUT_FI)|(1<<OUT_I)); //apaga todo
		for (int k=0; k<4; k++ )
		{
			uint8_t t=0;
			if (k>1) t=k+1;
			else t=k;
			
			lecoff[k]=lectura_sensor(t);			
		}
		i=0;
	}
	
	if (medidas[0]>ref_d)
	{
		dentrorango[0]=2;
		DD_anterior=0;
		DI_anterior=1;
	}
	else dentrorango[0]=0;

	if (medidas[1]>ref_fd)
	{
		dentrorango[1]=1;
		DD_anterior=1;
		DI_anterior=0;
	}
	else dentrorango[1]=0;
		
	
	if (medidas[2]>ref_fi)
	{
		dentrorango[2]=-1;
		DD_anterior=1;
		DI_anterior=0;
	}  //f
	else dentrorango[2]=0;
	
	
	if (medidas[3]>ref_i)
	{
		dentrorango[3]=-2;
		DD_anterior=0;
		DI_anterior=1;
	}
	else dentrorango[3]=0;
	
}

ISR (TIMER1_COMPB_vect)  //cada 1ms
{

}

ISR (USART0_RX_vect)
{
	while ((UCSR0A & 0B10000000)==0)  ;//ESPERA A Q TERMINE DE RECIBIR
	uint8_t dato_recibido=UDR0;
	switch (dato_recibido)
	{
	case 'a':
		serial_transmit(medidas[0]);
	break;
	case 'b':
		serial_transmit(medidas[1]);
	break;
	case 'c':
		serial_transmit(medidas[2]);
	break;
	case 'd':
		serial_transmit(medidas[3]);
	break;				
	case 'e':
		serial_transmit(medidas[4]);
	break;
	case 'f':
		serial_transmit(medidas[5]);
	break;
	case 'g':
		movimiento(mov_avanzar,200,200);
	break;
	case 'h':
		movimiento(mov_derecha,200,200);
	break;	
	case 'i':
	movimiento(mov_izquierda,200,200);
	break;
	case 'j':
	movimiento(mov_retroceder,200,200);
	break;
	case 'k':
	movimiento(mov_parar,200,200);
	break;
	}
}

void pid(void)
{
	//funciona solo si algun sensor detecta algo dentro de su rango
	if ((dentrorango[0]>0)||(dentrorango[1]>0)||(dentrorango[2]>0)||(dentrorango[3]>0))
	{
		temp=0;		
		error=set_point-(dentrorango[2]+dentrorango[3]+dentrorango[4]);
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
			_delay_ms(150);         			
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
		}
		else
		{
			movimiento(mov_retroceder,250,250);
			_delay_ms(150);
			temp=0;
		}
		
	}

}