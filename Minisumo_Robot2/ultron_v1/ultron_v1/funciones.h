/*
 * funciones.h
 *
 * Created: 7/30/2017 3:52:39 PM
 *  Author: jon_r
 */ 


#ifndef FUNCIONES_H_
#define FUNCIONES_H_

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/eeprom.h>

//Definiciones de los motores
#define PORT_MOTORES PORTD
#define MOTOR1A 6
#define MOTOR1B 7
#define MOTOR2A 4
#define MOTOR2B 5

#define mov_avanzar	   2
#define mov_retroceder 1
#define mov_derecha    3
#define mov_izquierda  4
#define mov_parar      5

//Definicion de pwms
#define PWMA OCR0A
#define PWMB OCR0B

typedef uint8_t ioport_port_t;
typedef uint8_t ioport_pin_t;

volatile uint8_t comenzar;
volatile uint8_t lala;
//volatile uint8_t Vlip=0;
volatile uint8_t vol_lipo;
volatile uint8_t u;
volatile uint8_t DD_anterior;
volatile uint8_t DI_anterior;
volatile int temp;
volatile uint16_t col;

#define receptor_derecho 0
#define receptor_frontal_derecho 1
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

//Definicion de emisores infrarojos
#define PORT_INFR PORTC
#define OUT_D      0
#define OUT_FD     1
#define OUT_FI     3
#define OUT_I      4
     


void declaraciones(void);
void conf_int_pulsadores(void);
void conf_timer1(void);
void conf_pwm(void);

void conf_serial_com(void);
void serial_transmit(unsigned char dato);
void adc_init(void);
void set_one (ioport_port_t port, ioport_pin_t pin);
void set_port (ioport_port_t port, ioport_pin_t pines);
int lectura_sensor(uint8_t canal);
void movimiento(uint8_t mov,uint8_t velocidad_der,uint8_t velocidad_iz);  //mov_xxxx
void tostring(char str[], int num);
void elegiraccion(uint8_t lala, uint8_t vol_lipo,uint8_t ref_lipo);
void esperar5s (void);
void primermov(uint8_t lala);


#endif /* FUNCIONES_H_ */