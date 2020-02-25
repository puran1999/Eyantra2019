/* 
 * Team Id: 4277
 * Author List: Jagmeet Singh
 * Filename:  SB#4277main.c
 * Theme:  Supply Bot
 * Functions:  
 * Global Variables: 
 */  

#define F_CPU 16000000UL
#define USART0_ENABLED

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "uart.h"

#define PIN_ADCR			PC0      //ADC pin to read analog values from right side of white line sensor  
#define PIN_ADCC			PC1		 //ADC pin to read analog values from center of white line sensor  
#define PIN_ADCL			PC2		 //ADC pin to read analog values from left side of white line sensor  
#define in11				PB5		 //1st input pin for motor driver (outer motor) 
#define in12				PB4		 //2nd input pin for motor driver (outer motor)
#define in21				PD2		 //3rd input pin for motor driver (inner motor)
#define in22				PD4		 //4th input pin for motor driver (inner motor)
#define pwm_servo			PD6		 //PWM pin for servo motor
#define pwm_motor1			PB3		 //ENA(enable pin of motor driver for inner motor - provides PWM)
#define pwm_motor2			PD3		 //ENB(enable pin of motor driver for outer motor - provides PWM) 
#define buzz				PC3		 //pin used to provide signal to buzzer

float ADC_ValueR, ADC_ValueC, ADC_ValueL,w ,ValueC;

/*
* Function Name: motors_init
* Input: None
* Output: Pins required for DC motors will be initialized.
* Logic: For a pin to function as output, direction registers are used to assign value '1' 
*		 to the pin to become an output pin.
* Example Call: motors_init();
*/
void motors_init(void){	
	DDRB    |= (1 << in11);
	DDRB    |= (1 << in12);
	DDRB    |= (1 << pwm_motor1);
	DDRD    |= (1 << in21);
	DDRD    |= (1 << in22);
	DDRD    |= (1 << pwm_motor2);
}

/*
* Function Name: forward_motion
* Input: None
* Output: The motors start to move in the forward direction.
* Logic: Motor Driver works on concept of H-Bridge, on basis of which we move the status of one 
*		 of input pins of each motor to HIGH which leads to motion in motor.
* Example Call: forward_motion();
*/
void forward_motion(void){
	PORTB   |= (1 << in11);
	PORTB   &= ~(1 << in12);
	PORTD   |= (1 << in21);
	PORTD   &= ~(1 << in22);
}

/*
* Function Name: forward_motion
* Input: None
* Output: The motors start to move in the forward direction.
* Logic: Motor Driver works on concept of H-Bridge, on basis of which we move the status of one
*		 of input pins of each motor to HIGH which leads to motion in motor.
* Example Call: forward_motion();
*/
void motors_stop(void){	
	PORTB   &= ~(1 << in11);
	PORTB   &= ~(1 << in12);
	PORTD   &= ~(1 << in21);
	PORTD   &= ~(1 << in22);
}

/*
* Function Name: adc_init
* Input: None
* Output: Initializes ADC and switches off comparator.
* Logic: 
* Example Call: adc_init();
*/
void adc_init(){
	ACSR = (1 << ACD);
	ADMUX = (1 << ADLAR);
	ADCSRA = ((1 << ADEN) |  (1 << ADPS2 | 1 << ADPS1)) ;
}

/*
* Function Name: ADC_Conversion
* Input: None
* Output: Initializes ADC and switches off comparator.
* Logic:
* Example Call: ADC_Conversion();
*/
//converting analog values from white line sensor to digital. using left shift mode(ADLAR=1). using only higher 8 bits as lower bits won't make that big of a change.
unsigned char ADC_Conversion(unsigned char Ch){
	unsigned char a;
	Ch = Ch & 0b00000111;
	ADMUX = 0x20 | Ch;
	ADCSRA |= (1 << ADSC);
	while((ADCSRA & (1 << ADIF) ) == 0);
	a = ADCH;
	ADCSRA |= (1 << ADIF);
	return a;
}

/*
* Function Name: adc_pin_config
* Input: None
* Output: Initializes ADC pins as input and sets them on floating.
* Logic:
* Example Call: adc_pin_config();
*/
void adc_pin_config (void){
	DDRC &= ~(1 << PIN_ADCR); //set PORTC direction as input
	DDRC &= ~(1 << PIN_ADCC);
	DDRC &= ~(1 << PIN_ADCL);
	PORTC &= ~(1 << PIN_ADCR); //set PORTC pins floating
	PORTC &= ~(1 << PIN_ADCC);
	PORTC &= ~(1 << PIN_ADCL);
}

/*
* Function Name: timer2_init
* Input: None
* Output: Initializes timer 2.
* Logic:
* Example Call: timer2_init();
*/
//timer 2 used to generate pwm for both the motors. prescaler used =1028. fast pwm mode
void timer2_init(){
	cli();
	TCCR2B = 0x00;
	
	TCNT2 = 0xFF;
	OCR2A = 0xFF;
	OCR2B = 0xFF;
	
	TCCR2A |= (1 << COM2A1);
	TCCR2A &= ~(1 << COM2A0);
	TCCR2A |= (1 << COM2B1);
	TCCR2A &= ~(1 << COM2B0);

	TCCR2A |= (1 << WGM20);
	TCCR2A |= (1 << WGM21);
	TCCR2B &= ~(1 << WGM22);
	
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
	
	sei();
}

/*
* Function Name: timer0_init
* Input: None
* Output: Initializes timer 0.
* Logic:
* Example Call: timer0_init();
*/
//timer 0 used to generate pwm for servomotor. prescaler used = 1024. fast pwm mode
void timer0_init(){
	cli();
	
	TCCR0B = 0x00;
	
	TCNT0 = 0xFF;
	OCR0A = 0xFF;
	
	TCCR0A |= (1 << COM0A1);
	TCCR0A &= ~(1 << COM0A0);
	
	TCCR0A |= (1 << WGM00);
	TCCR0A |= (1 << WGM01);
	TCCR0B &= ~(1 << WGM02);
	
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TCCR0B &= ~(1 << CS01);
	
	sei();
}

/*
* Function Name: pwm_servomotor_init
* Input: None
* Output: 
* Logic:
* Example Call: pwm_servomotor_init();
*/
//initializing servomotor pins
void pwm_servomotor_init(void){
	DDRD    |= (1 << PD6);
	PORTD   |= (1 << PD6);
}


/*
* Function Name: servomotor_pwm
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/
//setting pwm for servomotor
void servomotor_pwm (unsigned char servo){
	OCR0A = (unsigned char)servo;
}

/*
* Function Name: servomotor_pwm
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/
//setting pwm for outermotor
void outermotor (unsigned char motor2){
	OCR2A = (unsigned char)motor2;
}

/*
* Function Name: servomotor_pwm
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/
//setting pwm for innermotor
void innermotor (unsigned char motor){
	OCR2B =  (unsigned char)motor;
}

/*
* Function Name: servomotor_pwm
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/
//initializing timers and ADC
void init_devices (void) {
	timer2_init();
	adc_pin_config();
	adc_init();
}

/*
* Function Name: servomotor_pwm
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/
void pwm_servomotor_stop(void){
	PORTD   |= (1 << PD6);
}

/*
* Function Name: servomotor_pwm
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/
void timer0stop(void){
	TCCR0B=0x00;
	TCCR0A=0x00;
}

/*
* Function Name: servomotor_pwm
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/
//program to control servomotor for striking mechanism
void striking(void){
	timer0_init();
	pwm_servomotor_init();
	servomotor_pwm(15);
	_delay_ms(1500);
	servomotor_pwm(4);
	_delay_ms(800);
	servomotor_pwm(55);
	_delay_ms(1500);
	servomotor_pwm(15);
	_delay_ms(2000);
	pwm_servomotor_stop();
	timer0stop();
}

/*
* Function Name: servomotor_pwm
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/
//Buzzer program
void buzzer(unsigned int x){
	int i;
	DDRC |= (1<<buzz);
	PORTC &= ~(1<<buzz);
	for (i = 0; i<x; i++)
	{
		_delay_ms(500);
	}
	PORTC |= (1<<buzz);
}

/*
* Function Name: servomotor_pwm
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/
char uart0_readByte(void){
	uint16_t rx;
	uint8_t rx_status, rx_data;

	rx = uart0_getc();
	rx_status = (uint8_t)(rx >> 8);
	rx = rx << 8;
	rx_data = (uint8_t)(rx >> 8);
	uart0_flush();
	if(rx_status == 0 && rx_data != 0){
		return rx_data;
	} else {
		return - 1;
	}
}

/*
* Function Name: servomotor_pwm
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/
void antimotion_rotate(){
	PORTB   |= (1 << in11);
	PORTD   |= (1 << in22);
	PORTB   &= ~(1 << in12);
	PORTD   &= ~(1 << in21);
	outermotor(255);
	innermotor(255);
	_delay_ms(300);
	while(1)
	{
		ValueC = ADC_Conversion(1);		
		if (ValueC<70 )
		{
			motors_stop();
			break;
		}
	}
}

/*
* Function Name: servomotor_pwm
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/
void rotate(){	
	motors_init();
	PORTB   |= (1 << in12);
	PORTD   |= (1 << in21);
	PORTB   &= ~(1 << in11);
	PORTD   &= ~(1 << in22);
	outermotor(255);
	innermotor(255);
	_delay_ms(100);
	while(1)
	{
		ValueC = ADC_Conversion(1);	
		if (ValueC<70 )
		{ 
			motors_stop();
			break;
		}
	}
}

/*
* Function Name: servomotor_pwm
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/
void move(int next_node, int direction )
{
	int node_counter = 0 , sens =0, b=1;
	int outermotor_pwm, innermotor_pwm;
	
	if (direction==1)		//1=anticlock
	{
		outermotor_pwm =61;
		innermotor_pwm=230;
	}
	else
	{
		outermotor_pwm =230;
		innermotor_pwm=61;
	}
	while(1)
	{		
		ADC_ValueR = ADC_Conversion(0);
		ADC_ValueC = ADC_Conversion(1);
		ADC_ValueL = ADC_Conversion(2);
		
		w = ADC_ValueR - ADC_ValueL;
		w*= 0.25;
		
		motors_init();
		outermotor(outermotor_pwm - (w/2));
		innermotor(innermotor_pwm + (w/2));
		forward_motion();
			
		
		sens = ADC_ValueL + ADC_ValueC + ADC_ValueR;
		
		
		if ( b == 1 && sens < 250)
		{
			b = 0;
			node_counter++;
			if (node_counter == next_node)
			{	
				if (direction==1)
				{
					_delay_ms(800);
				}
				else
				{
					_delay_ms(200);	
				}
				motors_stop();
				break;
			}
		}
		else if( b == 0 && sens > 300)
		{
			b = 1;
		}
	}
}
	
	
/*
* Function Name: main
* Input: None
* Output:
* Logic:
* Example Call: servomotor_pwm(15);
*/	
int main(void) {
	
	int src = 1, final_des = 1;
	int des, diff, next=3;
	char rx_byte;
	uart0_init(UART_BAUD_SELECT(9600, F_CPU));
	uart0_flush();
	init_devices();
	
	while(1)
	{
		rx_byte = uart0_readByte();
		uart0_putc('0');
	
		
		if(rx_byte >= '1' && rx_byte <= '9')
		{
			des= (int)rx_byte - 48 ;
			diff = des - src  ;
			if (abs(diff) < 5)
			{
				next =	diff ;
			}
			else if(diff>0) 
			{
				next =  diff -9 ;
			}
			else
			{
				next=diff+9;
			}
			
			src = des;
			
			
			while(1)
			{	
				if (next<0)
				{
					rotate();
					move(abs(next),1);
					antimotion_rotate();
					_delay_ms(50);
					buzzer(1);
					_delay_ms(200);
					buzzer(1);
					striking();
					_delay_ms(300);
					buzzer(2);
					break;
				}
				
				else 
				{
					move(abs(next),0);
					_delay_ms(50);
					buzzer(1);
					_delay_ms(200);
					buzzer(1);
					striking();
					_delay_ms(300);
					buzzer(2);
					break;
				}				
			}			
		}
		
		else if(rx_byte == '0')
		{
					
			diff = final_des - src  ;
			if (abs(diff) < 5)
			{
				next =	diff ;
			}
			else if(diff>0) 
			{
				next =  diff -9 ;
			}
			else
			{
				next=diff+9;
			}
			while(1)
			{
				if (next<0)
				{
					rotate();
					move(abs(next),1);
					antimotion_rotate();
					buzzer(10);
					break;
				}					
				else
				{
					move(abs(next),0);
					buzzer(10);
					break;
				}
			}
		}		
	}
	return 0;
}
