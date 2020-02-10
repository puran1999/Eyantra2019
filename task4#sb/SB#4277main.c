#define F_CPU 16000000UL
#define USART0_ENABLED

#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
#include <stdlib.h>
#include <avr/interrupt.h>	

#define PIN_ADCR			PC0
#define PIN_ADCC			PC1
#define PIN_ADCL			PC2
#define in11				PB5
#define in12				PB4
#define in21				PD2
#define in22				PD4
#define pwm_servo			PD6
#define pwm_motor1			PB3
#define pwm_motor2			PD3
#define buzz				PC3

//initializing motor pins
void motor_init(void){
	
	DDRB    |= (1 << in11);//in1
	DDRB    |= (1 << in12);//in1
	DDRB    |= (1 << pwm_motor1);
	DDRD    |= (1 << in21);//in2
	DDRD    |= (1 << in22);//in2
	DDRD    |= (1 << pwm_motor2);//pwm2
}	

//pin values for motors to move in clockwise motion
void clock(void){
	
	PORTB   |= (1 << in11);
	PORTB   &= ~(1 << in12);
	PORTB   |= (1 << pwm_motor1);
	PORTD   |= (1 << in21);
	PORTD   &= ~(1 << in22);
	PORTD   |= (1 << pwm_motor2);
}

//pin values to stop motors
void motorsstop(void){
	
	PORTB   &= ~(1 << in11);
	PORTB   &= ~(1 << in12);
	PORTD   &= ~(1 << in21);
	PORTD   &= ~(1 << in22);
}

//pin values for motors to move in anticlockwise motion
void anticlock(void){
	
	PORTB   &= ~(1 << in11);
	PORTB   |= (1 << in12);
	PORTB   |= (1 << pwm_motor1);
	PORTD  &= ~(1 << in21);
	PORTD   |= (1 << in22);
	PORTD   |= (1 << pwm_motor2);
}

//switching off comparator and initialising ADC
void adc_init(){
	
	ACSR = (1 << ACD);   	
	ADMUX = (1 << ADLAR);
	ADCSRA = ((1 << ADEN) |  (1 << ADPS2 | 1 << ADPS1)) ;
}

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

//initializing ADC pins to use for white line sensor
void adc_pin_config (void){
	
	DDRC &= ~(1 << PIN_ADCR); //set PORTC direction as input
	DDRC &= ~(1 << PIN_ADCC);
	DDRC &= ~(1 << PIN_ADCL);
	PORTC &= ~(1 << PIN_ADCR); //set PORTC pins floating
	PORTC &= ~(1 << PIN_ADCC);
	PORTC &= ~(1 << PIN_ADCL);
}

//timer 2 used to generate pwm for both the motors. prescaler used =1028. fast pwm mode
void timer2_init()
{
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

//timer 0 used to generate pwm for servomotor. prescaler used = 1024. fast pwm mode
void timer0_init()
{
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

//initializing servomotor pins
void pwm_servomotor_init(void){
	
	DDRD    |= (1 << PD6);
	PORTD   |= (1 << PD6);
}

//setting pwm for servomotor
void servomotor_pwm (unsigned char servo){
	OCR0A = (unsigned char)servo; 	
}

//setting pwm for outermotor
void outermotor (unsigned char motor2){
	OCR2A = (unsigned char)motor2; 	
}

//setting pwm for innermotor
void innermotor (unsigned char motor){
	OCR2B =  (unsigned char)motor; 	
}

//initializing timers and ADC
void init_devices (void) {
	
	timer2_init();
	adc_pin_config();
	adc_init();
		
}

void pwm_servomotor_down(void){
	
	PORTD   |= (1 << PD6);
	
}

void timer0stop(void){
	TCCR0B=0x00;
	TCCR0A=0x00;
}

//program to control servomotor for striking mechanism
void striking(void){
	timer0_init();
	pwm_servomotor_init();
	servomotor_pwm(15);
	_delay_ms(1500);
	servomotor_pwm(5);
	_delay_ms(800);
	servomotor_pwm(60);
	_delay_ms(1500);
	servomotor_pwm(15);
	_delay_ms(2000);
	pwm_servomotor_down();
	timer0stop();
}

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

//reading byte received on XBee
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


float ADC_ValueR, ADC_ValueC, ADC_ValueL,w ;
int node_counter = 0, b = 1, sens=0;
char strr[20];

int main(void) {
	
	int src = 1, final_des = 1;
	int des, diff, next;
	char rx_byte;
	uart0_init(UART_BAUD_SELECT(9600, F_CPU));
	uart0_flush();
	init_devices();
	
	

	while(1)
	{
		rx_byte = uart0_readByte();
		uart0_putc('0');

		if(rx_byte >= '1' && rx_byte <= '9'){
			des= (int)rx_byte;
			diff = des  ;
			if (abs(diff) <= 6){
				next =	diff;
			}
			else{
				next = (abs(diff) - 9);
			}
			src = des;
			
			while(1){
			ADC_ValueR = ADC_Conversion(0);
			ADC_ValueC = ADC_Conversion(1);
			ADC_ValueL = ADC_Conversion(2);
			
			w = ADC_ValueR - ADC_ValueL;
			w*= 0.25;		
			
			motor_init();
			clock();
			outermotor(180 - (w/2));//180
			innermotor(48 + (w/2));	//48
			
			sens = ADC_ValueL + ADC_ValueC + ADC_ValueR;
			
			sprintf(strr, "%d,%d,%d,%d,%d,%d", b,node_counter,sens,next,des,diff);
			uart0_puts(strr);
			uart0_puts("\n");
			
			if ( b == 1 && sens < 250)
			{
				b = 0;
				node_counter++;
				if (node_counter == next-39)
				{
					_delay_ms(100);
					motorsstop();
					_delay_ms(50);
					buzzer(1);
					_delay_ms(200);
					buzzer(1);
					striking();
					_delay_ms(300);
					buzzer(2);
					node_counter = 0;
					break;
					
				}
			}
			else if( b == 0 && sens > 300)
			{
				b = 1;
			}
		
			
			}			
			
		}
		
		else if(rx_byte == '0'){
			diff = final_des - src;
			if (abs(diff) <= 5){
				next =	diff -1 ;
			}
			else{
				next = (char)(abs(diff) - 9);
			}
			while(1){
			ADC_ValueR = ADC_Conversion(0);
			ADC_ValueC = ADC_Conversion(1);
			ADC_ValueL = ADC_Conversion(2);
			
			w= ADC_ValueR - ADC_ValueL;
			w*= 0.25;
			
			clock();
			outermotor(180 - (w/2));//180
			innermotor(48 + (w/2));	//48
			
			sens = ADC_ValueL + ADC_ValueC + ADC_ValueR;
			
			if ( b == 1 && sens < 250)
			{
				b = 0;
				node_counter++;
				if (node_counter == next-39)
				{
					motorsstop();
					buzzer(10);
					break;
				}
			}
			else if( b == 0 && sens > 300)
			{
				b = 1;
			}
			}			
			
		}
		
		
	}

	return 0;
	
}
