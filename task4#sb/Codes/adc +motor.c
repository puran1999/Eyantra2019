#define F_CPU 16000000UL		// Define Crystal Frequency of Uno Board

#include <avr/io.h>				// Standard AVR IO Library
#include <util/delay.h>			// Standard AVR Delay Library
#include <avr/interrupt.h>		// Standard AVR Interrupt Library


#define PIN_ADCR			PC0
#define PIN_ADCC			PC1
#define PIN_ADCL			PC2
#define in11				PB5
#define in12				PB4
#define in21				PD5
#define in22				PD4
#define pwm_servo			PD6
#define pwm_motor1			PB3
#define pwm_motor2			PD3
#define buzz				PB1


float ADC_ValueR, ADC_ValueC, ADC_ValueL,w ;
int node_counter;
int received_number=3;


void clock(void){
	    
	DDRB    |= (1 << in11);//in1
	PORTB   |= (1 << in11);
	DDRB    |= (1 << in12);//in1
	PORTB   &= ~(1 << in12);
	DDRB    |= (1 << pwm_motor1); //pwm1
	PORTB   |= (1 << pwm_motor1);
	
	DDRD    |= (1 << in21);//in2
	PORTD   |= (1 << in21);
	DDRD    |= (1 << in22);//in2
	PORTD   &= ~(1 << in22);
	DDRD    |= (1 << pwm_motor2);//pwm2
	PORTD   |= (1 << pwm_motor2);
}

void motorsstop(void){
	DDRB    |= (1 << PB5);
	PORTB   &= ~(1 << PB5);
	DDRB    |= (1 << PB4);
	PORTB   &= ~(1 << PB4);
	DDRB    |= (1 << PB3);
	PORTB   |= (1 << PB3);
	
	DDRD    |= (1 << PD6);
	PORTD   &= ~(1 << PD6);
	DDRD    |= (1 << PD5);
	PORTD   &= ~(1 << PD5);
	DDRD    |= (1 << PD3);
	PORTD   |= (1 << PD3);
}

void anticlock(void){
	
	DDRB    |= (1 << PB5);//in1
	PORTB   &= ~(1 << PB5);
	DDRB    |= (1 << PB4);//in1
	PORTB   |= (1 << PB4);
	DDRB    |= (1 << PB3); //pwm1
	PORTB   |= (1 << PB3);
	
	DDRD    |= (1 << PD6);//in2
	PORTD  &= ~(1 << PD6);
	DDRD    |= (1 << PD5);//in2
	PORTD   |= (1 << PD5);
	DDRD    |= (1 << PD3);//pwm2
	PORTD   |= (1 << PD3);
}

void adc_init(){
	ACSR = (1 << ACD);   	// Analog Comparator Disable; else ADC wont work
	ADMUX = (1 << ADLAR);
	// (turn ADC ON) | (set prescalar to 64 110)
	ADCSRA = ((1 << ADEN) |  (1 << ADPS2 | 1 << ADPS1)) ;
}

unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	
	// Extract Last 3 bits from Ch for ADMUX
	Ch = Ch & 0b00000111; //0x07
	ADMUX = 0x20 | Ch; // (Left Adjusted Output) | (ADMUX4:0)
	ADCSRA |= (1 << ADSC);		//Set start conversion bit
	// Wait for ADC conversion to complete; ADIF = 0, conversion going on; ADIF = 1, conversion complete
	while((ADCSRA & (1 << ADIF) ) == 0);
	// store ADC value in variable are return it.
	a = ADCH;
	ADCSRA |= (1 << ADIF); // IMP: clear ADIF (ADC Interrupt Flag) by writing 1 to it
	return a;
}

void adc_pin_config (void)
{
	DDRC &= ~(1 << PIN_ADCR); //set PORTC direction as input
	DDRC &= ~(1 << PIN_ADCC);
	DDRC &= ~(1 << PIN_ADCL);
	PORTC &= ~(1 << PIN_ADCR); //set PORTC pins floating
	PORTC &= ~(1 << PIN_ADCC);
	PORTC &= ~(1 << PIN_ADCL);
}

void timer2_init()
{
	cli(); //disable all interrupts
	
	TCCR2B = 0x00;	//Stop
	
	TCNT2 = 0xFF;	//Counter higher 8-bit value to which OCR2A value is compared with
	
	OCR2A = 0xFF;	//Output compare register low value for Led
	OCR2B = 0xFF;
	
	//  Clear OC2A, on compare match (set output to low level)
	TCCR2A |= (1 << COM2A1);
	TCCR2A &= ~(1 << COM2A0);
	TCCR2A |= (1 << COM2B1);
	TCCR2A &= ~(1 << COM2B0);

	// FAST PWM 8-bit Mode
	TCCR2A |= (1 << WGM20);
	TCCR2A |= (1 << WGM21);
	TCCR2B &= ~(1 << WGM22);
	
	// Set Prescalar to 64
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
	
	sei(); //re-enable interrupts
}

void timer0_init()
{
	cli(); //disable all interrupts
	
	TCCR0B = 0x00;	//Stop
	
	TCNT0 = 0xFF;	//Counter higher 8-bit value to which OCR2A value is compared with
	
	OCR0A = 0xFF;	//Output compare register low value for Led
	//OCR2B =0xFF;
	//  Clear OC2A, on compare match (set output to low level)
	TCCR0A |= (1 << COM0A1);
	TCCR0A &= ~(1 << COM0A0);
	//TCCR2A |= (1 << COM2B1);
	//TCCR2A &= ~(1 << COM2B0);

	// FAST PWM 8-bit Mode
	TCCR0A |= (1 << WGM00);
	TCCR0A |= (1 << WGM01);
	TCCR0B &= ~(1 << WGM02);
	
	// Set Prescalar to 64
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TCCR0B &= ~(1 << CS01);
	sei(); //re-enable interrupts
}

void pwm_servomotor_init(void){
	
	DDRD    |= (1 << PD6);
	PORTD   |= (1 << PD6);
	
}

void servomotor_pwm (unsigned char red_led){
	OCR0A = (unsigned char)red_led; 	// active low thats why subtracting by 255
}


void outermotor (unsigned char motor2){
	OCR2A = (unsigned char)motor2; 	// active low thats why subtracting by 255
}

void innermotor (unsigned char motor){
	OCR2B =  (unsigned char)motor; 	// active low thats why subtracting by 255
}

//use this function to initialize all devices
void init_devices (void) {
	
	timer2_init();
	adc_pin_config();
	adc_init();
	timer0_init();
}

void striking(void)
{
	pwm_servomotor_init();
	servomotor_pwm(15);
	_delay_ms(1500);
	servomotor_pwm(0);
	_delay_ms(800);
	servomotor_pwm(50);
	_delay_ms(1500);
	servomotor_pwm(15);
	_delay_ms(2000);
	
}

void buzzer(unsigned int x)
{	int i;
	DDRB |= (1<<buzz);
	PORTB |= (1<<buzz);
	for (i=0;i<=x;i++)
	{
		_delay_ms(1000);
	}
	PORTB &= ~(1<<buzz);
}

int main(){
	init_devices();
	_delay_ms(2000);
	while(1){
		
		ADC_ValueR = ADC_Conversion(0);
		ADC_ValueC = ADC_Conversion(1);
		ADC_ValueL = ADC_Conversion(2);
		
		w=ADC_ValueR-ADC_ValueL;
		w*= 0.25;
		clock();
		
		outermotor(180+(w/2));
		innermotor(48-(w/2));	
		
		_delay_ms(200);
		
		if ( ADC_ValueL+ADC_ValueC+ADC_ValueR <=375 )	
		{
			node_counter++;
			if (node_counter==received_number)
			{   
				motorsstop();
				buzzer(1);
				striking();
				buzzer(1);
				_delay_ms(200);
				buzzer(1);
			}
		}			
			
			
			
		
	}
	
	return 0;
}
