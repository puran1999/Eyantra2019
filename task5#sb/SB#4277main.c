/* 
 * Team Id: 4277
 * Author List: Jagmeet Singh
 * Filename:  SB#4277main.c
 * Theme:  Supply Bot
 * Functions:  
 * Global Variables: ValueC
 */  

#define F_CPU 16000000UL
#define USART0_ENABLED

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "uart.h"

//Defining pin names
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
#define buzz				PC3		 //pin used to send signal to buzzer

float ValueC;			//ADC value from center of white line sensor

/*
* Function Name: motors_init
* Input: None
* Output: Pins required for DC motors will be initialized.
* Logic: For a pin to function as output, direction registers are used to assign value '1' 
*		 to the pin to become an output pin.
* Example Call: motors_init();
*/
void motors_init(void){	
	DDRB    |= (1 << in11);			//set pins as output
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
	PORTB   |= (1 << in11);		//set pin HIGH
	PORTB   &= ~(1 << in12);	//set pin LOW
	PORTD   |= (1 << in21);
	PORTD   &= ~(1 << in22);
}

/*
* Function Name: motors_stop
* Input: None
* Output: The motors stop.
* Logic: Motor Driver works on concept of H-Bridge, on basis of which if we need to stop the motors
*		 then we should either assign 1 to both input pins of a motor or 0 to both input pins of a motor.
*		 Here we have assigned 0 to both input pins of a motor.
* Example Call: motors_stop();
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
* Output: Switches off comparator and initializes ADC. 
* Logic: The function is run once and ADC is initialized and is now available to use for converting
*		 analog values to digital.
* Example Call: adc_init();
*/
void adc_init(){
	ACSR = (1 << ACD);
	//comparator is switched off
	ADMUX = (1 << ADLAR);
	//result is left adjusted
	ADCSRA = ((1 << ADEN) |  (1 << ADPS2 | 1 << ADPS1)) ;
	//ADEN switches ADC on
	//using ADPS2,ADPS1 prescalar is set to 64 
}

/*
* Function Name: ADC_Conversion
* Input: ADC_pin-> stores value of analog pin on which ADC conversion is needed(Expected range- 0 to 2).
* Output: Converts analog inputs into digital values. 
* Logic: Analog values are measured by white line sensor ,which are converted to digital values 
*		 using ADC. 328p has a 10 bit ADC.
*		 Here, left shift mode is used which is activated by assigning 1 to ADLAR bit, therefore 
*		 ADCH contains 8 higher bits which give values ranging between 0-255 according to readings
*		 from sensor. We don't read the lower 2 bits because they don't affect the result much
*		 because their bit weightage is less. 
* Example Call: ADC_Conversion(1);
*/
unsigned char ADC_Conversion(unsigned char ADC_pin){
	unsigned char digital_converted;
	ADC_pin = ADC_pin & 0b00000111;
	//pin number is extracted from ADC_pin
	ADMUX = 0x20 | ADC_pin;
	//MUX bits are assigned pin number where conversion is to take place
	ADCSRA |= (1 << ADSC);
	//start conversion
	while((ADCSRA & (1 << ADIF) ) == 0);
	//wait till conversion is completed
	digital_converted = ADCH;					
	//digital value converted from analog input is stored in digital_converted 
	ADCSRA |= (1 << ADIF);
	//ADIF is set when conversion is done, it needs to be cleared for next conversion
	return digital_converted;
}

/*
* Function Name: adc_pin_config
* Input: None
* Output: Initializes ADC pins as input and sets them on floating.
* Logic: Direction registers are used to assign whether a pin is an input or output.
*		 If output, Port register is used to assign whether pin is low or high.
*		 If input, Port register is used to assign whether pull up register on pin is on or not.
* Example Call: adc_pin_config();
*/
void adc_pin_config (void){
	DDRC &= ~(1 << PIN_ADCR); //set pins as input
	DDRC &= ~(1 << PIN_ADCC);
	DDRC &= ~(1 << PIN_ADCL);
	PORTC &= ~(1 << PIN_ADCR); //set pins on floating
	PORTC &= ~(1 << PIN_ADCC);
	PORTC &= ~(1 << PIN_ADCL);
}

/*
* Function Name: timer2_init
* Input: None
* Output: Initializes timer 2. Timer 2 generates PWM for DC motors.
* Logic: Only 1 timer i.e. timer 2 has been used to generate PWM for both DC motors by using 2 compare
*		 registers available in the timer. Value of both compare registers is compared with a set 
*		 value i.e. TCNT and then an output signal is generated.
*		 The code below is to initialize timer so that it can be used for above function.
* Example Call: timer2_init();
*/
void timer2_init(){
	cli();
	TCCR2B = 0x00;
	
	TCNT2 = 0xFF;
	OCR2A = 0xFF;
	//compare register for outermotor
	OCR2B = 0xFF;
	//compare register for innermotor
	
	TCCR2A |= (1 << COM2A1);
	TCCR2A &= ~(1 << COM2A0);
	TCCR2A |= (1 << COM2B1);
	TCCR2A &= ~(1 << COM2B0);
	
	//fast PWM mode
	TCCR2A |= (1 << WGM20);
	TCCR2A |= (1 << WGM21);
	TCCR2B &= ~(1 << WGM22);
	
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
	//prescalar set to 1024
	
	sei();
}

/*
* Function Name: timer0_init
* Input: None
* Output: Initializes timer 0. Timer 0 generates PWM for servomotor.
* Logic: The code below initializes the timer0 by setting up specific register bits.
* Example Call: timer0_init();
*/
void timer0_init(){
	cli();
	
	TCCR0B = 0x00;
	
	TCNT0 = 0xFF;
	OCR0A = 0xFF;
	//compare register for servomotor
	
	TCCR0A |= (1 << COM0A1);
	TCCR0A &= ~(1 << COM0A0);
	
	//fast PWM mode
	TCCR0A |= (1 << WGM00);
	TCCR0A |= (1 << WGM01);
	TCCR0B &= ~(1 << WGM02);
	
	TCCR0B |= (1 << CS02) | (1 << CS00);
	TCCR0B &= ~(1 << CS01);
	//prescalar set to 1024
	
	sei();
}

/*
* Function Name: pwm_servomotor_init
* Input: None
* Output: PWM pin for servomotor is initialized
* Logic: Direction registers are used to assign whether a pin is an input or output. 
*		 If output, Port register is used to assign whether pin is low or high. 
*		 If input, Port register is used to assign whether pull up register on pin is on or not.
* Example Call: pwm_servomotor_init();
*/
void pwm_servomotor_init(void){
	DDRD    |= (1 << pwm_servo);		//set pin as output
	PORTD   |= (1 << pwm_servo);		//set pin HIGH
}


/*
* Function Name: servomotor_pwm
* Input: servo->stores value of PWM to be given to servomotor to load and release the spring. 
* Output: PWM output on specific pin PD6
* Logic: OCR0A is compared with TCNT0 and PWM is generated accordingly.
* Example Call: servomotor_pwm(15);
*/
void servomotor_pwm (unsigned char servo){
	OCR0A = (unsigned char)servo;
}

/*
* Function Name: outermotor
* Input: motor2->stores value of PWM to be given to outermotor.
* Output: PWM output on specific pin PD3
* Logic: OCR2A is compared with TCNT2 and PWM is generated accordingly.
* Example Call: outermotor(105);
*/
void outermotor (unsigned char motor2){
	OCR2A = (unsigned char)motor2;
}

/*
* Function Name: innermotor
* Input: motor->stores value of PWM to be given to innermotor.
* Output: PWM output on specific pin PB3
* Logic: OCR2B is compared with TCNT2 and PWM is generated accordingly.
* Example Call: innermotor(105);
*/
void innermotor (unsigned char motor){
	OCR2B =  (unsigned char)motor;
}

/*
* Function Name: init_devices
* Input: None
* Output: Initializes timer2 and ADC
* Logic: This function calls other functions which initialize timer2 and ADC by assigning specific
*		 values to concerned registers.
* Example Call: init_devices();
*/
void init_devices (void) {
	timer2_init();
	adc_pin_config();
	adc_init();
}

/*
* Function Name: timer0stop
* Input: None
* Output: Stops Timer 0.
* Logic: Registers used to initialize and set up timer 0 are terminated by assigning 0 to them. 
* Example Call: timer0stop();
*/
void timer0stop(void){
	TCCR0B=0x00;
	TCCR0A=0x00;
}

/*
* Function Name: striking
* Input: None
* Output: Loaded spring is released, aid is striked and spring is loaded again.
* Logic: timer0 and PWM pin is initialized by calling the function then, specific PWM signal is 
*		 given to the pin,which in turn shoots/releases the loaded spring, which pushes forward
*		 the striker and the coin is hit. After that the spring is loaded again. Also, timer0 is 
*		 stopped, so that it doesn't interfere with other PWM pins.
*		 The striking mechanism is a MECHANICAL STRUCTURE so its force cannot be changed with code,
*		 the code is only used for releasing spring by changing position of servomotor.
*		 However, force can be fine tuned by adjusting spring in the structure of mechanism.
* Example Call: striking();
*/
void striking(void){
	timer0_init();
	pwm_servomotor_init();
	servomotor_pwm(15);
	_delay_ms(1500);
	servomotor_pwm(10);
	_delay_ms(500);
	servomotor_pwm(37);
	_delay_ms(1300);
	servomotor_pwm(15);
	_delay_ms(2000);
	timer0stop();
}

/*
* Function Name: buzzer
* Input:  x-> basically a factor by times of which the buzzer is beeped. 
*		 '1' received as x means buzzer is to be beeped for 500ms*1=500ms.
*		 Expected range of x=[1,10]
* Output: beeps buzzer for required time
* Logic: Loop generates time for which buzzer is to be beeped.
* Example Call: buzzer(2);
*/
void buzzer(unsigned int x){
	int i;							//variable used for loop
	DDRC |= (1<<buzz);
	PORTC &= ~(1<<buzz);
	for (i = 0; i<x; i++)
	{
		_delay_ms(500);
	}
	PORTC |= (1<<buzz);
}

/*
* Function Name: uart0_readByte
* Input: None
* Output: returns byte received at input 
* Logic: 16 bit (2 bytes) data is received and stored in variable rx. The bytes are separated 
*		 and status byte(which tells error) is checked and then data byte(received data) is 
*        returned to main function.  
* Example Call: uart0_readByte();
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
* Function Name: antimotion_rotate
* Input: None
* Output: Bot rotates back 180 degree to make sure striking mechanism faces towards center.
* Logic: The motors are rotated in opposite directions and rotation is stopped when bot senses that 
*		 it is back on white line.
* Example Call: antimotion_rotate();
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
* Function Name: rotate
* Input: None
* Output: Bot rotates 180 degree
* Logic: The motors are rotated in opposite directions and rotation is stopped when bot senses that
*		 it is back on white line.
* Example Call: rotate();
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
* Function Name: move
* Input: next_node-> specifies number of nodes needed to be traveled by the bot.
*		 direction-> specifies whether bot is needed to move in clockwise or anticlockwise direction. 
* Output: Bot moves to the required node.
* Logic: Values from white line sensor are regularly checked and accordingly bot is moved left or right by 
*		 changing the value of PWM signal of DC motors. Simultaneously, nodes are counted with help of readings
*		 of white line sensor.
* Example Call: move(2,1);
*/
void move(int next_node, int direction ){
	float ADC_ValueR, ADC_ValueC, ADC_ValueL,w ;
	int node_counter = 0 , total_ADC =0, flag=1;
	int outermotor_pwm, innermotor_pwm;
	
	if (direction==1)						//1=anticlockwise motion
	{
		outermotor_pwm =48;					//Calculated PWM base values to be given for circular motion.
		innermotor_pwm=180;
	}
	else
	{
		outermotor_pwm =180;
		innermotor_pwm=48;
	}
	/*the above values can be increased to 255, but its not done because when bot moves faster,
	it is not able to count nodes correctly. Also if increased to full limits then, the below 
	mentioned algorithm of changing PWM of motors will fail as values will go beyond 255 after 
	adding. 
	*/
	while(1)
	{		
		ADC_ValueR = ADC_Conversion(0);
		ADC_ValueC = ADC_Conversion(1);
		ADC_ValueL = ADC_Conversion(2);
		
		w = ADC_ValueR - ADC_ValueL;
		w*= 0.30;
		
		motors_init();
		outermotor(outermotor_pwm - (w/2));
		innermotor(innermotor_pwm + (w/2));
		forward_motion();
			
		total_ADC = ADC_ValueL + ADC_ValueC + ADC_ValueR;
		
		//'flag' is used below to ensure that a single node is not counted multiple times.
		if ( flag == 1 && total_ADC < 250)
		{
			flag = 0;								//'flag' is reset when on node. 			
			node_counter++;					
			if (node_counter == next_node)
			{	
				if (direction==1)
				{
					_delay_ms(650);					//bot is stopped after a delay to ensure a space for anti-rotation.
				}
				else
				{
					_delay_ms(200);					
				}
				motors_stop();
				break;
			}
		}
		else if( flag == 0 && total_ADC > 300)
		{
			flag = 1;								//'flag' is set when bot has passed from node and is on white line 
		}
	}
}
	
	
/*
* Function Name: main
* Input: None
* Output: The bot travels to required nodes and strikes relief aids and returns back to capital node. 
* Logic:  Reads node number received and does calculations and calls functions to reach required nodes,
*		  fires buzzer and calls for striking of coin. Finally in end when command from satellite is received
*		  to go back to capital, it takes bot back to the capital node and fires buzzer.
* Example Call: Called automatically by the Operating System 
*/	
int main(void) {
	
	int src = 1, final_des = 1;  //here src is source(the capital node in start), which 
	//							   keeps on changing as the bot traverses. It is initialized as 1
	//							   because starting capital node will always have node number 1. 
	//							   final_des is used to store the value of capital node i.e. 1,
	//							   final_des is used for return to capital after all
	//							   aids have been served.
	int des, diff, next;		 //here des is the destination node number received from
	//							   the satellite. diff is the difference between destination node 
	//							   and source node. next is the number of nodes(including destination node)
	//							   bot needs to cross to reach the destination.
	char rx_byte;				 //rx_byte is byte received from satellite  
	uart0_init(UART_BAUD_SELECT(9600, F_CPU));  //Initializing UART at baud rate 9600
	uart0_flush();								//clears input buffer
	init_devices();								//function call to initialize ADC(for white line sensor) and 
	//											  timer 2(for motors)
	
	while(1)
	{
		rx_byte = uart0_readByte();				//byte received is stored in rx_byte
		uart0_putc('0');						//regularly transmitting 0 to receiver to keep connection intact,
		//										  so that communication is not missed when satellite sends next node number. 
		//										  This will only be running when bot needs location for the next city 
		//										  to be serviced.
		
		if(rx_byte >= '1' && rx_byte <= '9')	//checking if byte received corresponds to any node number 
		{
			des= (int)rx_byte - 48 ;			//byte received is in char format so when type casted into int 
			//									  it gives ASCII value, so 48 is deducted from that value to receive
			//									  required node number. In ASCII,48 corresponds to 0 and so on till
			//									  57 which corresponds to 9.
			diff = des - src  ;					//diff is calculated as difference of destination and source node.
			
			//The below if, else if, else conditions calculate the number of nodes needed to be traveled to the city to be serviced.
			//These conditions generate a value 'next' that depicts the number of nodes to be traveled and its sign depicts whether travel 
			//should be in anticlockwise or clockwise direction according to the shortest possible. -(negative sign) means anticlockwise, and
			//+(positive sign) shows clockwise movement.
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
			
			src = des;							//the destination node number is stored in variable 'source', so that it can be used to calculate 
			//									  next city to be served.
			
			
			while(1)							
			{	
				if (next<0)						//if anticlockwise motion is shortest to go to node to be serviced, this condition is run
				{
					rotate();					//bot rotates
					move(abs(next),1);			//bot travels and reaches required node, 1 here means anticlockwise direction
					antimotion_rotate();		//bot anti rotates
					_delay_ms(50);
					buzzer(1);					//buzzer is switched on for 0.5 second (first beep)
					_delay_ms(200);
					buzzer(1);					//buzzer is switched on for 0.5 second (in total 2 beeps, which means bot is going to service the node) 
					striking();					//servomotor is started and rotated as such that it releases the loaded spring.
					_delay_ms(200);
					buzzer(2);					//buzzer is beeped for for 1 second (meaning the node has been serviced)
					break;
				}
				
				else							//if clockwise motion is shortest to go to node to be serviced, this condition is run
				{
					move(abs(next),0);			//bot travels and reaches required node, 0 here means clockwise direction
					_delay_ms(50);
					buzzer(1);
					_delay_ms(200);
					buzzer(1);
					striking();
					_delay_ms(200);
					buzzer(2);
					break;
				}				
			}			
		}
		
		else if(rx_byte == '0')						//rx_byte=0 means that all services are done and bot needs
		//											  to go back to capital node.
		{
					
			diff = final_des - src  ;				//position of bot with respect to capital is calculated.
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
				if (next<0)							//if anticlockwise motion is shortest to go to capital, this condition is run
				{
					rotate();						//bot rotates
					move(abs(next),1);				//bot travels and reaches capital node, 1 here means anticlockwise direction
					antimotion_rotate();			//bot anti rotates
					buzzer(10);						//buzzer is switched on for 5 seconds
					break;							
				}					
				else								//if clockwise motion is shortest to go to capital, this condition is run
				{
					move(abs(next),0);				//bot travels and reaches capital node, 0 here means clockwise direction
					buzzer(10);						//buzzer is switched on for 5 seconds
					break;
				}
			}
		}		
	}
	return 0;
}
