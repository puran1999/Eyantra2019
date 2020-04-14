

#ifndef _ADC_H_
#define _ADC_H_						//This is the header for AVR Microcontroller.

unsigned int adcdata,adcdata1;

 void adc_init()
 {
  ADCSRA=0X81;						//ADC enable, ADC interrupt enable, set prescaler to 2
  		
 }
 unsigned char getdata(unsigned char chno)	
  {
    ADMUX=0X60;						//right align the ADC result
    ADMUX|=chno;					//select the ADC channel
    ADCSRA|=0X40;					//start ADC convertion
    while((ADCSRA & (1 << ADIF)) == 0){}					//give some time delay to complete the aDC convertion
	return ADCH;
  }

#endif 