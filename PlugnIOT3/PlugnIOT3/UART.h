#ifndef _UART_H_
#define _UART_H_

void uart_init()
{
	UCSRA=0;
	UBRRH=0;
	UBRRL=103;  // 8 for 115200 error 3.7% || 103 for 9600 error 0.2%
	UCSRB=0b00011000;
	UCSRC=0b10000110;
}

void uart_char(unsigned char t)
{
	while((UCSRA & 0b00100000)==0);
	UDR=t;
}

 
void uart_string(char a[])
{
	
	int i;
	for(i=0;a[i]!='\0';i++)
	uart_char(a[i]);
	_delay_ms(100);
}

 
void uart_num(unsigned char num)
{
    unsigned char H=0,T=0,O=0;
	H=num/100;
	T=(num - (H*100))/10;
	O=(num - (H*100) - (T*10));
	
	uart_char(H+48);
	uart_char(T+48);
	uart_char(O+48);	
}
 
unsigned char uart_read()
{
	while((UCSRA & 0b10000000)==0);
	return UDR;
}

#endif