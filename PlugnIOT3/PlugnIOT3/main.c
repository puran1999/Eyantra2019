#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
#include "atmega-adc.h"
#include <stdlib.h>
#include <stdio.h>
/*
  CONNECTIONS==============================
  Inputs:
  PC1 - CURRENT SENSOR
  PC2 - VOLTAGE SENSOR
  PC3 - EARTH LEAK TERMINAL
  Outputs:
  PB1 - BUZZER
  PB5 - STATUS LED
  PB2 - RELAY
  =========================================
*/
float price, power, voltage, current, earthLeak;
float voltageFactor = 1.38;
float currentFactor = 120.0;
float unitPrice = 100.0;

int status = 0; //  0 => normal function
//  1 => earth leak
//  2 => over current
char c = '1';
float cur, lowest, lowestMean, offset;
char buffer[10];

void measurePower() { //each call takes nearly 1 second
  lowestMean = 0;
  for (int j = 0; j < 10; j++) {
    lowest = 1023;
    cur = adc_read(ADC_PRESCALER_128, ADC_VREF_AVCC, 1);
    _delay_ms(2);
    for (int i = 0; i < 50; i++) {
      cur = adc_read(ADC_PRESCALER_128, ADC_VREF_AVCC, 1);
      _delay_ms(2);
      if (lowest > cur) {
        lowest = cur;
      }
    }
    lowestMean += lowest;
  }
  lowestMean = lowestMean / 10.0;
  lowestMean = round(lowestMean);
  current = abs(offset - lowestMean) / currentFactor;
  voltage = adc_read(ADC_PRESCALER_128, ADC_VREF_AVCC, 2) / voltageFactor;
  _delay_ms(2);
  voltage = adc_read(ADC_PRESCALER_128, ADC_VREF_AVCC, 2) / voltageFactor;
  power = current * voltage;
  price = power / unitPrice;
}

void delay_0_1_s(int t) {
  for (int i = 0; i < t; i++) {
    _delay_ms(50);
    earthLeak = adc_read(ADC_PRESCALER_128, ADC_VREF_AVCC, 3);
    _delay_ms(2);
    earthLeak = adc_read(ADC_PRESCALER_128, ADC_VREF_AVCC, 3);
    _delay_ms(50);
    if (earthLeak > 20) {
      status = 1;
    }
    else if (power > 100) {
      status = 2;
    }
    else {
      status = 0;
    }
    if (status == 0) {
      if (c == '1') {
        PORTB = 0b00000100;
      }
      else if (c == '0') {
        PORTB = 0b00000000;
      }
    }
    else {
      PORTB = 0b00100010;
    }
  }
}
void sendPowerStatus() {
  uart_string("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
  _delay_ms(1000);
  uart_string("AT+CIPSEND=67\r\n");
  _delay_ms(1000);
  uart_string("GET /update?api_key=3UCS7X8XX3O5PQCK&field1=");
  _delay_ms(100);
  dtostrf(price, 7, 6, buffer);
  uart_string(buffer);
  uart_string("&field3=");
  uart_num(status);
  uart_string("\r\n");
  _delay_ms(500);
  uart_string("AT+CIPCLOSE\r\n");
}
void getOnstate() {
  uart_string("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
  _delay_ms(1000);
  uart_string("AT+CIPSEND=70\r\n");
  _delay_ms(1000);
  uart_string("GET /channels/984326/fields/2/last.txt?api_key=1VG7RM5A9AQXSKV3\r\n");
  _delay_ms(1000);
  uart_string("AT+CIPCLOSE\r\n");
  while (c != ':') {
    c = uart_read();
  }
  c = uart_read();
}

int main(void)
{
  DDRB = 0b00100110;
  PORTB = 0b00000100;
  uart_init();

  measurePower(); //first time returns wrong power
  float offsetMean = 0;
  for (int i = 0; i < 5; i++) { // 5 seconds
    measurePower();
    offsetMean += lowestMean;
    //dtostrf(lowestMean, 6, 7 , buffer);
    //uart_string(buffer);
    //uart_string("\r\n");
  }
  offsetMean = offsetMean / 5.0;
  offsetMean = round(offsetMean);
  offset = offsetMean;

  while (1) {
    measurePower();
    sendPowerStatus();
    for (int ii = 0; ii < 12; ii++) {
      delay_0_1_s(20);
      getOnstate();
    }
  }
}