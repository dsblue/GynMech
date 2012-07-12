// tempsensor.c
// for NerdKits with ATmega168
// mrobbins@mit.edu

#define F_CPU 14745600

#include <stdio.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include "libnerdkits/delay.h"
#include "libnerdkits/lcd.h"
#include "libnerdkits/uart.h"

FILE lcd_stream;
FILE uart_stream;

void stepper_init() {

  DDRB = 0xff;  // Set Port B to Outputs
  PORTB = 0x00;

  // Must set DDR Direction on the PWM pin
  TCCR1A = 0b00000011; // OC1A Disconnected, OC1B = 
  TCCR1B = 0b00000011; // WGM=CTC, Prescale = 64

  TCNT1=0;

  OCR1A = 0x00ff;
  OCR1B = 0x00ff;
}

void syringe_on(){
  TCCR1A |= 1 << 5;
}

void syringe_off(){
  TCCR1A &= ~(1 << 5);
}

void adc_init() {
  // set analog to digital converter
  // for external reference (5v), single ended input ADC0
  ADMUX = 0;

  // set analog to digital converter
  // to be enabled, with a clock prescale of 1/128
  // so that the ADC clock runs at 115.2kHz.
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);

  // fire a conversion just to get the ADC warmed up
  ADCSRA |= (1<<ADSC);
}

uint16_t adc_read() {
  // read from ADC, waiting for conversion to finish
  // (assumes someone else asked for a conversion.)
  // wait for it to be cleared
  while(ADCSRA & (1<<ADSC)) {
    // do nothing... just hold your breath.
  }
  // bit is cleared, so we have a result.

  // read from the ADCL/ADCH registers, and combine the result
  // Note: ADCL must be read first (datasheet pp. 259)
  uint16_t result = ADCL;
  uint16_t temp = ADCH;
  result = result + (temp<<8);

  // set ADSC bit to get the *next* conversion started
  ADCSRA |= (1<<ADSC);
  
  return result;
}

enum {
  COMMAND_SET_ZERO,
  COMMAND_SET_STOP_PRESURE,
  COMMAND_INFLATE
};

enum state {
  STATE_IDLE,
  STATE_RECV,
  STATE_INFLATE
} current_state;

void poll(){
  // Main polling loop
  
  // Check for a character from the USART
  // See if there is a command comming in
  if (uart_char_is_waiting()) {
    char c = uart_read();

    // Update the command state-machine
    if (c=='1') {
      current_state = STATE_INFLATE;
      syringe_on();
    } else if (c=='0') {
      current_state = STATE_IDLE;
      syringe_off();
    }
    // Handle a new command if it exists
    // Commands = Set Presure 0-point
    //


  }
  
  // Check the current pressure
  // take 100 samples and average them!
  // holder variables for temperature data
  uint16_t last_sample = 0;
  double this_temp;
  double pres_avg = 0.0;
  uint8_t i;
  
  for(i=0; i<100; i++) {
    last_sample = adc_read();
    //this_temp = sampleTokPi(last_sample);
    this_temp = last_sample;
    
    // add this contribution to the average
    pres_avg = pres_avg + this_temp/100.0;
  }

  // write message to LCD
  lcd_home();
  lcd_write_string(PSTR("ADC: "));
  lcd_write_int16(last_sample);
  lcd_write_string(PSTR(" of 1024   "));
  lcd_line_two();
  if ( current_state == STATE_IDLE ) {
    fprintf_P(&lcd_stream, PSTR("State: Idle      "));
  } else if ( current_state == STATE_RECV ) {
    fprintf_P(&lcd_stream, PSTR("State: Recv      "));
  } else if ( current_state == STATE_INFLATE ) {
    fprintf_P(&lcd_stream, PSTR("State: Inflate   "));
  }

  // write message to serial port
  //printf_P(PSTR("%.2f degrees F\r\n"), temp_avg);

}


int main() {
  // start up the LCD
  lcd_init();
  fdev_setup_stream(&lcd_stream, lcd_putchar, 0, _FDEV_SETUP_WRITE);
  lcd_home();

  // start up the Analog to Digital Converter
  adc_init();  
  
  // configure the stepper controller
  stepper_init();
  syringe_off();
  
  // start up the serial port
  uart_init();
  fdev_setup_stream(&uart_stream, uart_putchar, uart_getchar, _FDEV_SETUP_RW);
  stdin = stdout = &uart_stream;

  // Initialize state machine
  current_state = STATE_IDLE;

  while(1) {    
    poll();
  } 

  return 0;
}
