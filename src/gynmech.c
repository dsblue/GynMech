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

// PC1 = Pump Direction
enum { 
  PUMP_DIR_PIN = 1 
};

void stepper_init() {

  DDRB = 0xff;  // Set Port B to Outputs
  PORTB = 0x00;

  DDRC  |=  (1U << PUMP_DIR_PIN);  // Set PIN PC1 to Output GPIO (Motor Direction)
  PORTC &= ~(1U << PUMP_DIR_PIN);

  // Must set DDR Direction on the PWM pin
  TCCR1A = 0b00000011; // OC1A Disconnected, OC1B = 
  TCCR1B = 0b00000011; // WGM=CTC, Prescale = 64

  TCNT1=0;

  OCR1A = 0x00ff;
  OCR1B = 0x00ff;
}

void syringe_inflate(){  
  PORTC &= ~(1U << PUMP_DIR_PIN);
  TCCR1A |= 1 << 5;
}

void syringe_deflate(){  
  PORTC |= (1U << PUMP_DIR_PIN);
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

enum state {
  STATE_IDLE,
  STATE_RECV,
  STATE_INFLATE,
  STATE_DEFLATE
} current_state;

enum {
  COMMAND_NONE,
  COMMAND_SET_ZERO,
  COMMAND_SET_PRESSURE,
  COMMAND_INFLATE,
  COMMAND_DEFLATE
} current_command;

#define MAX_VALUE_LENGTH         8

enum commandState {
  CMD_STATE_COMMAND,
  CMD_STATE_EQUAL,
  CMD_STATE_VALUE  
} current_command_state;

char value_str[MAX_VALUE_LENGTH];
int value_str_i;

int target_pressure;

void poll(){
  // Main polling loop
  
  // Check for a character from the USART
  // See if there is a command comming in
  if (uart_char_is_waiting()) {
    char c = uart_read();
    
    // Update the command state-machine
    if (c == '\n' || c == '\r' ) {
      // End of a command, process it and reset the command string
      if (current_command == COMMAND_NONE) {
      } else if (current_command == COMMAND_INFLATE) {
	syringe_inflate();
	current_state = STATE_INFLATE;
      } else if (current_command == COMMAND_DEFLATE) {
	syringe_deflate();
	current_state = STATE_DEFLATE;
      } else if (current_command == COMMAND_SET_PRESSURE) {
	int ret = sscanf_P(value_str, PSTR("%d"), &target_pressure);
	//	printf_P(PSTR("1 = %d\r\n"), ret );  
	//	printf_P(PSTR("2 = %d\r\n"), target_pressure );
      }
      value_str_i = 0;
      current_command = COMMAND_NONE;
      current_command_state = CMD_STATE_COMMAND;
    } else {
      // In a command string
      if (current_command_state == CMD_STATE_COMMAND) {
	// Command character
	if (c == 'i') {
	  // Inflate
	  current_command = COMMAND_INFLATE;
	} else if (c == 'd') {
	  // Deflate
	  current_command = COMMAND_DEFLATE;
	} else if (c == 'p') {
	  // Set pressure point
	  current_command = COMMAND_SET_PRESSURE;
	} else {
	  current_command = COMMAND_NONE;
	}
	current_command_state = CMD_STATE_EQUAL;
      } else if (current_command_state == CMD_STATE_EQUAL) {
	// Assume the second char is always '='
	current_command_state = CMD_STATE_VALUE;
      } else if (current_command_state == CMD_STATE_VALUE) {
	// In value part of command
	value_str[value_str_i++] = c;
	if (value_str_i >= MAX_VALUE_LENGTH) {
	  value_str_i = MAX_VALUE_LENGTH-1;
	}
	value_str[value_str_i] = 0; // Write NULL terminator
      }
    }
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

  // Check to see if we hit the target presure
  if ( current_state == STATE_INFLATE ) {
    if ( pres_avg >= target_pressure ) {
      // Stop Inflating
      syringe_off();
      current_state = STATE_IDLE;
    }
  } else if ( current_state == STATE_DEFLATE ) {
    if ( pres_avg <= target_pressure ) {
      // Stop Deflating
      syringe_off();
      current_state = STATE_IDLE;      
    }
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
  } else if ( current_state == STATE_DEFLATE ) {
    fprintf_P(&lcd_stream, PSTR("State: Deflate   "));
  }
  lcd_line_three();
  fprintf_P(&lcd_stream, PSTR("Target Pres: %d"), target_pressure );
  // write message to serial port
  //printf_P(PSTR("%.2f degrees F\r\n"), temp_avg);
  //printf_P(PSTR("Pres = %d\r\n"), target_pressure );
  //printf_P(PSTR("Value = %s\r\n"), value_str );

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
  current_command = COMMAND_NONE;
  current_command_state = CMD_STATE_COMMAND;
  value_str_i = 0;
  target_pressure = 512;

  while(1) {    
    poll();
  } 

  return 0;
}
