#ifndef _CFG_H_
#define _CFG_H_

#define ANALOG_PPM
// #define DEBUG_ANALOG_PPM

#define ALT_ANALOG_PINS

#define SERIAL_SPEED (57600)

// ############ Wiring ################

//SPI Comm.pins with nRF24L01
#define MOSI_pin  11  // MOSI - PB3
#define SCK_pin   13  // SCK  - PB5
#define CE_pin    8  // CE - PB0
#define MISO_pin  12 // MISO - PB4
#define CS_pin    7 // CS   - PD7

#define ledPin    A0 // LED  - PC0

#define BUZ_PIN 6     //buzzer on D6

// SPI outputs
#define MOSI_on PORTB |= _BV(3)  // PB3
#define MOSI_off PORTB &= ~_BV(3)// PB3
#define SCK_on PORTB |= _BV(5)   // PD5
#define SCK_off PORTB &= ~_BV(5) // PD5
#define CE_on PORTB |= _BV(0)    // PB0
#define CE_off PORTB &= ~_BV(0)  // PB0
#define CS_on PORTD |= _BV(7)    // PD7
#define CS_off PORTD &= ~_BV(7)  // PD7
// SPI input
#define  MISO_on (PINB & _BV(4)) // PB4

#ifndef ALT_ANALOG_PINS

#define THR_PIN A7
#define RUD_PIN A6 
#define ELE_PIN A3
#define AIL_PIN A4
#define BTN_PIN A5

#else

#define THR_PIN A1
#define RUD_PIN A2 
#define ELE_PIN A3
#define AIL_PIN A7
#define BTN_PIN A6

#endif

#endif /*_CFG_H_*/
