#ifndef _CFG_H_
#define _CFG_H_

// #define DEBUG_ANALOG_PPM

// #define ALT_ANALOG_PINS

#define SERIAL_SPEED (230400)

// ############ Wiring ################


#define PPM_pin   2  // PPM in
//SPI Comm.pins with nRF24L01

#define MOSI_pin  3  // MOSI - D3
#define SCK_pin   4  // SCK  - D4
#define CE_pin    5  // CE - D5
#define MISO_pin  A0 // MISO - A0
#define CS_pin    A1 // CS   - A1

#define ledPin    13 // LED  - D13

#define BUZ_PIN 6     //buzzer on D6

// SPI outputs
#define MOSI_on PORTD |= _BV(3)  // PB3
#define MOSI_off PORTD &= ~_BV(3)// PB3
#define SCK_on PORTD |= _BV(4)   // PD5
#define SCK_off PORTD &= ~_BV(4) // PD5
#define CE_on PORTD |= _BV(5)    // PB0
#define CE_off PORTD &= ~_BV(5)  // PB0
#define CS_on PORTC |= _BV(1)    // PD7
#define CS_off PORTC &= ~_BV(1)  // PD7
// SPI input
#define  MISO_on (PINC & _BV(0)) // PB4


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


#endif /* _CFG_H_ */
