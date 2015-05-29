/*
 * main.c
 *
 *  Created on: May 19, 2015
 *      Author: titan
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define MASTER
//#define SLAVE

#define ACK 0x7E

#ifdef MASTER

//SPI init
void SPIMasterInit(void) {
//set MOSI, SCK and SS as output
	DDRB |= (1 << PB3) | (1 << PB5) | (1 << PB2);
//set SS to high
	PORTB |= (1 << PB2);
//enable master and clock fosc/128
	SPCR = (1 << SPE) | (1 << MSTR) | (1<< SPR1) | (1 << SPR0);
}

uint8_t SPIMasterSend(uint8_t data) {
	//send data
	SPDR = data;

	//wait for transmition complete
	while (!(SPSR & (1 << SPIF)))
	;

	return SPDR;
}

int main(void) {
	//led
	DDRC |= _BV(PC1) | _BV(PC2);

	SPIMasterInit();

	uint8_t result = 0;
	while (1) {

		PORTB &= ~(1 << PB2);			// SS low
		result = SPIMasterSend(0x82);// WRITE frame (register 0x02)
		result = SPIMasterSend(0xAB);// dummy send to read the data
		result = SPIMasterSend(0xCD);// dummy send to read the data
		result = SPIMasterSend(0xEF);// dummy send to read the data
		PORTB |= (1 << PB2);// SS high


		PORTB &= ~(1 << PB2);			// SS low
		result = SPIMasterSend(0x01);// READ error_num (register 0x01) and frame[0..2]
		result = SPIMasterSend(0xFF);// dummy send to read the data
		result = SPIMasterSend(0xFF);// dummy send to read the data
		result = SPIMasterSend(0xFF);// dummy send to read the data
		result = SPIMasterSend(0xFF);// dummy send to read the data
		PORTB |= (1 << PB2);// SS high

		if (result == 0xEF) {
			PORTC ^= _BV(PC1);
			PORTC &= ~(_BV(PC2));
		} else {
			PORTC &= ~(_BV(PC1));
			PORTC |= _BV(PC2);
		}

		_delay_ms(500);
		result = 0;
	}
}

//int main(void) {
//	//led
//	DDRC |= _BV(PC1);
//
//	// ss
//	DDRB |= _BV(PB2);
//
//	while (1) {
//		PORTB ^= _BV(PB2);
//		PORTC ^= _BV(PC1);
//		_delay_ms(100);
//	}
//}

#endif

#ifdef SLAVE

// MODE not be confused with the AVR SPI MODE, which is MODE 0
#define SPI_SS_PIN_MASK			(1 << PB2)
#define SPI_MODE_READING    	0x00
#define SPI_MODE_WRITING	 	0x80
#define SPI_MODE_MASK			0x80
#define SPI_REGISTER_MASK		0x7F
#define SPI_STATE_IDLE			SPI_SS_PIN_MASK
#define SPI_STATE_ACTIVE	 	0x00
#define SPI_SOF_TRUE 			0xff
#define SPI_SOF_FALSE			0x00

volatile uint8_t spi_state;
uint8_t spi_mode;
uint8_t spi_sof;
void * spi_register;

typedef struct {
	uint8_t status;				// Register 0x00
	uint8_t error_num;				// Register 0x01
	uint8_t frame[20];				// Register 0x02
} REGISTRY_t;
REGISTRY_t registry;

void * registry_index[] = { &registry.status, &registry.error_num, &registry.frame[0] };

void SPISlaveInit(void) {
	//set MISO as output
	DDRB |= (1 << PB4);

	// int for SS for spi_active flag
	PCICR |= (1 << PCIE0);
	PCMSK0 |= (1 << PCINT2);

	// initial data/control vars
	spi_state = SPI_STATE_IDLE;
	spi_sof = SPI_SOF_FALSE;
	SPDR = 0x00;

	//enable SPI and enable SPI interrupt
	SPCR = (1 << SPE) | (1 << SPIE);
}

ISR(PCINT0_vect) {
	// DEFINITELY TRIGGERING ~5us AFTER SS TRANISITION (before STC_vect)

	spi_state = (PINB & SPI_SS_PIN_MASK);
	if (spi_state == SPI_STATE_ACTIVE) {
		spi_sof = SPI_SOF_TRUE;
	}
}

ISR(SPI_STC_vect) {
	// DEFINITELY TRIGGERING AFTER EACH BYTE RECEIVED

	uint8_t data = SPDR;

	if (spi_sof == SPI_SOF_TRUE) {
		PORTC ^= _BV(PC1);
		spi_mode = (data & SPI_MODE_MASK);
		spi_register = registry_index[data & SPI_REGISTER_MASK];
	}

	if (spi_mode == SPI_MODE_READING) {
		SPDR = (*(uint8_t*) spi_register++);
	} else if (spi_sof == SPI_SOF_FALSE){
		*((uint8_t*) spi_register++) = data;
	}

	spi_sof = SPI_SOF_FALSE;
}

// Monitor SS to set state to idle on transition to HIGH

int main(void) {

	// test data
	registry.status = 0xAB;
	registry.error_num = 0x03;
	registry.frame[0] = 0x01;
	registry.frame[1] = 0x12;
	registry.frame[2] = 0x05;
	registry.frame[3] = 0x07;
	registry.frame[4] = 0x09;
	registry.frame[5] = 0x00;

	DDRC |= _BV(PC1);
	//PORTC &= ~(_BV(PC1)); // turn off the pullup

	SPISlaveInit();
	sei();

	while (1) {
	}
}

#endif

