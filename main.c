/*
 * main.c
 *
 *  Created on: May 19, 2015
 *      Author: titan
 */
#define F_CPU		8000000

//SPI master
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define ACK 0x7E
//SPI init
void SPIMasterInit(void)
{
//set MOSI, SCK and SS as output
	DDRB |= (1 << PB3) | (1 << PB5) | (1 << PB2);
//set SS to high
	PORTB |= (1 << PB2);
//enable master SPI at clock rate Fck/16
	SPCR = (1 << SPE) | (1 << MSTR);// | (1 << SPR0);
}
//master send function
uint8_t SPIMasterSend(uint8_t data)
{
//select slave
	PORTB &= ~(1 << PB2);
//send data
	SPDR = data;
//wait for transmition complete
	while (!(SPSR & (1 << SPIF)))
		;
//SS to high
	PORTB |= (1 << PB2);

	return SPDR ;
}

int main(void)
{
	//led
	DDRC |= _BV(PC1);
//initialize master SPI
	SPIMasterInit();

	uint8_t val = 0;
	uint8_t result = 0;
	while (1)
	{
		result = SPIMasterSend(val++);
		if (result == ACK)
			PORTC ^= _BV(PC1);
		_delay_ms(100);
		result = 0;
	}
}

//////////////////////////////////////////////////////////
//SPI init slave
//void SPISlaveInit(void)
//{
////set MISO as output
//	DDRB |= (1 << PB4);
////enable SPI and enable SPI interrupt
//	SPCR = (1 << SPE) | (1 << SPIE);
//}
//
//ISR(SPI_STC_vect)
//{
//	SPDR = ACK;                                  //Load data into buffer
//	while (!(SPSR & (1 << SPIF)))
//		;                  //Wait until transmission complete
//	//return(SPDR);
//}
//
//int main(void)
//{
//	SPISlaveInit();
//	sei();
//	while (1)
//	{
//
//	}
//}
//////////////////////////////////////////////////////////



