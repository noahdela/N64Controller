/*
    4-24-2014
    N64 Controller Project
    ATmega328p
*/

#include <avr/io.h>
#include <avr/interrupt.h>

//Define functions
//======================
void SPI_MasterInit(void);
void SPI_MasterTransmit(char cData);
void initTimer0(void);
//======================

//global var
int alt = 0;

int main (void)
{
	initTimer0();
    /*SPI_MasterInit();
	while(1)
	{
		SPI_MasterTransmit(0x0F);
	}*/
	while(1) {}//sit and loop while testing interrupt
	
    return(0);
}

void SPI_MasterInit(void)
{
	/*Set MOSI, SCK, and SS as output */
	DDRB |= (1 << 2)|(1 << 3)|(1 << 5);
	DDRB &= ~(1 << 4);
	/*Enable SPI , Master, set clock rate fck/128*/
	SPCR |= (1 << MSTR);
	SPCR |= (1 << SPR0)|(1 << SPR1);
	SPCR |= (1 << SPE);
}
void SPI_MasterTransmit(char cData)
{
	/*Start Transmission*/
	SPDR = cData;
	/*Wait for transmission*/
	while(!(SPSR & (1 << SPIF)))
		;
}

// this code sets up timer1 for a .25s interrupt @ 16Mhz Clock (mode 4)
void initTimer0(void)
{
    OCR1A = 0xF42;//3906 in hex = .25s

    TCCR1B |= (1 << WGM12);
    // Mode 4, CTC on OCR1A

    TIMSK1 |= (1 << OCIE1A);
    //Set interrupt on compare match

    TCCR1B |= (1 << CS12) | (1 << CS10);
    // set prescaler to 1024 and start the timer (64 us resolution)


    sei();
    // enable interrupts
}

ISR (TIMER1_COMPA_vect)
{
    // action to be done every .25s sec
	//blink the LED again
	if(alt == 0)
	{
		PORTC = 0xFF;//all port C high
		alt = 1;
	}
	else if(alt == 1)
	{
		PORTC = 0x00;//all port c low
		alt = 0;
	}
	
}