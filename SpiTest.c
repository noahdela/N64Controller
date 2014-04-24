/*
    5-10-07
    Copyright Spark Fun Electronics© 2007
    Nathan Seidle
    nathan at sparkfun.com
    
    ATmega328p
	
	Example Blink
	Toggles all IO pins at 1Hz
*/

#include <avr/io.h>

//Define functions
//======================
void delay_ms(uint16_t x); //General purpose delay
void SPI_MasterInit(void);
void SPI_MasterTransmit(char cData);
//======================

int main (void)
{
    SPI_MasterInit();
	while(1)
	{
		SPI_MasterTransmit(0x0F);
		delay_ms(500);
	}
	
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
//General short delays
void delay_ms(uint16_t x)
{
  uint8_t y, z;
  for ( ; x > 0 ; x--){
    for ( y = 0 ; y < 90 ; y++){
      for ( z = 0 ; z < 6 ; z++){
        asm volatile ("nop");
      }
    }
  }
}