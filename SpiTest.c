/*
    4-24-2014
    N64 Controller Project
    ATmega328p
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#define F_CPU 16000000UL  // 16 MHz
#include <util/delay.h>

#include "nRF24L01.h"

#define dataLen 32  //length of data packet sent / received

//Define functions
//======================
void SPI_MasterInit(void);
char WriteByteSPI(unsigned char cData);
uint8_t GetReg(uint8_t reg);
void WriteToNrf(uint8_t reg, uint8_t Package);
//uint8_t *WriteToNrf(uint8_t ReadWrite, uint8_t reg, uint8_t *val, uint8_t antVal);
void initTimer0(void);
//======================

int main (void)
{
	//Test code for testing spi output and timer0
	DDRC = 0b11111111; //All port c set as output
	//initTimer0();
	
    SPI_MasterInit();
	while(1)
	{
		if(GetReg(STATUS)==0x0E)
		{
			//PORTC |= (1 << 5);//turn LED on if status is read
		}
	}

    return(0);
}

void SPI_MasterInit(void)
{
	/*Set CE, MOSI, SCK, and SS/CSN as output */
	DDRB |= (1 << 1)|(1 << 2)|(1 << 3)|(1 << 5);
	DDRB &= ~(1 << 4);//set MISO as input
	/*Set as Master, set clock rate fck/128, SpiEnable */
	SPCR |= (1 << MSTR);
	SPCR |= (1 << SPR0)|(1 << SPR1);
	SPCR |= (1 << SPE);
	
	PORTB |= (1 << 2);//CSN high to begin with, no communication with NRF yet
	PORTB &= ~(1 << 1);//CE low to begin with since nothing being sent/received
}

//send or recieve a byte from the nRF
char WriteByteSPI(unsigned char cData)
{
	/*Start Transmission*/
	SPDR = cData;
	/*Wait for transmission*/
	while(!(SPSR & (1 << SPIF)))
		;
	return SPDR;//return what's recieved to the nRF
}

//Read a register on the nRF
uint8_t GetReg(uint8_t reg)
{	
	//begin with a delay for timing purposes
	_delay_us(10);	
	PORTB &= ~(1 << 2);//CSN low
	_delay_us(10);
	WriteByteSPI(R_REGISTER + reg);	//R_Register = set nRF to reading mode, "reg" will be read back
	_delay_us(10);
	reg = WriteByteSPI(NOP);//Send dummy byte to recieve back first byte in reg register
	_delay_us(10);
	PORTB |= (1 << 2);	//CSN back to high, nRF doing nothing
	return reg;	// Return the read registry
}

void WriteToNrf(uint8_t reg, uint8_t Package)
{
	//begin with a delay for timing purposes
	_delay_us(10);	
	PORTB &= ~(1 << 2);//CSN low
	_delay_us(10);
	WriteByteSPI(W_REGISTER + reg);	//R_Register = set nRF to reading mode, "reg" will be read back
	_delay_us(10);
	reg = WriteByteSPI(Package);//Send package to be written to "reg"
	_delay_us(10);
	PORTB |= (1 << 2);	//CSN back to high, nRF doing nothing
}

/*****************nrf-setup************************** //Sets the nrf first by sending the register, then the value of the register.
uint8_t *WriteToNrf(uint8_t ReadWrite, uint8_t reg, uint8_t *val, uint8_t antVal)	//takes in "ReadWrite" (W or R), "reg" (ett register), "*val" (an array) & "antVal" (number of values in val)
{
	cli();	//disable global interrupts
	
	if (ReadWrite == W)	//W = write to nrf (R= reads it, R_REGISTER (0x00)
	{
		reg = W_REGISTER + reg;	//ex: reg = EN_AA: 0b0010 0000 + 0b0000 0001 = 0b0010 0001  
	}
	
	//Static uint8_t returns an array (note the "*")
	static uint8_t ret[dataLen];	//assume that the longest you want it to read when it calls the "R" is n(some amount) - far, and that it uses only 1 byte times n. You want to read out 5 bytes RF_Address so the write 5 is here!!	
	
	_delay_us(10);		//the proper delay for the NRF! (microseconds)
	PORTB &= ~(1 << 2);;	//CSN low = nrf chip starts listening
	_delay_us(10);		
	WriteByteSPI(reg);	//set the nRF to write or read mode of "reg"
	_delay_us(10); 		
	
	int i;
	for(i=0; i<antVal; i++)
	{
		if (ReadWrite == R && reg != W_TX_PAYLOAD)//want to read a registry?
		{
			ret[i]=WriteByteSPI(NOP);//send dummy bytes to read out the data
			_delay_us(10);			
		}
		else 
		{
			WriteByteSPI(val[i]);//send commands to the nRF one at a time
			_delay_us(10);
		}		
	}
	PORTB |= (1 << 2);	//CSN back to high - nrf chip stops listening
	
	sei(); //enable global interrupts
	
	return ret;	//returns an array
}*/

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


//global var
int alt = 0;//dumb debug var
ISR (TIMER1_COMPA_vect)
{
    // action to be done every .25s sec
	//blink the LED every .5 seconds (.25 on, .25 off)
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