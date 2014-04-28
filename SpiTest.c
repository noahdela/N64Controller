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

#define dataLen 3  //length of data packet sent / received

//Define functions
//======================
void SPI_MasterInit(void);
char WriteByteSPI(unsigned char cData);
uint8_t *WriteToNrf(uint8_t ReadWrite, uint8_t reg, uint8_t *val, uint8_t antVal);
void nrf24L01_init(void);
//uint8_t GetReg(uint8_t reg);
//void WriteToNrf(uint8_t reg, uint8_t Package);
//void initTimer0(void);
//======================

int main (void)
{
	DDRC |= (1 << 5); //port c bit 5 set as output
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

/*****************nrf-setup**************************///Sets the nrf first by sending the register, then the value of the register.
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
}

/****************************************************************************************************************/

void nrf24L01_init(void)
{
	_delay_ms(100);	//allow radio to reach power down if shut down
	
	uint8_t val[5];	//an array of integers that sends values to WriteToNrf function
 
	//EN_AA - (auto-acknowledgments) - The transmitter will response of the receiver to-package arrived(Need only be enablad the transmitter!)
	//Requires the transmitter also has sat SAME RF_Adress on its receiver channel below example: RX_ADDR_P0 = TX_ADDR
	val[0]=0x01;//gives first integern in the array "choice" one value: 0x01 = EN_AA on pipe P0.
	WriteToNrf(W, EN_AA, val, 1);//W ='ll write / modify anything in the NRF, EN_AA = which register shall be amended, val = an array of 1 to 32 values ??to be written to the register, 1 = number of values be read out of "choice" array.
	
	//SETUP_RETR (the setup for "EN_AA")
	val[0]=0x2F;//0b0010 00011 "2" sets it up to 750uS delay between every retry (at least 500us at 250kbps and if payload >5bytes in 1Mbps, and if payload >15byte in 2Mbps) "F" is number of retries (1-15, now 15)
	WriteToNrf(W, SETUP_RETR, val, 1);
	
	//Selects which / what data pipes (0-5) that should be running.
	val[0]=0x01;
	WriteToNrf(W, EN_RXADDR, val, 1); //enable data pipe 0
 
    //RF_Adress width setup (how many bytes should RF_Adressen consist of? 1-5 bytes) 
	//(5bytes safer when there is interference but slower data transmission) 5addr-32data-5addr-32data ....	val[0]=0x03;
	WriteToNrf(W, SETUP_AW, val, 1); //0b0000 00011 motsvarar 5byte RF_Adress
 
	//RF channel setup - select the frequency from 2.400 to 2.527 GHz 1MHz/steg
	val[0]=0x01;
	WriteToNrf(W, RF_CH, val, 1); //RF channel registry 0b0000 0001 = 2.401 GHz (same on the TX RX)
 
	//RF setup - select power and transmission
	val[0]=0x07;
	//00000111 bit 3 = "0" produces lower transmission rate 1Mbps = Longer range, bit 2-1 gives power 
	//high (-0dB) ("11" = (-18dB) gives lower power = more power-efficient, but lower range
	WriteToNrf(W, RF_SETUP, val, 1);  
	
	//RX RF_Adress setup 5 bytes - choose RF_Adressen on The receiver
	// (Must be given the same RF_Adress if the transmitter has EN_AA turned on!)
	int i;
	for(i=0; i<5; i++)	
	{
		val[i]=0x12;//RF channel registry 0b10101011 x 5 
		//- writes the same RF_Adress 5 times to get an easy and safe RF_Adress (same on the transmitter chip!)
	}
	WriteToNrf(W, RX_ADDR_P0, val, 5); // 0b0010 1010 write registry 
	//- because we chose pipe 0 in "EN_RXADDR" section above, we give RF_Adressen to this pipe. 
	//(May cause various RF_Adresser to various pipes and thus listen to different transmitters)
	
	//TX RF_Adress setup 5 byte -  choose RF_Adressen on transmitter (can be commented out on a "clean" Reciver)
	//int i; //reuse previous i
	for(i=0; i<5; i++)	
	{
		val[i]=0x12;	//RF channel registry 0b10111100 x 5 - writes the same RF_Adress 5 times 
		//to get an easy and safe RF_Adress 
		//(same on the Receiver chip and the RX-RF_Adressen above if EN_AA enabled!)
	}
	WriteToNrf(W, TX_ADDR, val, 5);
 
	//Payload width setup - How many bytes will be sent by air? 1-32byte
	val[0]=dataLen;	//"0b0000 0001" = 1 byte per 5bytes RF_Adress 
	//(can choose up to "0b00100000" = 32byte/5byte RF_Adress) 
	//(defined at the top of the global variable!)
	WriteToNrf(W, RX_PW_P0, val, 1);
	
	//CONFIG reg setup - Now everything is set up, boot up nrf'en and make it either Transmitter lr Reciver
	val[0]=0x1E;  //0b0000 1110 config registry bit "1": 1 = power up, bit "0": 0 = Transmitter 
	//(bit "0": 1 = Reciver) (bit "4": 1 => mask_Max_RT, ie IRQ vector will not respond if the transmission failed.
	WriteToNrf(W, CONFIG, val, 1);
 
//device need 1.5ms to reach standby mode
	_delay_ms(100);	
 
	//sei();	
}

/**************************************************************************************************************/

//vector that is triggered when transmit_payload managed to send or 
//when receive_payload received data NOTE: when Mask_Max_rt is set in the 
//config register so it will not go off when MAX_RT is was reached on the mailing lodge nmisslyckats!

ISR(INT0_vect)
{
	cli();	//Disable global interrupt
	PORTB &= ~(1 << 1);//CE back to low
	
	PORTC |= (1 << 5);//turn LED on if status is read //led on
	_delay_ms(500);
	PORTC &= ~(1 << 5); //led off
	
	//Receiver function to print out on usart:
	//data=WriteToNrf(R, R_RX_PAYLOAD, data, dataLen);	//läs ut mottagen data
	//reset();
//
	//for (int i=0;i<dataLen;i++)
	//{
		//USART_Transmit(data[i]);
	//}
	//
	sei();
}

/***************************rest of code is not being used*************************************************/

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
//Write to a register on the nRF
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