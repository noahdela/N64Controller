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

#define dataLen 1  //length of data packet sent / received # of bytes

uint8_t *data; //random array for storage
//Define functions
//======================
void SPI_MasterInit(void);
char WriteByteSPI(unsigned char cData);
uint8_t *WriteToNrf(uint8_t ReadWrite, uint8_t reg, uint8_t *val, uint8_t antVal);
void nrf24L01_init(void);
void INT0_interrupt_init(void);
void reset(void);
void receive_payload(void);
void transmit_payload(uint8_t * W_buff);
uint8_t GetReg(uint8_t reg);
void ChangeAddress(uint8_t adress)
{
	_delay_ms(100);
	uint8_t val[5];
	int i;
	for(i=0; i<5; i++)
	{
		val[i]=adress;	//RF channel registry 0b10101011 x 5
	}
	WriteToNrf(W, RX_ADDR_P0, val, 5); //0b0010 1010 write registry
	
	//TX RF_Adress setup 5 byte
	for(i=0; i<5; i++)
	{
		val[i]=adress;	//RF channel registry 0b10111100 x 5
	}
	WriteToNrf(W, TX_ADDR, val, 5);
	_delay_ms(100);
}

void USART_interrupt_init(void)
{
	UCSR0B |= (1<<RXCIE0);
}
void usart_init(void)
{
	DDRD |= (1<<1);	//Set TXD (PD1) as output for USART
	
	unsigned int USART_BAUDRATE = 9600;		//Same as in "terminal.exe"
	unsigned int ubrr = (((F_CPU / (USART_BAUDRATE * 16UL))) - 1);	//baud prescale calculated according to F_CPU-define at top
	
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	
	/*	Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
 
	/* Set frame format: 8data, 2stop bit, The two stop-bits does not seem to make any difference in my case!?*/
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	
}
  
void USART_Transmit(uint8_t data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
}
 
uint8_t USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) );	//This loop is only needed if you not use the interrupt...
	
	/* Get and return received data from buffer */
	return UDR0; //Return the received byte
}
//void WriteToNrf(uint8_t reg, uint8_t Package);
//void initTimer0(void);
//======================

int main (void)
{
	DDRC |= (1<<5); //enable LED
	SPI_MasterInit(); //initialize SPI 
	INT0_interrupt_init();//INT0 interrupt is triggered when CE pin switches, signals data transfer to nRF
	nrf24L01_init();//initalizes the nrF module to our desired specifications(channel,power,data width,TX/RX)
	
	usart_init();
	USART_interrupt_init();
	
	/*
	USART_Transmit(GetReg(STATUS));
    USART_Transmit(GetReg(EN_AA));
	USART_Transmit(GetReg(SETUP_RETR));
	USART_Transmit(GetReg(EN_RXADDR));
	USART_Transmit(GetReg(SETUP_AW));
	USART_Transmit(GetReg(RF_CH));
	USART_Transmit(GetReg(RF_SETUP));
	USART_Transmit(GetReg(RX_ADDR_P0));
	USART_Transmit(GetReg(TX_ADDR));
	USART_Transmit(GetReg(RX_PW_P0));
	USART_Transmit(GetReg(CONFIG));
	*/
	
	/*uint8_t W_buffer[5];
	int i;
	for (i=0;i<5;i++)
	{
		W_buffer[i]=0x93;
	}
	for (i=0;i<5;i++)
	{
		USART_Transmit(W_buffer[i]);
	}
	*/
	
	reset();
	
	while(1)
	{
		//let interrupts do all the work**************************************
	}
	
	/* if nRF is in receiver mode
	while(1)
	{
        reset();
        receive_payload();
	}
	*/

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

void INT0_interrupt_init(void)	
{
	DDRD &= ~(1 << DDD2);	//Set PD2 as input (INT0 pin)
	
	EICRA |=  (1 << ISC01);// INT0 falling edge generates request	PD2
	EICRA  &=  ~(1 << ISC00);// same as above
 
	EIMSK |=  (1 << INT0);	//enable int0 external interrupt request
  	sei();// enables global interrupts
} 

/*****************nrf-setup**************************///Sets the nrf first by sending the register, then the value of the register.
uint8_t *WriteToNrf(uint8_t ReadWrite, uint8_t reg, uint8_t *val, uint8_t antVal)	//takes in "ReadWrite" (W or R), "reg" (ett register), "*val" (an array) & "antVal" (number of values in val)
{
	cli();	//disable global interrupts so no interrupts can interfere with the write process
	
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
	
	sei(); //re-enable global interrupts
	
	return ret;	//returns an array
}

/****************************************************************************************************************/

void nrf24L01_init(void)
{
	_delay_ms(100);	//allow radio to reach power down if shut down
	
	uint8_t val[5];	//an array of integers to send to WriteToNrf function
	
	//EN_AA - (enable auto-acknowledgements) - transmitter gets automatic response from reciever on successful transmit
	//only works if transmitter has identical rf address on its channel ex: RX_ADDR_P0 = TX_ADDR
	val[0]=0x01;//value: 0x01 = EN_AA on pipe 0.
	WriteToNrf(W, EN_AA, val, 1);//W=write mode, EN_AA=register to write to, val=data to write, 1=number of bytes
	
	//SETUP_RETR (setup of automatic retransmission)
	val[0]=0x2F;//2 = 750us between retransmissions
	//(at least 500us at 250kbps and if payload >5bytes in 1Mbps, and if payload >15byte in 2Mbps)
    // "F" is number of retries (1-15, F=15 retries = max)
	WriteToNrf(W, SETUP_RETR, val, 1);
	
	//Selects data pipes (0-5) that should be running.
	val[0]=0x01; //enable data pipe 0
	WriteToNrf(W, EN_RXADDR, val, 1); 
	
    //RF_Address width setup (3-5 bytes, "11" = 5 bytes)
	val[0]=0x03;//0000 0011
	WriteToNrf(W, SETUP_AW, val, 1);
 
	//RF channel setup - choose freq 2.400-2.527GHz 1MHz/step
	val[0]=0x01;
	WriteToNrf(W, RF_CH, val, 1); //RF channel registry 0b0000 0001 = 2.401 GHz (same on the TX RX)
 
	//RF setup	- choose signal power and data speed 
	val[0]=0x07; //0000 0111
	//bit 0 is don't care
	//bit 2-1 gives power ("11" = 0dBm; "00" = -18dBm, can also choose -6dBm and -12dBm)
	//bit 3 and 5 give data speed, 00=1Mbps, 01=2Mbps, 10=250kbps
	WriteToNrf(W, RF_SETUP, val, 1);//0dBm (max power) and 1Mbps (middle data speed)
	
	//RX RF_Address setup 5 bytes - choose RF_Address on The receiver
	// (Must be given the same RF_Address if the transmitter has EN_AA turned on!)
	int i;
	for(i=0; i<5; i++)	
	{
		val[i]=0x12;//RF channel registry 0b 0001 0010 x 5 
		//- writes the same RF_Address 5 times to get an easy and safe RF_Address (same on the transmitter chip!)
	}
	WriteToNrf(W, RX_ADDR_P0, val, 5); //write address created in val as RX_address 
	//since we chose pipe 0, we give RF_Address to this pipe. 
	//here you can give different addresses to different pipes to listen to several different transmitters
	
	//TX RF_Address setup 5 byte-choose RF_Address on transmitter
	//int i; //reuse previous i
	for(i=0; i<5; i++)	
	{
		val[i]=0x12; //RF channel registry 0b 0001 0010 x 5 - writes the same RF_Address 5 times 
		//to get an easy and safe RF_Adress 
		//(same on the Receiver chip and the RX-RF_Address above if EN_AA enabled!)
	}
	WriteToNrf(W, TX_ADDR, val, 5);//write same address as TX_Address since EN_AA is enabled
 
	//Payload width setup - How many bytes will be sent by air? 1-32byte per transmission
	val[0]=dataLen;	//dataLen is defined earlier (currently 5 (bytes) )
	//same on transmitter and receiver
	WriteToNrf(W, RX_PW_P0, val, 1);//RX Payload Width on Pipe 0 = RX_PW_P0
	
	//CONFIG reg setup - Now everything is set up, boot up nrf and make it either Transmitter or Reciver
	val[0]=0x1E;  //0b 0001 1110 config registry 
	//bit 0=0:transmitter,bit 0=1:receiver,
	//bit 1=1 power up,bit 4=1: mask_Max_RT i.e. IRQ interrupt isn't triggered if transmission failed
	//bits 2-3 CRC encoding scheme(?)
	WriteToNrf(W, CONFIG, val, 1);//transmitter,power up,CRC enabled 2byte encoding,maskMaxRT 
 
    //device need 1.5ms to reach standby mode
	_delay_ms(100);	
 
	sei();	
}

/**************************************************************************************************************/
//receive data
void receive_payload(void)
{
	sei();		//Enable global interrupt
	
	PORTB |= (1 << 1);	//CE goes high, "listening"
	_delay_ms(1000);	//Listen for 1 second, interrupt int0 executes
	PORTB &= ~(1 << 1); //CE goes low, "stop listening"
	
	cli();	//Disable global interrupt
}
 
//Send data
void transmit_payload(uint8_t * W_buff)
{
	USART_Transmit('a');
	WriteToNrf(R, FLUSH_TX, W_buff, 0);//send 0xE1 which flushes the registry of old data 
	USART_Transmit('b');
	WriteToNrf(R, W_TX_PAYLOAD, W_buff, dataLen);//send data in W_buff to nrf
	USART_Transmit('c');
	
	sei();	//enable global interrupts
 
	_delay_ms(10);		//necessary delay
	PORTB |= (1 << 1);	//CE high, send data, int0 interrupt
	_delay_us(20);		//at least 10us needed
	PORTB &= ~(1 << 1);//CE low
	_delay_ms(10);		//necessary delay
 
	cli();	//Disable global interrupt.. 
	USART_Transmit('d');
}
/**************************************************************************************************************/

void reset(void)//called after a successful data transmission
{
	_delay_us(10);
	PORTB &= ~(1 << 2);//CSN low
	_delay_us(10);
	WriteByteSPI(W_REGISTER + STATUS);	//write to status registry
	_delay_us(10);
	WriteByteSPI(0x70);	//reset all irq in status registry
	_delay_us(10);
	PORTB |= (1 << 2);	//CSN back to high, nRF doing nothing
}

/***************************************************************************************************************/
//When you use interrupt, it is crucial that you enables the external interrupts by the command sei();
// This should be done before the receive_payload, and transmit_payload is used.

ISR(INT0_vect)//this interrupt is triggerred on successful transmission
{
	USART_Transmit('i');
	cli();	//Disable global interrupt
	PORTB &= ~(1 << 1);//CE back to low-"stop listening/transmitting"
	
	reset();
	
	/* if recieve mode...
	data = WriteToNrf(R, R_RX_PAYLOAD, data, 5);//store recieved message
	reset();//reset chip for further communication
	
	int i;
	for (i=0;i<dataLen;i++)
	{
		USART_Transmit(data[i]);//send recieved data to computer via usart
	}
	*/
	
	PORTC |= (1<<5); //blink LED to show success
	_delay_ms(500);
	PORTC &= ~(1<<5);
	
	sei();
}

/****************************************************************************/

ISR(USART_RX_vect)	///Vector that triggers when computer sends something to the Atmega328
{
	uint8_t W_buffer[dataLen];	//Creates a buffer to receive data with specified length (ex. dataLen = 5 bytes)
	
	int i;
	for (i=0;i<dataLen;i++)
	{
		W_buffer[i]=USART_Receive();	//receive the USART
		USART_Transmit(W_buffer[i]);	//Transmit the Data back to the computer to make sure it was correctly received
		//This probably should wait until all the bytes is received, but works fine in to send and receive at the same time... =)
	}

	if (W_buffer[0] == 's')
	{
		USART_Transmit(GetReg(STATUS));
	}
	if (W_buffer[0] == 'f')
	{
		USART_Transmit(GetReg(FIFO_STATUS));
	}
	
	if (W_buffer[0]=='9')
	{
		ChangeAddress(0x13);
		transmit_payload(W_buffer);
		ChangeAddress(0x12);
	} 
	else
	{
		transmit_payload(W_buffer);
	}

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

/***********************************rest of code is not being used****************************************
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
*/