/*
 * sps30_baremetal_i2c.c
 *
 * Created: 1/31/2025 11:29:29 AM
 * Author : tye84
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>  // uart_transmit
#include <stdint.h>

#define F_CPU 16000000UL
#define BAUD 9600                                   // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate for UBRR
#define SPS30_ADDR 0x69

#define FOSC 16000000UL // Clock frequency = Oscillator freq .
#define BDIV ( FOSC / 100000 - 16) / 2 + 1

#define SPS30_RESET 0xD304


void uart_init (void);
void uart_transmit (unsigned char data);
int uart_putchar(char c, FILE *stream);
void init_printf(void);
uint8_t i2c_io(uint8_t device_addr, uint8_t *wp, uint16_t wn, uint8_t *rp, uint16_t rn);
uint8_t CalcCrc(uint8_t data[2]);


FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

int main(void)
{
	uart_init();  // Initialize UART
	init_printf();  // Initialize printf redirection
	
	i2c_init(BDIV);
	
	unsigned char status;
	unsigned char buf [2] = {0xD3, 0x04}; // fill this buffer with the command you wish to send 
	unsigned char rbuf [60];
	uint8_t count = 1;
	
    while (1) 
    {
		status = i2c_io (SPS30_ADDR, buf, 2, NULL, 0);
		printf("Reset Write Status = %d\n", status);
		_delay_ms(2500);
		
		// buf[0] = 0xD1;
		// buf[1] = 0x00;
		// status = i2c_io (SPS30_ADDR, buf , 2, rbuf, 3);
		// printf("Read Status = %d\n", status);
		// printf("Firmware Version: ");
		// for (int i = 0; i < 1; i++)
		// {
		// 	printf("%d ");
		// }
		// printf("\n");

		buf[0] = 0x00;
		buf[1] = 0x10; 
		buf[2] = 0x03; // format for measurements, here we chose float
		buf[3] = 0x00; // dummy byte
		buf[4] = CalcCrc(&buf[2]);
		status = i2c_io (SPS30_ADDR, buf, 2, NULL, 0);
		printf("Start Measurement Write Status = %d\n", status);
		_delay_ms(20);

		buf[0] = 0x03;
		buf[1] = 0x00;
 		status = i2c_io(SPS30_ADDR, buf, 2, rbuf, 60);
		printf("Read Measurement Status = %d\n", status);
		for(int i = 0; i < 60; i++){
			if(count != 3){
				printf("")
			}
		}
		_delay_ms(20);
    }
}

/*
  i2c_init - Initialize the I2C port
*/
void i2c_init(uint8_t bdiv)
{
    TWSR0 = 0;                           // Set prescalar for 1
    TWBR0 = bdiv;                        // Set bit rate register
}

uint8_t i2c_io(uint8_t device_addr, uint8_t *wp, uint16_t wn, uint8_t *rp, uint16_t rn)
{
	uint8_t status, send_stop, wrote, start_stat;

	status = 0;
	wrote = 0;
	send_stop = 0;

	if (wn > 0) {
		wrote = 1;
		send_stop = 1;

		TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);  // Send start condition
		while (!(TWCR0 & (1 << TWINT)));     // Wait for TWINT to be set
		status = TWSR0 & 0xf8;
		if (status != 0x08)                 // Check that START was sent OK
		{
			printf("Status != 0x08\n");
			return(status);
		}

		//TWDR0 = device_addr & 0xfe;          // Load device address and R/W = 0;
		TWDR0 = device_addr << 1;
		// 01101001 & 11111110 = 01101000
		// 01101001 << 1 = 11010010
		TWCR0 = (1 << TWINT) | (1 << TWEN);  // Start transmission
		while (!(TWCR0 & (1 << TWINT)));     // Wait for TWINT to be set
		status = TWSR0 & 0xf8;
		if (status != 0x18) {               // Check that SLA+W was sent OK
			if (status == 0x20)             // Check for NAK
			{
				printf("Going to nakstop\n");
				goto nakstop;
			}               // Send STOP condition
			return(status);                 // Otherwise just return the status
		}

		// Write "wn" data bytes to the slave device
		while (wn-- > 0) {
			TWDR0 = *wp++;                   // Put next data byte in TWDR
			TWCR0 = (1 << TWINT) | (1 << TWEN); // Start transmission
			while (!(TWCR0 & (1 << TWINT))); // Wait for TWINT to be set
			status = TWSR0 & 0xf8;
			if (status != 0x28) {           // Check that data was sent OK
				if (status == 0x30)         // Check for NAK
				goto nakstop;           // Send STOP condition
				return(status);             // Otherwise just return the status
			}
		}

		status = 0;                         // Set status value to successful
	}

	if (rn > 0) {
		send_stop = 1;

		// Set the status value to check for depending on whether this is a
		// START or repeated START
		start_stat = (wrote) ? 0x10 : 0x08;

		// Put TWI into Master Receive mode by sending a START, which could
		// be a repeated START condition if we just finished writing.
		TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
		// Send start (or repeated start) condition
		while (!(TWCR0 & (1 << TWINT)));     // Wait for TWINT to be set
		status = TWSR0 & 0xf8;
		if (status != start_stat)           // Check that START or repeated START sent OK
		return(status);

		//TWDR0 = device_addr  | 0x01;         // Load device address and R/W = 1;
		TWDR0 = (device_addr << 1) | 0x01;
		
		TWCR0 = (1 << TWINT) | (1 << TWEN);  // Send address+r
		while (!(TWCR0 & (1 << TWINT)));     // Wait for TWINT to be set
		status = TWSR0 & 0xf8;
		if (status != 0x40) {               // Check that SLA+R was sent OK
			if (status == 0x48)             // Check for NAK
			goto nakstop;
			return(status);
		}

		// Read all but the last of n bytes from the slave device in this loop
		rn--;
		while (rn-- > 0) {
			TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // Read byte and send ACK
			while (!(TWCR0 & (1 << TWINT))); // Wait for TWINT to be set
			status = TWSR0 & 0xf8;
			if (status != 0x50)             // Check that data received OK
			return(status);
			*rp++ = TWDR0;                   // Read the data
		}

		// Read the last byte
		TWCR0 = (1 << TWINT) | (1 << TWEN);  // Read last byte with NOT ACK sent
		while (!(TWCR0 & (1 << TWINT)));     // Wait for TWINT to be set
		status = TWSR0 & 0xf8;
		if (status != 0x58)                 // Check that data received OK
		return(status);
		*rp++ = TWDR0;                       // Read the data

		status = 0;                         // Set status value to successful
	}
	
	nakstop:                                    // Come here to send STOP after a NAK
	if (send_stop)
		TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);  // Send STOP condition
	
	return(status);
}

void uart_init (void)
{
	UBRR0H = (BAUDRATE>>8);        // shift the register right by 8 bits
	UBRR0L = BAUDRATE;             // set baud rate
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0);  // enable receiver and transmitter
	UCSR0C|= (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format
}
void uart_transmit (unsigned char data) {
	while (!( UCSR0A & (1<<UDRE0))); // wait while register is free
	UDR0 = data;                     // load data in the register
}
// Redirect printf to UART
int uart_putchar(char c, FILE *stream) {
	while (!(UCSR0A & (1 << UDRE0))) {
		;  // Wait until the transmit buffer is empty
	}
	UDR0 = c;  // Send the character
	return 0;  // Return 0 to indicate success
}
// Initialize printf for UART
void init_printf(void) {
	stdout = &uart_output;  // Link stdout to uart_output
}

uint8_t CalcCrc(uint8_t data[2]) {
	uint8_t crc = 0xFF;
	for(int i = 0; i < 2; i++) {
		crc ^= data[i];
		for(uint8_t bit = 8; bit > 0; --bit) {
			if(crc & 0x80) {
				crc = (crc << 1) ^ 0x31u;
			} 
			else {
				crc = (crc << 1);
			}
		}
	}
	return crc;
}
