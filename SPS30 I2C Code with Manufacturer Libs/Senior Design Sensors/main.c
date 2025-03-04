/*
 * Senior Design Sensors.c
 *
 * Created: 11/21/2024 8:34:08 AM
 * Author : tye84
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>  
#include <stdint.h>
#include <string.h>
#include "sps30-i2c-3.1.1\sps30-i2c-3.1.1\sps30.h"

#define F_CPU 16000000UL
#define BAUD 9600                                   // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate for UBRR
#define HM10_response_buffer_SIZE 64

void uart0_init (void);
void uart0_transmit (unsigned char data);
int uart0_putchar(char c, FILE *stream);
void init_printf(void);
void uart1_init (void);
void uart1_transmit(unsigned char data);
void HM10_transmit(const char* str);
void HM10_init();
void HM10_print_response_buffer();
void sps30_init();

FILE uart0_output = FDEV_SETUP_STREAM(uart0_putchar, NULL, _FDEV_SETUP_WRITE);

volatile unsigned char HM10_response_buffer[HM10_response_buffer_SIZE];
volatile uint8_t HM10_response_buffer_index = 0;

struct sps30_measurement m; // Struct to store sps30 measurements

int main(void)
{
	uart0_init();  // Initialize UART
	init_printf();  // Initialize printf redirection
	uart1_init(); // Initalize UART 1 (HM10 Serial Connection)
	sei(); // Enable global interrupts
	sensirion_i2c_init(); // Must call or sensirion libs don't work. Currently empty. 
	sps30_init(); // Initialize SPS30
	HM10_init(); 

	printf("Initialization Complete!\n\n");

    int16_t SPS30_command_response_code = sps30_start_measurement();
    if (SPS30_command_response_code < 0)
        printf("error starting measurement\n");

    printf("measurements started\n");

    while (1) 
	{
        sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC); /* wait 1s */
        SPS30_command_response_code = sps30_read_measurement(&m);
		
        if (SPS30_command_response_code < 0) 
		{
            printf("error reading measurement\n");
        } 
		else 
		{			
			char SPS30_measurements[256];  
			
			// sprintf(SPS30_measurements, "measured values:\n"
			// 				"  %.2f pm1.0\n"
			// 				"  %.2f pm2.5\n"
			// 				"  %.2f pm4.0\n"
			// 				"  %.2f pm10.0\n"
			// 				"  %.2f nc0.5\n"
			// 				"  %.2f nc1.0\n"
			// 				"  %.2f nc2.5\n"
			// 				"  %.2f nc4.5\n"
			// 				"  %.2f nc10.0\n"
			// 				"  %.2f typical particle size\n",
			// 				m.mc_1p0, m.mc_2p5, m.mc_4p0, m.mc_10p0, 
			// 				m.nc_0p5, m.nc_1p0, m.nc_2p5, m.nc_4p0, 
			// 				m.nc_10p0, m.typical_particle_size);

			// Commented out code above is more concise. but formatting on DSD tech app is weird so have to send each measurement individually
			sprintf(SPS30_measurements, "measured values:\n");
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f pm1.0\n", m.mc_1p0);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f pm2.5\n", m.mc_2p5);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f pm4.0\n", m.mc_4p0);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f pm10.0\n", m.mc_10p0);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f nc0.5\n", m.nc_0p5);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f nc1.0\n", m.nc_1p0);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f nc2.5\n", m.nc_2p5);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f nc4.5\n", m.nc_4p0);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f nc10.0\n", m.nc_10p0);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f typical particle size\n", m.typical_particle_size);
			HM10_transmit(SPS30_measurements);  

			sensirion_sleep_usec(5000000);
        }
	}

    return 0;
}

void uart0_init (void)
{
	UBRR0H = (BAUDRATE>>8);        // shift the register right by 8 bits
	UBRR0L = BAUDRATE;             // set baud rate
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0);  // enable receiver and transmitter
	UCSR0C|= (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format
}

void uart0_transmit (unsigned char data) {
	while (!( UCSR0A & (1<<UDRE0))); // wait while register is free
	UDR0 = data;                     // load data in the register
}

// Redirect printf to UART
int uart0_putchar(char c, FILE *stream) {
	// Wait until the transmit buffer is empty
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;  
	return 0;  
}

// Initialize printf for UART
void init_printf(void) {
	stdout = &uart0_output;  // Link stdout to uart0_output
}

// Initialize UART1
void uart1_init (void)
{
    SPCR1 &= ~(1 << SPE); // Disable SPI
	PRR0 &= ~(1 << PRUSART1); // enable usart1 in power register

    DDRB |= (1 << DDB3);  // Set TX1 (PB3) as output
    DDRB &= ~(1 << DDB4); // Set RX1 (PB4) as input

    UBRR1H = (BAUDRATE>>8);        // shift the register right by 8 bits
	UBRR1L = BAUDRATE;             // set baud rate
	UCSR1B|= (1<<TXEN1) | (1<<RXEN1) | (1 << RXCIE1);  // enable receiver and transmitter
	UCSR1C|= (1<<UCSZ10) | (1<<UCSZ11);   // 8bit data format
	UCSR1C &= ~((1 << UMSEL11) | (1 << UMSEL10)); // Clear UMSEL11 and UMSEL10 for async mode
}

// Should use interrupt
void uart1_transmit(unsigned char data) 
{
	// Wait for buffer to be empty
    while (!(UCSR1A & (1<<UDRE1))); 
    UDR1 = data;
}

// Send TX to HM10 over UART1 
// Any TX to HM10 while not paired to a device will be considered as AT commands
// Once HM10 is paired to a device, any TX is considered plain text
// To go back into AT mode, send "AT"
void HM10_transmit(const char* str) 
{
	printf("\nSent ");
    while (*str) {
		printf("%c", *str);
        uart1_transmit(*str++);
    }
	printf(" to HM10\n");
}

// Clear HM10 buffer
void HM10_clear_response_buffer() 
{
    memset((char*)HM10_response_buffer, 0, HM10_response_buffer_SIZE);
    HM10_response_buffer_index = 0;
}

// Print the response from HM10
void HM10_print_response_buffer() 
{
	printf("HM10 Response: ");

	// Print the HM10_response_buffer, ignoring empty indexes
    for (uint16_t i = 0; i < HM10_response_buffer_SIZE; i++) {
        if (HM10_response_buffer[i] != 0x00) {  
            printf("%c", HM10_response_buffer[i]); 
        }
    }

    printf("\r\n\n"); 
}

// Initialize HM10
void HM10_init()
{
	HM10_transmit("AT");
	sensirion_sleep_usec(2000000);

	// Wait until HM10 responds with "OK"
	while(!(HM10_response_buffer[0] == 0x4F && HM10_response_buffer[1] == 0x4B))
	{
		HM10_transmit("AT");
		sensirion_sleep_usec(2000000);
	}
	HM10_print_response_buffer();
	HM10_clear_response_buffer();

	// Start broadcasting
	HM10_transmit("AT+NAMEScene_Safe");
	HM10_print_response_buffer();
	HM10_clear_response_buffer();

	HM10_transmit("AT+ROLE0");
	sensirion_sleep_usec(2000000);
	HM10_print_response_buffer();
	HM10_clear_response_buffer();

	HM10_transmit("AT+ROLE?");
	sensirion_sleep_usec(2000000);
	HM10_print_response_buffer();
	HM10_clear_response_buffer();
}

// Initialize SPS30
void sps30_init()
{
	uint8_t fw_major;
    uint8_t fw_minor;
	int16_t SPS30_command_response_code;

	while (sps30_probe() != 0) {
        printf("SPS sensor probing failed\n");
        sensirion_sleep_usec(1000000); /* wait 1s */
    }
    printf("SPS sensor probing successful\n");

    SPS30_command_response_code = sps30_read_firmware_version(&fw_major, &fw_minor);
    if (SPS30_command_response_code) {
        printf("error reading firmware version\n");
    } 
	else {
        printf("FW: %u.%u\n", fw_major, fw_minor);
    }

    char serial_number[SPS30_MAX_SERIAL_LEN];
    SPS30_command_response_code = sps30_get_serial(serial_number);
    if (SPS30_command_response_code) {
        printf("error reading serial number\n");
    } 
	else {
        printf("Serial Number: %s\n", serial_number);
    }
}

// ISR to handle USART1 rx from HM10
ISR(USART1_RX_vect) 
{
	char received_char = UDR1; 

	// Ensure buffer does not overflow
	if (HM10_response_buffer_index < HM10_response_buffer_SIZE - 1) {  
		HM10_response_buffer[HM10_response_buffer_index] = received_char;
		HM10_response_buffer_index++;
	}
	else {
		printf("Buffer full, clearing...");
		HM10_clear_response_buffer();
	}
}