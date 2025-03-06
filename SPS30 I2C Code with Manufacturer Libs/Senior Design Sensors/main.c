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
#include <util/delay.h>

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
void ADC_init(void);
uint16_t ADC_Read();
float convert_ADC_to_pressure(uint16_t adc_value);

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
	ADC_init();
	HM10_init(); 

	printf("Initialization Complete!\n\n");

    int16_t SPS30_command_response_code = sps30_start_measurement();
    if (SPS30_command_response_code < 0)
        printf("error starting measurement\n");

    printf("measurements started\n");

    while (1) 
	{
		char pressure_sensor_reading[256];
		uint16_t res = ADC_Read();	// ADC conversion for pressure sensor
      
		// Voltage Conversion
        float output_voltage = (res/1023.0) * 5.0;	
        if(output_voltage < 0.5){
            output_voltage = 0.5;
        }
        else if(output_voltage > 4.5){
            output_voltage = 4.5;
        }
		
		// Convert ADC voltage to pressure reading
    	float pressure_value = convert_ADC_to_pressure(res);	
        
		// sprintf(pressure_sensor_reading, "ADC Reading: %u", res);
		// HM10_transmit(pressure_sensor_reading);
		// sprintf(pressure_sensor_reading, "Output Voltage: %.2fV", output_voltage);
		// HM10_transmit(pressure_sensor_reading);

		// Send Pressure Reading to HM10
		sprintf(pressure_sensor_reading, "Pressure: %.2fkPa", pressure_value);
		HM10_transmit(pressure_sensor_reading);
		sprintf(pressure_sensor_reading, "\r\n\r\n");
		HM10_transmit(pressure_sensor_reading);

		_delay_ms(5000); // 5 seconds

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

			//Commented out code above is more concise. but formatting on DSD tech app is weird so have to send each measurement individually
			sprintf(SPS30_measurements, "measured values:\n");
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f pm1.0\n\r", m.mc_1p0);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f pm2.5\n\r", m.mc_2p5);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f pm4.0\n\r", m.mc_4p0);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f pm10.0\n\r", m.mc_10p0);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f nc0.5\n\r", m.nc_0p5);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f nc1.0\n\r", m.nc_1p0);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f nc2.5\n\r", m.nc_2p5);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f nc4.5\n\r", m.nc_4p0);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f nc10.0\n\r", m.nc_10p0);
			HM10_transmit(SPS30_measurements);  
			sprintf(SPS30_measurements, "%.2f typical particle size\n\r", m.typical_particle_size);
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
	printf(" to HM10\n\r");
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
	printf("Inside: HM10 Init\n");
	HM10_transmit("AT+RENEW");
	HM10_print_response_buffer();
	HM10_clear_response_buffer();
	sensirion_sleep_usec(2000000);
	// Wait until HM10 responds with "OK"
	while(!(HM10_response_buffer[0] == 0x4F && HM10_response_buffer[1] == 0x4B))
	{
		HM10_transmit("AT+RENEW");
		sensirion_sleep_usec(2000000);
	}
	HM10_print_response_buffer();
	HM10_clear_response_buffer();
	sensirion_sleep_usec(2000000);

	// HM10_transmit("AT");
	// sensirion_sleep_usec(2000000);

	// // Wait until HM10 responds with "OK"
	// while(!(HM10_response_buffer[0] == 0x4F && HM10_response_buffer[1] == 0x4B))
	// {
	// 	HM10_transmit("AT");
	// 	sensirion_sleep_usec(2000000);
	// }
	// HM10_print_response_buffer();
	// HM10_clear_response_buffer();
	
	// HM10_transmit("AT+RESET");
	// HM10_print_response_buffer();
	// HM10_clear_response_buffer();

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

//fucntions for ADC functionality
void ADC_init(){	//initialize ADC
	ADCSRA |= (1 << ADEN);	//enables ADC and interrupt enable
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	//prescales ADC (factor of 128)
	ADMUX = (1<<REFS0);  // AVCC (5V) as reference, ADC0 selected
	DDRC &= ~(1 << PINC0);	//setting input as PORTC pin 3
}

uint16_t ADC_Read() {
	uint16_t ADC_conversion;	//16 bit integer to hold ADC conversion result
	ADCSRA |= (1<<ADSC);  // Start conversion
	while (ADCSRA & (1<<ADSC));  // Wait for conversion to complete
	ADC_conversion |= ADCL;	//extracting conversion from lower register
	ADC_conversion |= (ADCH << 8);	//extracting upper bits of conversion from ADCH register
	return ADC_conversion;
}


float convert_ADC_to_pressure(uint16_t adc_value) {
	float output_voltage = (adc_value / 1023.0) * 5.0;	//using ADC conversion value to get sensor output voltage
	float pressure_reading = (output_voltage - 2.5) / 0.05;	//using pressure conversion formula to get sensor reading (page 5 of data sheet)
	if(pressure_reading < -40){
		pressure_reading = -40;
		printf("Extreme Negative Pressure");
	}
	else if(pressure_reading > 40){
		pressure_reading = 40;
		printf("Extreme Positive Pressure");
	}
	return pressure_reading;
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