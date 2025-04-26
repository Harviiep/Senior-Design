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
#include <inttypes.h> // PRIx64
#include "sps30-i2c-3.1.1\sps30-i2c-3.1.1\sps30.h"
#include "scd41_driver\scd4x_i2c.h"
#define F_CPU 16000000UL
#include <util/delay.h>

#define BAUD 9600							   // define baud
#define BAUDRATE ((F_CPU) / (BAUD * 16UL) - 1) // set baud rate for UBRR
#define HM10_response_buffer_SIZE 64

// #ifndef PRIx64
// 	#error "PRIx64 is not defined! Check your compiler settings."
// #endif

void uart0_init(void);
void uart0_transmit(unsigned char data);
int uart1_putchar(char c, FILE *stream);
void init_printf(void);
void uart1_init(void);
void uart1_transmit(unsigned char data);
void HM10_transmit(const char *str);
void HM10_init();
void HM10_print_response_buffer();
uint8_t HM10_scan_for_poll_request();
void sps30_init();
void ADC_init(void);
void ADC_disable(void);
uint16_t ADC_Read();
float convert_ADC_to_pressure(uint16_t adc_value);
void convert_and_print_serial(uint16_t *serial_raw);

FILE uart1_output = FDEV_SETUP_STREAM(uart1_putchar, NULL, _FDEV_SETUP_WRITE);

volatile unsigned char HM10_response_buffer[HM10_response_buffer_SIZE];
volatile uint8_t HM10_response_buffer_index = 0;
uint8_t sensor_poll_requested = 0;

struct sps30_measurement m; // Struct to store sps30 measurements

int main(void)
{
	uart0_init();		  // Initialize UART
	init_printf();		  // Initialize printf redirection
	uart1_init();		  // Initalize UART 1 (HM10 Serial Connection)
	sei();				  // Enable global interrupts
	sensirion_i2c_init(); // Must call or sensirion libs don't work. Currently empty.
	sps30_init(); // Initialize SPS30
	//ADC_init();
	//HM10_init();
	//ADC_init(); // enable ADC
	int16_t error = NO_ERROR;
	// sensirion_i2c_hal_init(); //doesn't exist
	scd4x_init(SCD41_I2C_ADDR_62);

	// uint16_t serial_number[3] = {0};
	//  	_delay_ms(30);

	printf("Initialization Complete!\n\n");

	_delay_ms(3000);

	uint16_t warning_sent = 0;

	while (1)
	{
		// Print the HM10_response_buffer, ignoring empty indexes
		for (uint16_t i = 0; i < HM10_response_buffer_SIZE; i++)
		{	
			// waits for user to press "Poll sensors" button on app, which transmits "@" symbol to MCU through UART1 in HM10_response_buffer
			if (HM10_response_buffer[i] == '@')
			{ 
				printf("Polling sensors!\n");

				// ADC conversion for pressure sensor
				char pressure_sensor_reading[256];
				ADC_init();
				uint16_t res = ADC_Read(); 
				printf("ADC: %d\n", res);
				float output_voltage = (res / 1023.0) * 5.0;

				// Convert ADC voltage to pressure reading
				float pressure_value = convert_ADC_to_pressure(res);
				printf("voltage: %.2f\n", output_voltage);
				sprintf(pressure_sensor_reading, "%.2f kPa\n", pressure_value);
				HM10_transmit(pressure_sensor_reading);

				error = scd4x_wake_up();
				if (error != NO_ERROR)
				{
					printf("error executing wake_up(): %i\n", error);
				}

				error = scd4x_stop_periodic_measurement();
				if (error != NO_ERROR)
				{
					printf("error executing stop_periodic_measurement(): %i\n", error);
				}

				error = scd4x_reinit();
				if (error != NO_ERROR)
				{
					printf("error executing reinit(): %i\n", error);
				}

				error = scd4x_set_sensor_altitude(134);
				if (error != NO_ERROR) 
				{
					printf("error executing set_sensor_altitude(): %i\n", error);
				}

				error = scd4x_start_periodic_measurement();
				if (error != NO_ERROR)
				{
					printf("error executing measure_single_shot(): %i\n", error);
					return error;
				}

				bool data_ready = false;
				uint16_t co2_concentration = 0;
				int32_t temperature = 0;
				double relative_humidity = 0;
				uint16_t repetition = 0;

				_delay_ms(5000);

				error = scd4x_get_data_ready_status(&data_ready);
				if (error != NO_ERROR)
				{
					printf("error executing get_data_ready_status(): %i\n", error);
					continue;
				}

				while (!data_ready)
				{
					//_delay_ms(5000);
					error = scd4x_get_data_ready_status(&data_ready);
					// printf("inside get_data_ready\n\n");
					if (error != NO_ERROR)
					{
						printf("error executing get_data_ready_status(): %i\n", error);
						continue;
					}
				}

				error = scd4x_read_measurement(&co2_concentration, &temperature, &relative_humidity);
				if (error != NO_ERROR)
				{
					printf("error executing read_measurement(): %i\n", error);
					continue;
				}

				char scd41_readings[256];

				temperature /= 1000.0;		 // scaling temperature to print as normal Celsius
				relative_humidity /= 1000.0; // scaling relative humidity to print as normal percentage

				// Print results in physical units.
				// printf("External SCD41 Readings:\n");
				sprintf(scd41_readings, "CO2[ppm]: %u\n", co2_concentration);
				HM10_transmit(scd41_readings);

				sprintf(scd41_readings, "Temp[C]: %d\n", temperature);
				HM10_transmit(scd41_readings);

				sprintf(scd41_readings, "Humidity[RH]: %u%%\n", relative_humidity);
				HM10_transmit(scd41_readings);

				HM10_clear_response_buffer();

				_delay_ms(3000);

				warning_sent = 0;

				// -------------------------SPS 30 Cocde ------------------------------------------------------------
				int16_t SPS30_command_response_code = sps30_start_measurement();
				if (SPS30_command_response_code < 0){
					printf("error starting measurement\n");
				}

				printf("SPS30 Readings:\n");
				sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC); // wait 1s

				SPS30_command_response_code = sps30_read_measurement(&m);

				if (SPS30_command_response_code < 0)
				{
					printf("error reading measurement\n");
				}
				else
				{
					char SPS30_measurements[256];

				// 	sprintf(SPS30_measurements, "measured values:\n");
				// 	HM10_transmit(SPS30_measurements);
				// 	sprintf(SPS30_measurements, "%.2f pm1.0\n", m.mc_1p0);
				// 	HM10_transmit(SPS30_measurements);

					sprintf(SPS30_measurements,"%.2f pm2.5\n", m.mc_2p5);
					HM10_transmit(SPS30_measurements);

				// 	sprintf(SPS30_measurements,"%.2f pm4.0\n", m.mc_4p0);
				// 	HM10_transmit(SPS30_measurements);

					sprintf(SPS30_measurements,"%.2f pm10.0\n", m.mc_10p0);
					HM10_transmit(SPS30_measurements);

				// 	sprintf(SPS30_measurements,"%.2f nc0.5\n", m.nc_0p5);
				// 	HM10_transmit(SPS30_measurements);
				// 	sprintf(SPS30_measurements,"%.2f nc1.0\n", m.nc_1p0);
				// 	HM10_transmit(SPS30_measurements);

					sprintf(SPS30_measurements,"%.2f nc2.5\n", m.nc_2p5);
					HM10_transmit(SPS30_measurements);

				// 	sprintf(SPS30_measurements,"%.2f nc4.5\n", m.nc_4p0);
				// 	HM10_transmit(SPS30_measurements);
				
					sprintf(SPS30_measurements,"%.2f nc10.0\n", m.nc_10p0);
					HM10_transmit(SPS30_measurements);

					sprintf(SPS30_measurements,"%.2f typical particle size\n", m.typical_particle_size);
					HM10_transmit(SPS30_measurements);

					// 	sensirion_sleep_usec(2000000); // here specifically, sensirion_sleep_usec is the only delay that works for some reason */
				}
			}

			else
			{
				if (warning_sent != 1)
				{
					ADC_disable();
					printf("ADC Disabled\n");
					warning_sent += 1;
				}
			}
		}
	}

	return 0;
}

void uart0_init(void)
{
	SPCR0 &= ~(1 << SPE);	  // Disable SPI
	PRR0 &= ~(1 << PRUSART0); // enable usart1 in power register

	DDRD |= (1 << DDD1);  // Set TX1 (PB3) as output pd0 is rx of micro pd1 is transmit of micro,
	DDRD &= ~(1 << DDD0); // Set RX1 (PB4) as input

	UBRR0H = (BAUDRATE >> 8);							   // shift the register right by 8 bits
	UBRR0L = BAUDRATE;									   // set baud rate
	UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // enable receiver and transmitter
	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);			   // 8bit data format
	UCSR0C &= ~((1 << UMSEL01) | (1 << UMSEL00));		   // Clear UMSEL11 and UMSEL10 for async mode
}

void uart0_transmit(unsigned char data)
{
	while (!(UCSR0A & (1 << UDRE0)))
		;		 // wait while register is free
	UDR0 = data; // load data in the register
}

// Redirect printf to UART
int uart1_putchar(char c, FILE *stream)
{
	// Wait until the transmit buffer is empty
	while (!(UCSR1A & (1 << UDRE1)))
		;
	UDR1 = c;
	return 0;
}

// Initialize printf for UART
void init_printf(void)
{
	stdout = &uart1_output; // Link stdout to uart0_output
}

// Initialize UART1
void uart1_init(void)
{
	UBRR1H = (BAUDRATE >> 8);				 // shift the register right by 8 bits
	UBRR1L = BAUDRATE;						 // set baud rate
	UCSR1B |= (1 << TXEN1) | (1 << RXEN1);	 // enable receiver and transmitter
	UCSR1C |= (1 << UCSZ10) | (1 << UCSZ11); // 8bit data format
}

// Should use interrupt
void uart1_transmit(unsigned char data)
{
	// Wait for buffer to be empty
	while (!(UCSR1A & (1 << UDRE1)))
		;
	UDR1 = data;
}

// Send TX to HM10 over UART1
// Any TX to HM10 while not paired to a device will be considered as AT commands
// Once HM10 is paired to a device, any TX is considered plain text
// To go back into AT mode, send "AT"
void HM10_transmit(const char *str)
{
	printf("Sent ");
	while (*str)
	{
		printf("%c", *str);
		uart0_transmit(*str++);
	}
	printf(" to HM10\n");
}

// Clear HM10 buffer
void HM10_clear_response_buffer()
{
	memset((char *)HM10_response_buffer, 0, HM10_response_buffer_SIZE);
	HM10_response_buffer_index = 0;
}

// Print the response from HM10
void HM10_print_response_buffer()
{
	printf("HM10 Response: ");

	// Print the HM10_response_buffer, ignoring empty indexes
	for (uint16_t i = 0; i < HM10_response_buffer_SIZE; i++)
	{
		if (HM10_response_buffer[i] != 0x00)
		{
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
	HM10_print_response_buffer();
	HM10_clear_response_buffer();
	// Wait until HM10 responds with "OK"
	while (!(HM10_response_buffer[0] == 0x4F && HM10_response_buffer[1] == 0x4B))
	{
		HM10_transmit("AT");
		sensirion_sleep_usec(2000000);
	}
	HM10_print_response_buffer();
	HM10_clear_response_buffer();
	sensirion_sleep_usec(2000000);

	HM10_transmit("AT+ADDR?");
	sensirion_sleep_usec(2000000);
	HM10_print_response_buffer();
	HM10_clear_response_buffer();
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
	HM10_transmit("AT+NAMEScene_Safe123");
	sensirion_sleep_usec(2000000);
	HM10_print_response_buffer();
	HM10_clear_response_buffer();

	HM10_transmit("AT+ROLE0");
	sensirion_sleep_usec(2000000);
	HM10_print_response_buffer();
	HM10_clear_response_buffer();
}

uint8_t HM10_scan_for_poll_request()
{
	// Check for '@' sent by Bluetooth app indicating a request for sensor poll
	for (uint16_t i = 0; i < HM10_response_buffer_SIZE; i++)
	{
		if (HM10_response_buffer[i] == "@")
		{
			printf("%c", HM10_response_buffer[i]);
			HM10_clear_response_buffer();
			return 1;
		}
	}

	return 0;
}

// Initialize SPS30
void sps30_init()
{
	uint8_t fw_major;
	uint8_t fw_minor;
	int16_t SPS30_command_response_code;

	printf("Starting sps30 probe...\n");
	while (sps30_probe() != 0)
	{
		printf("SPS sensor probing failed\n");
		sensirion_sleep_usec(1000000); /* wait 1s */
	}
	printf("SPS sensor probing successful\n");

	SPS30_command_response_code = sps30_read_firmware_version(&fw_major, &fw_minor);
	if (SPS30_command_response_code)
	{
		printf("error reading firmware version\n");
	}
	else
	{
		printf("FW: %u.%u\n", fw_major, fw_minor);
	}

	char serial_number[SPS30_MAX_SERIAL_LEN];
	SPS30_command_response_code = sps30_get_serial(serial_number);
	if (SPS30_command_response_code)
	{
		printf("error reading serial number\n");
	}
	else
	{
		printf("Serial Number: %s\n", serial_number);
	}

	SPS30_command_response_code = sensirion_i2c_write(SPS30_I2C_ADDRESS, 0x5607, 2);
	if (SPS30_command_response_code)
	{
		printf("error initiating fan cleaning\n");
	}
	else
	{
		printf("Fan cleaned!\n");
	}
}

// fucntions for ADC functionality
void ADC_init()
{ // initialize ADC

	ADCSRA |= (1 << ADEN);								  // enables ADC
	ADMUX = ((1 << MUX1) | (1 << MUX0) | (1 << REFS0));	  // multiplexing to select ADC3 and setting reference voltage of 5V
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescales ADC (factor of 128)
	DDRC &= ~(1 << DDC3);								  // setting input as PORTC pin 3
}

void ADC_disable()
{
	ADCSRA &= ~(1 << ADEN); // turns ADC off
}

uint16_t ADC_Read()
{
	uint16_t ADC_conversion = 0; // 16 bit integer to hold ADC conversion result
	ADCSRA |= (1 << ADSC);		 // Start conversion
	while (ADCSRA & (1 << ADSC))
		;						   // Wait for conversion to complete
	ADC_conversion |= ADCL;		   // extracting conversion from lower register
	ADC_conversion |= (ADCH << 8); // extracting upper bits of conversion from ADCH register
	return ADC_conversion;
}

float convert_ADC_to_pressure(uint16_t adc_value)
{
	char pressure_reading_buffer[256];

	float output_voltage = (adc_value / 1023.0) * 5.0;		// using ADC conversion value to get sensor output voltage
	float pressure_reading = (output_voltage - 2.5) / 0.05; // using pressure conversion formula to get sensor reading (page 5 of data sheet)
	if (pressure_reading < -40)
	{
		pressure_reading = -40;
		sprintf(pressure_reading_buffer, "Extreme Negative Pressure\n");
		HM10_transmit(pressure_reading_buffer);
	}
	else if (pressure_reading > 40)
	{
		pressure_reading = 40;
		sprintf(pressure_reading_buffer, "Extreme Positive Pressure\n");
		HM10_transmit(pressure_reading_buffer);
	}
	memset(pressure_reading_buffer, 0, sizeof(pressure_reading_buffer)); // clearing pressure_reading_buffer
	return pressure_reading;
}

// scd41
void convert_and_print_serial(uint16_t *serial_raw)
{
	uint64_t serial_as_int = 0;
	sensirion_common_to_integer((uint8_t *)serial_raw, (uint8_t *)&serial_as_int,
								LONG_INTEGER, 6);
	// printf("0x%" PRIx64 "\n", serial_as_int);
	printf("0x%llx\n", serial_as_int);
}

// ISR to handle USART1 rx from HM10
ISR(USART0_RX_vect)
{
	volatile char received_char = UDR0;
	printf("Received: %c\n", received_char); // <- this prints the char!

	// Ensure buffer does not overflow
	if (HM10_response_buffer_index < HM10_response_buffer_SIZE - 1)
	{
		HM10_response_buffer[HM10_response_buffer_index] = received_char;
		HM10_response_buffer_index++;
	}
	else
	{
		printf("Buffer full, clearing...\n");
		printf("Buffer content before clear: ");
		HM10_print_response_buffer();
		HM10_clear_response_buffer();
	}
}