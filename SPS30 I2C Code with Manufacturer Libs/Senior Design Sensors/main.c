/*
 * Senior Design Sensors.c
 *
 * Created: 11/21/2024 8:34:08 AM
 * Author : tye84
 */ 

#include <avr/io.h>
#include <stdio.h>  // uart_transmit
#include <stdint.h>
#include "sps30-i2c-3.1.1\sps30-i2c-3.1.1\sps30.h"

#define F_CPU 16000000UL
#define BAUD 9600                                   // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate for UBRR

void uart_init (void);
void uart_transmit (unsigned char data);
int uart_putchar(char c, FILE *stream);
void init_printf(void);

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

int main(void)
{
	uart_init();  // Initialize UART
	init_printf();  // Initialize printf redirection
	
	// struct sps30_measurement m;
    // int16_t ret;

    // /* Initialize I2C bus */
    // sensirion_i2c_init();
	
	// //printf("probe=%d\n", sps30_probe());
	
	// //sensirion_i2c_write(SPS30_I2C_ADDRESS, 0x5607, 16);
	
	
	// printf("Probing!");
	// if(sps30_probe() != 0)
	// {
	// 	printf("SPS sensor probing failed\n");
	// }
	
	// /*
	// while (sps30_probe() != 0) {
	// 	printf("SPS sensor probing failed\n");
	// 	sensirion_sleep_usec(1000000);
	// }
	// */
	
	// //printf("SPS sensor probing successful\n");
	
	
	// ret = sps30_start_measurement();
	// if (ret < 0){
	// 	printf("error starting measurement\n");
	// }
	// printf("measurements started\n");
	
	
    // /* Replace with your application code */
    // while (1) 
    // {
		
	// 	sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC); 
	// 	ret = sps30_read_measurement(&m);
	// 	if (ret < 0) {
	// 		printf("error reading measurement\n");

	// 		} else {
	// 		printf("measured values:\n"
	// 		"\t%0.2f pm1.0\n"
	// 		"\t%0.2f pm2.5\n"
	// 		"\t%0.2f pm4.0\n"
	// 		"\t%0.2f pm10.0\n"
	// 		"\t%0.2f nc0.5\n"
	// 		"\t%0.2f nc1.0\n"
	// 		"\t%0.2f nc2.5\n"
	// 		"\t%0.2f nc4.5\n"
	// 		"\t%0.2f nc10.0\n"
	// 		"\t%0.2f typical particle size\n\n",
	// 		m.mc_1p0, m.mc_2p5, m.mc_4p0, m.mc_10p0, m.nc_0p5, m.nc_1p0,
	// 		m.nc_2p5, m.nc_4p0, m.nc_10p0, m.typical_particle_size);
	// 	}
		
    // }

	struct sps30_measurement m;
    int16_t ret;

    /* Initialize I2C bus */
    sensirion_i2c_init();

    /* Busy loop for initialization, because the main loop does not work without
     * a sensor.
     */
    while (sps30_probe() != 0) {
        printf("SPS sensor probing failed\n");
        sensirion_sleep_usec(1000000); /* wait 1s */
    }
    printf("SPS sensor probing successful\n");

    uint8_t fw_major;
    uint8_t fw_minor;
    ret = sps30_read_firmware_version(&fw_major, &fw_minor);
    if (ret) {
        printf("error reading firmware version\n");
    } else {
        printf("FW: %u.%u\n", fw_major, fw_minor);
    }

    char serial_number[SPS30_MAX_SERIAL_LEN];
    ret = sps30_get_serial(serial_number);
    if (ret) {
        printf("error reading serial number\n");
    } else {
        printf("Serial Number: %s\n", serial_number);
    }

    ret = sps30_start_measurement();
    if (ret < 0)
        printf("error starting measurement\n");
    printf("measurements started\n");

    while (1) {
        sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC); /* wait 1s */
        ret = sps30_read_measurement(&m);
        if (ret < 0) {
            printf("error reading measurement\n");

        } else {
            printf("measured values:\n"
                   "\t%0.2f pm1.0\n"
                   "\t%0.2f pm2.5\n"
                   "\t%0.2f pm4.0\n"
                   "\t%0.2f pm10.0\n"
                   "\t%0.2f nc0.5\n"
                   "\t%0.2f nc1.0\n"
                   "\t%0.2f nc2.5\n"
                   "\t%0.2f nc4.5\n"
                   "\t%0.2f nc10.0\n"
                   "\t%0.2f typical particle size\n\n",
                   m.mc_1p0, m.mc_2p5, m.mc_4p0, m.mc_10p0, m.nc_0p5, m.nc_1p0,
                   m.nc_2p5, m.nc_4p0, m.nc_10p0, m.typical_particle_size);
        }
    }

    return 0;
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

