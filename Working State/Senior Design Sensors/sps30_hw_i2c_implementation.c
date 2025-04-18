/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//#include "sensirion_arch_config.h"
#include <stdint.h>
#include "sps30-i2c-3.1.1\sps30-i2c-3.1.1\sensirion_common.h"
#include "sps30-i2c-3.1.1\sps30-i2c-3.1.1\sensirion_i2c.h"
#include <avr/io.h>
#define __DELAY_BACKWARD_COMPATIBLE__
#define F_CPU 16000000UL
#include <util/delay.h>
#include <stdio.h>  // uart_transmit

//#define __DELAY_BACKWARD_COMPATIBLE__

// #define F_SCL 100000UL    // I2C clock frequency in Hz (100 kHz for standard mode)
// #define I2C_PS 1

#define BAUD 9600                                   // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate for UBRR
#define SPS30_ADDR 0x69
#define SCD41_ADDR 0x62

#define FOSC 16000000UL // Clock frequency = Oscillator freq .
#define BDIV ( FOSC / 100000 - 16) / 2 + 1

static volatile uint8_t selected_bus = 0;

/*
 * INSTRUCTIONS
 * ============
 *
 * Implement all functions where they are marked as IMPLEMENT.
 * Follow the function specification in the comments.
 */

uint8_t i2c_io(uint8_t device_addr, uint8_t *wp, uint16_t wn, uint8_t *rp, uint16_t rn);

/**
 * Select the current i2c bus by index.
 * All following i2c operations will be directed at that bus.
 *
 * THE IMPLEMENTATION IS OPTIONAL ON SINGLE-BUS SETUPS (all sensors on the same
 * bus)
 *
 * @param bus_idx   Bus index to select
 * @returns         0 on success, an error code otherwise
 */
int16_t sensirion_i2c_select_bus(uint8_t bus_idx) {
    selected_bus = bus_idx;
    return NO_ERROR;
}

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_init(void) {
    TWSR0 = 0;                           // Set prescalar for 1
    TWBR0 = BDIV;                        // Set bit rate register

	TWSR1 = 0;                           // Set prescalar for 1
    TWBR1 = BDIV;                        // Set bit rate register
}

/**
 * Release all resources initialized by sensirion_i2c_init().
 */
void sensirion_i2c_release(void) {
    // IMPLEMENT or leave empty if no resources need to be freed
}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count) {
    if(i2c_io(SPS30_ADDR, NULL, 0, data, count) != 0)
    	return STATUS_FAIL;
	return NO_ERROR;
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data,
                           uint16_t count) {
	if(i2c_io(SPS30_ADDR, data, count, NULL, 0) != 0)
    	return STATUS_FAIL;
	return NO_ERROR;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_sleep_usec(uint32_t useconds) {
    _delay_us(useconds);
	// while (useconds > 0) {
    //     if (useconds > 255) {
    //         _delay_us(255);  // Delay in chunks of 255 µs
    //         useconds -= 255;
    //     } else {
    //         _delay_us(useconds);
    //         useconds = 0;
    //     }
    // }
}

uint8_t i2c_io(uint8_t device_addr, uint8_t *wp, uint16_t wn, uint8_t *rp, uint16_t rn)
{
	uint8_t status, send_stop, wrote, start_stat;

	status = 0;
	wrote = 0;
	send_stop = 0;

	// I2C0 selected
	if(selected_bus = 0)
	{
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
					//printf("Going to nakstop\n");
					goto nakstop_0;
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
					goto nakstop_0;           // Send STOP condition
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
				goto nakstop_0;
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
		
		nakstop_0:                                    // Come here to send STOP after a NAK
		if (send_stop){
			TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);  // Send STOP condition
		}

		return(status);

	}
	
	// I2C1 selected
	else if (selected_bus == 1)
	{
		if (wn > 0) {
			wrote = 1;
			send_stop = 1;
	
			TWCR1 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);  // Send start condition
			while (!(TWCR1 & (1 << TWINT)));     // Wait for TWINT to be set
			status = TWSR1 & 0xf8;
			if (status != 0x08)                 // Check that START was sent OK
			{
				printf("Status != 0x08\n");
				return(status);
			}
	
			//TWDR0 = device_addr & 0xfe;          // Load device address and R/W = 0;
			TWDR1 = device_addr << 1;
			// 01101001 & 11111110 = 01101000
			// 01101001 << 1 = 11010010
			TWCR1 = (1 << TWINT) | (1 << TWEN);  // Start transmission
			while (!(TWCR1 & (1 << TWINT)));     // Wait for TWINT to be set
			status = TWSR1 & 0xf8;
			if (status != 0x18) {               // Check that SLA+W was sent OK
				if (status == 0x20)             // Check for NAK
				{
					//printf("Going to nakstop\n");
					goto nakstop_1;
				}               // Send STOP condition
				return(status);                 // Otherwise just return the status
			}
	
			// Write "wn" data bytes to the slave device
			while (wn-- > 0) {
				TWDR1 = *wp++;                   // Put next data byte in TWDR
				TWCR1 = (1 << TWINT) | (1 << TWEN); // Start transmission
				while (!(TWCR1 & (1 << TWINT))); // Wait for TWINT to be set
				status = TWSR1 & 0xf8;
				if (status != 0x28) {           // Check that data was sent OK
					if (status == 0x30)         // Check for NAK
					goto nakstop_1;           // Send STOP condition
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
			TWCR1 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
			// Send start (or repeated start) condition
			while (!(TWCR1 & (1 << TWINT)));     // Wait for TWINT to be set
			status = TWSR1 & 0xf8;
			if (status != start_stat)           // Check that START or repeated START sent OK
			return(status);
	
			//TWDR0 = device_addr  | 0x01;         // Load device address and R/W = 1;
			TWDR1 = (device_addr << 1) | 0x01;
			
			TWCR1 = (1 << TWINT) | (1 << TWEN);  // Send address+r
			while (!(TWCR1 & (1 << TWINT)));     // Wait for TWINT to be set
			status = TWSR1 & 0xf8;
			if (status != 0x40) {               // Check that SLA+R was sent OK
				if (status == 0x48)             // Check for NAK
				goto nakstop_1;
				return(status);
			}
	
			// Read all but the last of n bytes from the slave device in this loop
			rn--;
			while (rn-- > 0) {
				TWCR1 = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // Read byte and send ACK
				while (!(TWCR1 & (1 << TWINT))); // Wait for TWINT to be set
				status = TWSR1 & 0xf8;
				if (status != 0x50)             // Check that data received OK
				return(status);
				*rp++ = TWDR1;                   // Read the data
			}
	
			// Read the last byte
			TWCR1 = (1 << TWINT) | (1 << TWEN);  // Read last byte with NOT ACK sent
			while (!(TWCR1 & (1 << TWINT)));     // Wait for TWINT to be set
			status = TWSR1 & 0xf8;
			if (status != 0x58)                 // Check that data received OK
			return(status);
			*rp++ = TWDR1;                       // Read the data
	
			status = 0;                         // Set status value to successful
		}
		
		nakstop_1:                                    // Come here to send STOP after a NAK
		if (send_stop)
			TWCR1 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);  // Send STOP condition
		
		return(status);
	}
}

// from scd41 hal.c
// writes to only scd41 
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint16_t count) {
	if(i2c_io(SCD41_ADDR, data, count, NULL, 0) != 0)
		return STATUS_FAIL;
	return NO_ERROR;
}

int16_t sensirion_i2c_write_data(uint8_t address, const uint8_t* data, uint16_t data_length) {
	return sensirion_i2c_hal_write(address, data, data_length);
}

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
	if(i2c_io(SCD41_ADDR, NULL, 0, data, count) != 0)
		return STATUS_FAIL;
	return NO_ERROR;
}

int16_t sensirion_i2c_read_data_inplace(uint8_t address, uint8_t* buffer, uint16_t expected_data_length) {
	int16_t error;
	uint16_t i, j;

	uint16_t size = (expected_data_length / SENSIRION_WORD_SIZE) * (SENSIRION_WORD_SIZE + CRC8_LEN);

	if (expected_data_length % SENSIRION_WORD_SIZE != 0) {
		return BYTE_NUM_ERROR;
	}

	error = sensirion_i2c_hal_read(address, buffer, size);
	if (error) {
		return error;
	}

	for (i = 0, j = 0; i < size; i += SENSIRION_WORD_SIZE + CRC8_LEN) {
		error = sensirion_i2c_check_crc(&buffer[i], SENSIRION_WORD_SIZE, buffer[i + SENSIRION_WORD_SIZE]);
		if (error) {
			return error;
		}
		buffer[j++] = buffer[i];
		buffer[j++] = buffer[i + 1];
	}

	return NO_ERROR;
}

uint16_t sensirion_i2c_add_uint16_t_to_buffer(uint8_t* buffer, uint16_t offset, uint16_t data) 
{
	buffer[offset++] = (uint8_t)((data & 0xFF00) >> 8);
	buffer[offset++] = (uint8_t)((data & 0x00FF) >> 0);
	buffer[offset] = sensirion_i2c_generate_crc(&buffer[offset - SENSIRION_WORD_SIZE], SENSIRION_WORD_SIZE);
	offset++;

	return offset;
}

uint16_t sensirion_i2c_add_command16_to_buffer(uint8_t* buffer, uint16_t offset, uint16_t command)
{
	buffer[offset++] = (uint8_t)((command & 0xFF00) >> 8);
    buffer[offset++] = (uint8_t)((command & 0x00FF) >> 0);
    return offset;
}

int8_t sensirion_i2c_check_crc(const uint8_t* data, uint16_t count, uint8_t checksum) {
	if (sensirion_i2c_generate_crc(data, count) != checksum)
		return CRC_ERROR;
	return NO_ERROR;
}

uint8_t sensirion_i2c_generate_crc(const uint8_t* data, uint16_t count) {
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}