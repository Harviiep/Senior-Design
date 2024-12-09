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

#define F_CPU 16000000UL


#include <util/delay.h>

#define F_SCL 100000UL    // I2C clock frequency in Hz (100 kHz for standard mode)
#define I2C_PS 1
/*
 * INSTRUCTIONS
 * ============
 *
 * Implement all functions where they are marked as IMPLEMENT.
 * Follow the function specification in the comments.
 */

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
    // IMPLEMENT or leave empty if all sensors are located on one single bus
    return STATUS_FAIL;
}

void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;

    while ( 1 )
    {
	    // send START condition
	    TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR0 & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TWSR0 & 0xF8;
    	if ( (twst != 0x08) && (twst != 0x10)) continue;
    
    	// send device address
    	TWDR0 = address;
    	TWCR0 = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR0 & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TWSR0 & 0xF8;
    	if ( (twst == 0x20 )||(twst == 0x48) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR0 & (1<<TWSTO));
	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }

}/* i2c_start_wait */

/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;

	// send START condition
	TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TWSR0 & 0xF8;
	if ( (twst != 0x08) && (twst != 0x10)) return 1;

	// send device address
	TWDR0 = address;
	TWCR0 = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR0 & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TWSR0 & 0xF8;
	if ( (twst != 0x18) && (twst != 0x40) ) printf("SLA Transmitted, ACK Not Received\n");

	return 0;

}/* i2c_start */

unsigned char i2c_rep_start(unsigned char address)
{
    return i2c_start( address );

}

void i2c_stop(void)
{
	/* send stop condition */
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR0 & (1<<TWSTO));

}

/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{	
    uint8_t   twst;
    
	// send data to the previously addressed device
	TWDR0 = data;
	TWCR0 = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TWSR0 & 0xF8;
	if( twst != 0x28) return 1;
	return 0;

}

unsigned char i2c_readAck(void)
{
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR0 & (1<<TWINT)));

	return TWDR0;

}
unsigned char i2c_readNak(void)
{
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR0 & (1<<TWINT)));
	
	return TWDR0;

}

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_init(void) {
	uint32_t twbr_value = (F_CPU-16*F_SCL)/(2*F_SCL*I2C_PS);
	TWBR0 = (uint8_t)twbr_value;
	
	// Set prescaler to 1 (TWSR default value is 0)
	TWSR0 &= ~((1 << TWPS1) | (1 << TWPS0));
	
	// Enable internal Pullups
	DDRC &= ~(1 << DDC4) & ~(1 << DDC5);
	PORTC |= (1 << PORTC4) | (1 << PORTC5);
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
	uint16_t i;

	address = (address << 1) | 0;
	i2c_start(address);
	
	// Read bytes from the I2C bus
    for (i = 0; i < count; i++) {
        if (i < count - 1) {
            // Use i2c_readAck to request the next byte
            data[i] = i2c_readAck();
        } else {
            // Use i2c_readNak for the last byte to indicate end of read
            data[i] = i2c_readNak();
        }
    }

    // Stop the I2C communication
    i2c_stop();
	
	printf("data=0x%02X\n", data);
    //printf("Read %u bytes from address 0x%02X\n", count, address);
    return 0;
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
	uint16_t i;
	address = (address << 1) | 0;
	i2c_start(address);
	
	// Write each byte of the data buffer
    for (i = 0; i < count; i++) {
        if (i2c_write(data[i])) {
            printf("Data byte %d not acknowledged\n", i);
            return 1;  // Stop on error
        }
    }

	i2c_stop();
	
	printf("Write complete!");
	return 0;
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
	//uint32_t temp = useconds * 1000;
    //_delay_ms(temp);
	
	uint32_t ms = useconds / 1000;
	uint32_t us = useconds % 1000;
	
	// Delay in milliseconds
	while (ms--) {
		_delay_ms(1);
	}

	// Delay remaining microseconds
	while (us--) {
		_delay_us(1);
	}
}
