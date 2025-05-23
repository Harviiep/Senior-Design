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

#include "sensirion_arch_config.h"
#include "sensirion_sw_i2c_gpio.h"
#include <avr/io.h>
#include <util/delay.h>

#define SDA_PIN PD2 // Pin for SDA
#define SCL_PIN PD3 // Pin for SCL

// Macros for readability
#define SDA_LOW() (PORTD &= ~(1 << SDA_PIN)) // Set SDA low
#define SDA_HIGH() (PORTD |= (1 << SDA_PIN)) // Set SDA high
#define SDA_INPUT() (DDRD &= ~(1 << SDA_PIN)) // Set SDA as input
#define SDA_OUTPUT() (DDRD |= (1 << SDA_PIN)) // Set SDA as output
#define SDA_READ() ((PIND & (1 << SDA_PIN)) != 0) // Read SDA

#define SCL_LOW() (PORTD &= ~(1 << SCL_PIN)) // Set SCL low
#define SCL_HIGH() (PORTD |= (1 << SCL_PIN)) // Set SCL high
#define SCL_INPUT() (DDRD &= ~(1 << SCL_PIN)) // Set SCL as input
#define SCL_OUTPUT() (DDRD |= (1 << SCL_PIN)) // Set SCL as output
#define SCL_READ() ((PIND & (1 << SCL_PIN)) != 0) // Read SCL
/*
 * INSTRUCTIONS
 * ============
 *
 * Implement all functions where they are marked as IMPLEMENT.
 * Follow the function specification in the comments.
 *
 * We use the following names for the two I2C signal lines:
 * SCL for the clock line
 * SDA for the data line
 *
 * Both lines must be equipped with pull-up resistors appropriate to the bus
 * frequency.
 */

/**
 * Initialize all hard- and software components that are needed to set the
 * SDA and SCL pins.
 */
void sensirion_init_pins(void) {
    // IMPLEMENT
	// Set SDA and SCL as outputs and drive them high (idle state)
	SDA_OUTPUT();
	SCL_OUTPUT();
	SDA_HIGH();
	SCL_HIGH();
}

/**
 * Release all resources initialized by sensirion_init_pins()
 */
void sensirion_release_pins(void) {
    // IMPLEMENT or leave empty if no resources need to be freed
	// Reset SDA and SCL pins to inputs (floating)
	SDA_INPUT();
	SCL_INPUT();
}

/**
 * Configure the SDA pin as an input. With an external pull-up resistor the line
 * should be left floating, without external pull-up resistor, the input must be
 * configured to use the internal pull-up resistor.
 */
void sensirion_SDA_in(void) {
    // IMPLEMENT
	    SDA_INPUT();
	    SDA_HIGH(); // Enable pull-up if external pull-up is not present
}

/**
 * Configure the SDA pin as an output and drive it low or set to logical false.
 */
void sensirion_SDA_out(void) {
    // IMPLEMENT
	SDA_OUTPUT();
	SDA_LOW();
}

/**
 * Read the value of the SDA pin.
 * @returns 0 if the pin is low and 1 otherwise.
 */
uint8_t sensirion_SDA_read(void) {
    // IMPLEMENT
	return SDA_READ();
    //return 1;
}

/**
 * Configure the SCL pin as an input. With an external pull-up resistor the line
 * should be left floating, without external pull-up resistor, the input must be
 * configured to use the internal pull-up resistor.
 */
void sensirion_SCL_in(void) {
    // IMPLEMENT
	SCL_INPUT();
	SCL_HIGH(); // Enable pull-up if external pull-up is not present
}

/**
 * Configure the SCL pin as an output and drive it low or set to logical false.
 */
void sensirion_SCL_out(void) {
    // IMPLEMENT
	SCL_OUTPUT();
	SCL_LOW();
}

/**
 * Read the value of the SCL pin.
 * @returns 0 if the pin is low and 1 otherwise.
 */
uint8_t sensirion_SCL_read(void) {
    // IMPLEMENT
	return SCL_READ();
    //return 1;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution approximately, but no less than, the given time.
 *
 * The precision needed depends on the desired i2c frequency, i.e. should be
 * exact to about half a clock cycle (defined in
 * `SENSIRION_I2C_CLOCK_PERIOD_USEC` in `sensirion_arch_config.h`).
 *
 * Example with 400kHz requires a precision of 1 / (2 * 400kHz) == 1.25usec.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_sleep_usec(uint32_t useconds) {
    while (useconds--) {
	    _delay_us(1);
    }
}
