#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>


#define SPS30_I2C_ADDR 0x69    // SPS30 I2C address

void setup();	//function to initialize i2c communication
void SPS30_function();	//function to read from particulate matter (PM) sensor

// Global variable to hold sensor readings for Bluetooth transmission (if required)
char errorMessage[64];
int16_t error;


int main(){
	setup();
}

void setup() {
	
	PRR0 &= ~(1 << PRTWI0);	//setting power reduction TWI bit in Power Reduction Register to 0; enables two-wire Serial interface
	
	PORTB |= (1 << 5);	//turning on LED to test function

}

void SPS30_function(){
	//page 18 of SPS30 datasheet has I2C commands
	TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);	//sending START condition
	while (!(TWCR0 & (1 << TWINT)));	//busy while to wait for START to be transmitted
	
	//here are where commands to write data from sensor to TWDR0 would go
	//0x0010 - command to start measurement of particulate matter concentration (measures different sizes of PM [1.0, 2.5, 4.0, 10] ug/m^3)
	//0x0104 - command to stop measurements
	//0x0300 - command to read measured values 
	
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);	//transmit STOP condition
	
}