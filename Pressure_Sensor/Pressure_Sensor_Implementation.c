/*
 * Pressure_Sensor_Implementation.c
 *
 * Created: 2/12/2025 4:12:16 PM
 * Author : harvi
 */ 

 #define F_CPU 16000000UL
 #define BAUD 9600                                   // define baud
 #define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate for UBRR
 #include <avr/io.h>
 #include <stdio.h>  // uart_transmit
 #include <stdint.h>
 #include <util/delay.h>
 
 /*
 #define F_CPU 16000000UL
 #define BAUD 9600                                   // define baud
 #define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate for UBRR
 */
 
 void uart_init (void);
 void uart_transmit (unsigned char data);
 int uart_putchar(char c, FILE *stream);
 void init_printf(void);
 void ADC_init(void);
 uint16_t ADC_Read();
 float convert_ADC_to_pressure(uint16_t adc_value);
 
 FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
 
 int main(void)
 {
     uart_init();  // Initialize UART
     init_printf();  // Initialize printf redirection
     ADC_init();	//initializing ADC
     
     while(1){
     uint16_t res = ADC_Read();	//ADC conversion
     printf("ADC Reading: %u\r\n", res);	//printing out ADC conversion
     
     float output_voltage = (res/1023.0) * 5.0;	//printing output voltage of sensor
     float pressure_value = convert_ADC_to_pressure(res);	//passing in ADC conversion to function to get sensor reading
     
     printf("Output Voltage: %.2fV\r\n", output_voltage);
     printf("Pressure: %.2f kPa\r\n\n", pressure_value);	//printing pressure value
     _delay_ms(3000);	//prints out pressure value every 5 seconds
     }
     
     
     
     return 0;
    
 }
 
 //functions to enable test print statements to console via UART
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
 //////////////////////////////////////////////////////////////////////////////
 
 //fucntions for ADC functionality
 void ADC_init(){	//initialize ADC
     ADCSRA |= (1 << ADEN) | (1 << ADIE);	//enables ADC and interrupt enable
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
     return pressure_reading;
 }