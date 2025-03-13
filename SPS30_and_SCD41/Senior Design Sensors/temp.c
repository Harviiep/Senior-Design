/*
 * Pressure_Sensor_Implementation.c
 *
 * Created: 2/12/2025 4:12:16 PM
 * Author : harvi
 */ 

 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <stdio.h>
 #include <stdint.h>
 #include <string.h>
 
 
 #define F_CPU 16000000UL
 #define BAUD 9600                                   // define baud
 #define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate for UBRR
 #define HM10_response_buffer_SIZE 64
 #include <util/delay.h>
  
  
  void uart0_init (void);
  void uart0_transmit (unsigned char data);
  int uart0_putchar(char c, FILE *stream);
  void init_printf(void);
  void uart1_init (void);
  void uart1_transmit(unsigned char data);
  void HM10_transmit(const char* str);
  //void HM10_init();
  void HM10_print_response_buffer();
  void ADC_init(void);
  uint16_t ADC_Read();
  float convert_ADC_to_pressure(uint16_t adc_value);
  
  FILE uart0_output = FDEV_SETUP_STREAM(uart0_putchar, NULL, _FDEV_SETUP_WRITE);
  
  volatile unsigned char HM10_response_buffer[HM10_response_buffer_SIZE];
  volatile uint8_t HM10_response_buffer_index = 0;
  //volatile uint8_t HM10_initialized = 0;
  
  int main(void)
  {
      uart0_init();  // Initialize UART
      init_printf();  // Initialize printf redirection
      uart1_init(); // Initalize UART 1 (HM10 Serial Connection)
      sei(); // Enable global interrupts
      ADC_init();	//initializing ADC
      HM10_init();
      
      printf("We are done\r\n");
      //_delay_ms(4000);
      
      //printf("Initialization Complete!\n\n");
      
      while(1){
         //printf("we are in the while loop now yayyy \r\n");
         char pressure_sensor_reading[256];
         uint16_t res = ADC_Read();	//ADC conversion
         printf("ADC Reading: %u\r\n", res);	//printing out ADC conversion
      
         float output_voltage = (res/1023.0) * 5.0;	//printing output voltage of sensor
         if(output_voltage < 0.5){
             output_voltage = 0.5;
         }
         else if(output_voltage > 4.5){
             output_voltage = 4.5;
         }
      
         float pressure_value = convert_ADC_to_pressure(res);	//passing in ADC conversion to function to get sensor reading
         
         sprintf(pressure_sensor_reading, "ADC Reading: %u", res);
         HM10_transmit(pressure_sensor_reading);
         sprintf(pressure_sensor_reading, "Output Voltage: %.2fV", output_voltage);
         HM10_transmit(pressure_sensor_reading);
         sprintf(pressure_sensor_reading, "Pressure: %.2fkPa", pressure_value);
         HM10_transmit(pressure_sensor_reading);
         sprintf(pressure_sensor_reading, "\r\n\r\n");
         HM10_transmit(pressure_sensor_reading);
      
      //printf("Output Voltage: %.2fV\r\n", output_voltage);
      //printf("Pressure: %.2f kPa\r\n\n", pressure_value);	//printing pressure value
         _delay_ms(3000);	//prints out pressure value every 5 seconds
      }
      
      
      
      return 0;
     
  }
  
  //functions to enable test print statements to console via UART
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
      while (!(UCSR0A & (1 << UDRE0))) {
          ;  // Wait until the transmit buffer is empty
      }
      UDR0 = c;  // Send the character
      return 0;  // Return 0 to indicate success
  }
  // Initialize printf for UART
  void init_printf(void) {
      stdout = &uart0_output;  // Link stdout to uart_output
  }
  
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
  
  void HM10_init()
  {
      HM10_clear_response_buffer();
      HM10_transmit("AT");
      _delay_ms(2000);
      //sensirion_sleep_usec(2000000);
 
      // Wait until HM10 responds with "OK"
      
      while(!(HM10_response_buffer[0] == 0x4F && HM10_response_buffer[1] == 0x4B))
      {
          HM10_transmit("AT");
          _delay_ms(2000);
             
      }
      HM10_print_response_buffer();
      HM10_clear_response_buffer();
 
      // Start broadcasting
      HM10_transmit("AT+NAMEScene_Safe");
      _delay_ms(2000);
      HM10_print_response_buffer();
      HM10_clear_response_buffer();
 
      HM10_transmit("AT+ROLE0");
      _delay_ms(2000);
      //sensirion_sleep_usec(2000000);
      HM10_print_response_buffer();
      HM10_clear_response_buffer();
 
      HM10_transmit("AT+ROLE?");
      _delay_ms(2000);
      //sensirion_sleep_usec(2000000);
      HM10_print_response_buffer();
      HM10_clear_response_buffer();
      
      HM10_transmit("AT+BAUD?");
      _delay_ms(2000);
      HM10_print_response_buffer();
      HM10_clear_response_buffer();
      
      //HM10_initialized = 1;
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
      if(pressure_reading < -40){
          pressure_reading = 40;
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