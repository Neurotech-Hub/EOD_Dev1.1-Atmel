#define F_CPU 1000000UL // Assuming 1 MHz due to CKDIV8 fuse
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define BAUD 4800
#define MYUBRR F_CPU/16/BAUD-1
#define LED_PIN PA2              // Define as the LED pin for blinking
#define FRAM_CS_PIN PA7          // Chip Select for FRAM
#define MAGNET_SENSOR_PIN PB2 // PCINT10

void USART0_Transmit_16bit_with_frame(int16_t data);
void magnetic_sensor_init(void)
{
	// Set PB2 as input
	DDRB &= ~(1 << MAGNET_SENSOR_PIN);

	// Enable internal pull-up resistor on PB2 (active LOW sensor)
	PORTB |= (1 << MAGNET_SENSOR_PIN);

	// Enable pin change interrupt on PB2 (PCINT10)
	PCMSK1 |= (1 << PCINT10);  // Enable pin change interrupt mask for PB2

	// Enable pin change interrupt for PCINT[11:8] group
	GIMSK |= (1 << PCIE1);     // Enable PCINT1 group interrupts (PCINT[11:8])

	// Enable global interrupts
	sei();
}

// ISR for Pin Change Interrupt on PCINT[11:8]
ISR(PCINT1_vect)
{
	if (!(PINB & (1 << MAGNET_SENSOR_PIN))) {
		// Magnet applied, turn LED on
		PORTA |= (1 << LED_PIN);  // Set PB1 high to turn on the LED
		} else {
		// Magnet not applied, turn LED off
		PORTA &= ~(1 << LED_PIN);  // Set PB1 low to turn off the LED
	}
}

// Initialize SPI for the ATTiny841
void SPI_init(void)
{
	// Set CS, SCK, and MOSI as output, MISO as input
	DDRA |= (1 << FRAM_CS_PIN) | (1 << PA4) | (1 << PA6); // CS, SCK, MOSI as output
	DDRA &= ~(1 << PA5);                                  // MISO as input

	// Set CS high (inactive)
	PORTA |= (1 << FRAM_CS_PIN);

	// Enable SPI, set as Master, and clock rate (F_CPU / 16)
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); // SPI enable, master, F_CPU/16
}

// SPI transfer function
uint8_t SPI_transfer(uint8_t data)
{
	SPDR = data; // Load data into SPI data register
	while (!(SPSR & (1 << SPIF)))
	;        // Wait for transmission complete
	return SPDR; // Return received data
}

// Write data to FRAM
void FRAM_write(uint16_t address, uint8_t data)
{
	PORTA &= ~(1 << FRAM_CS_PIN); // Pull CS low to select FRAM

	SPI_transfer(0x02);                  // Send WRITE opcode
	SPI_transfer((address >> 8) & 0xFF); // Send high byte of address
	SPI_transfer(address & 0xFF);        // Send low byte of address
	SPI_transfer(data);                  // Send data byte

	PORTA |= (1 << FRAM_CS_PIN); // Pull CS high to deselect FRAM
}

// Read data from FRAM
uint8_t FRAM_read(uint16_t address)
{
	uint8_t data;

	PORTA &= ~(1 << FRAM_CS_PIN); // Pull CS low to select FRAM

	SPI_transfer(0x03);                  // Send READ opcode
	SPI_transfer((address >> 8) & 0xFF); // Send high byte of address
	SPI_transfer(address & 0xFF);        // Send low byte of address
	data = SPI_transfer(0xFF);           // Read data

	PORTA |= (1 << FRAM_CS_PIN); // Pull CS high to deselect FRAM

	return data;
}

void ADC_init(void)
{
	// Clear PRADC bit to enable ADC power
	PRR &= ~(1 << PRADC);

	// Set the voltage reference to 1.1V internal reference (REFS1 = 1)
	ADMUXB = (1 << REFS1); // REFS1 = 1 selects 1.1V internal reference

	// Select differential input: PA0 (ADC0) - PA1 (ADC1), with gain 1x
	ADMUXA = 0x11; // 01 0001 (0x11): Differential input on PA0 and PA3, gain 1x

	// Enable ADC and set prescaler to 64 (for 125kHz ADC clock with 1 MHz F_CPU)
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // ADC enable, prescaler=64
}

int16_t ADC_read(void)
{
	// Start conversion
	ADCSRA |= (1 << ADSC);

	// Wait for conversion to complete
	while (ADCSRA & (1 << ADSC))
	;

	// Read the result from ADCL and ADCH
	int16_t result = (ADCL | (ADCH << 8));

	// Sign extend the 10-bit result (considering the result is in two's complement form)
	if (result & 0x0200)
	{                     // Check if the 10th bit (sign bit) is set
		result |= 0xFC00; // If yes, extend the sign by setting the upper bits
	}
	return result;
}

void blink_LED(void)
{
	PORTB ^= (1 << LED_PIN); // Toggle
}

void USART0_Init(unsigned int ubrr)
{
	// Set baud rate
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;

	// Enable transmitter
	UCSR0B = (1<<TXEN0);

	// Set frame format: 8 data bits, no parity, 1 stop bit
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
}

void USART0_Transmit(unsigned char data)
{
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1<<UDRE0)));

	// Put data into buffer, sends the data
	UDR0 = data;
}

void USART0_Transmit_16bit_with_frame(int16_t data)
{
	// Send start byte
	USART0_Transmit(0xAA);                // Start of frame
	
	// Send the high byte
	USART0_Transmit((data >> 8) & 0xFF);  // Transmit the upper 8 bits
	
	// Send the low byte
	USART0_Transmit(data & 0xFF);         // Transmit the lower 8 bits
	
	// Optionally send an end byte
	// USART0_Transmit(0x55);             // End of frame (optional)
}

int main(void)
{
	DDRA |= (1 << LED_PIN); // LED as output
	//ADC_init();
	//SPI_init();             // Initialize SPI for FRAM communication
	magnetic_sensor_init();

	// Example of writing and reading from FRAM
	// FRAM_write(0x0000, 0xBA);              // Write 0xAA to address 0x0000
	// uint8_t read_data = FRAM_read(0x0000); // Read from address 0x0000
	
	USART0_Init(MYUBRR); // Initialize USART0 with calculated baud rate

	while (1)
	{
		//int16_t adc_value = ADC_read(); // Read ADC value
		//USART0_Transmit_16bit_with_frame(adc_value);
		//blink_LED();
		

		_delay_ms(20); // Delay between readings (and LED toggles)
	}
}
