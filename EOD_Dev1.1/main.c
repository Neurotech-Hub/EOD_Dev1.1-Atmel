#define F_CPU 1000000UL // Assuming 1 MHz due to CKDIV8 fuse
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define BAUD 4800
#define BUFFER_SIZE 64
#define HALF_BUFFER_SIZE (BUFFER_SIZE / 2)  // Half buffer size for centering
#define THRESHOLD 200
#define MYUBRR F_CPU/16/BAUD-1
#define LED_PIN PA2
#define FRAM_CS_PIN PA7          // Chip Select for FRAM
#define MAGNET_SENSOR_PIN PB2 // PCINT10

volatile uint8_t start_sampling = 0;  // Flag to trigger sampling in main loop
int16_t adc_buffer[BUFFER_SIZE];      // Buffer to store ADC samples
int buffer_index = 0;                 // Current index in the circular buffer
volatile uint8_t threshold_crossed = 0;  // Flag for threshold crossing

void USART0_Transmit_16bit_with_frame(int16_t data);
int16_t ADC_read(void);

void sampling_and_transmit(void)
{
	int16_t adc_value;
	int event_index = -1;  // To record the index when threshold is crossed
	buffer_index = 0;      // Reset the circular buffer index

	// Continuously sample and fill the circular buffer
	for (int i = 0; i < BUFFER_SIZE; i++) {
		adc_buffer[buffer_index] = ADC_read();
		buffer_index = (buffer_index + 1) % BUFFER_SIZE;
	}

	// Now sample until the threshold is crossed
	while (!threshold_crossed) {
		// Read ADC value
		adc_value = ADC_read();

		// Store ADC value in circular buffer (wrap-around)
		adc_buffer[buffer_index] = adc_value;
		buffer_index = (buffer_index + 1) % BUFFER_SIZE;

		// Check if the threshold has been crossed
		if (abs(adc_value) > THRESHOLD) {
			threshold_crossed = 1;
			// Save the index where the threshold was crossed
			event_index = (buffer_index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
		}
	}

	// Continue sampling until the buffer is fully populated post-threshold
	for (int i = 0; i < HALF_BUFFER_SIZE; i++) {
		adc_buffer[buffer_index] = ADC_read();
		buffer_index = (buffer_index + 1) % BUFFER_SIZE;
	}

	// Now we need to rearrange the buffer to center the threshold event
	int centered_buffer[BUFFER_SIZE];  // Temporary buffer for centered waveform

	// Calculate start position to center the threshold crossing
	int start_index = (event_index - HALF_BUFFER_SIZE + BUFFER_SIZE) % BUFFER_SIZE;

	// Rearrange the circular buffer to center the threshold event
	for (int i = 0; i < BUFFER_SIZE; i++) {
		centered_buffer[i] = adc_buffer[(start_index + i) % BUFFER_SIZE];
	}

	// Transmit the centered buffer over USART
	for (int i = 0; i < BUFFER_SIZE; i++) {
		USART0_Transmit_16bit_with_frame(centered_buffer[i]);
		_delay_ms(20);  // Optional delay between transmissions
	}

	// Reset the threshold flag for the next event
	threshold_crossed = 0;
}


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
		PORTA |= (1 << LED_PIN);  // turn on the LED
		start_sampling = 1; // trigger ADC sample/transfer
		} else {
		// Magnet not applied, turn LED off
		PORTA &= ~(1 << LED_PIN);  // turn off the LED
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
	PRR &= ~(1 << PRADC); // Clear PRADC bit to enable ADC power
	//ADMUXB = (1 << REFS1); // REFS1 = 1 selects 1.1V internal reference
	ADMUXB = (1 << REFS1) | (1 << GSEL0);
	ADMUXA = 0x11; // 01 0001 (0x11): Differential input on PA0 and PA3, gain 1x
	ADCSRA = (1 << ADEN) | (1 << ADPS2); // | (1 << ADPS0); // ADC enable, prescaler=32
}

int16_t ADC_read(void)
{
	// Start conversion
	ADCSRA |= (1 << ADSC);

	// Wait for conversion to complete
	while (ADCSRA & (1 << ADSC));

	// Read the low byte first, then the high byte
	uint8_t low_byte = ADCL;
	uint8_t high_byte = ADCH;

	// Combine the low and high bytes
	int16_t result = (high_byte << 8) | low_byte;

	// Sign extend the 10-bit result
	if (result & 0x0200) {   // Check if the 10th bit (sign bit) is set
		result |= 0xFC00;    // Sign extend the upper bits
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
	ADC_init();
	//SPI_init();             // Initialize SPI for FRAM communication
	magnetic_sensor_init();

	// Example of writing and reading from FRAM
	// FRAM_write(0x0000, 0xBA);              // Write 0xAA to address 0x0000
	// uint8_t read_data = FRAM_read(0x0000); // Read from address 0x0000
	
	USART0_Init(MYUBRR); // Initialize USART0 with calculated baud rate

	while (1)
	{
		// Check if the flag is set by the interrupt
		if (start_sampling) {
			// Start sampling and transmitting
			sampling_and_transmit();

			// Reset the flag to wait for the next trigger
			start_sampling = 0;
		}
	}
}
