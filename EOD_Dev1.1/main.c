#define F_CPU 1000000UL // Assuming 1 MHz due to CKDIV8 fuse
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define BAUD 2400
#define BIT_DELAY _delay_us(365) // Adjust for 9600 baud
#define TX_PIN PA2               // Define PA2 as the TX pin for software UART
#define LED_PIN PA3              // Define PA3 as the LED pin for blinking
#define FRAM_CS_PIN PA7          // Chip Select for FRAM
#define MAGNET_SENSOR_PIN PB3    // PB3 as the magnetic sensor pin

void blinkLED(void);

// Initialize the magnetic sensor as an input with an interrupt
void magnetic_sensor_init(void)
{
	// Set PB3 as input
	DDRB &= ~(1 << MAGNET_SENSOR_PIN);

	// Enable internal pull-up resistor on PB3 (active LOW sensor)
	PORTB |= (1 << MAGNET_SENSOR_PIN);

	// Enable pin change interrupt on PB3 (PCINT11)
	PCMSK1 |= (1 << PCINT11); // Enable pin change interrupt for PB3
	GIMSK |= (1 << PCIE1);    // Enable pin change interrupts for PCINT[11:8]
}

// ISR for handling pin change interrupts on PCINT[11:8] (PB3, PB2, PB1, PB0)
ISR(PCINT1_vect)
{
	// Check if PB3 is low (magnet is applied)
	if (!(PINB & (1 << MAGNET_SENSOR_PIN)))
	{
		//blink_LED();
	}
	else
	{
		// Magnet removed (PB3 is HIGH)
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

void soft_serial_init(void)
{
	DDRA |= (1 << TX_PIN);  // Set PA2 as output for software serial
	PORTA |= (1 << TX_PIN); // Set PA2 high (idle state)
	DDRA |= (1 << LED_PIN); // Set PA3 as output for LED
}

void soft_serial_send(uint8_t data)
{
	// Start bit (pull line low)
	PORTA &= ~(1 << TX_PIN);
	BIT_DELAY;

	// Send 8 data bits (LSB first)
	for (uint8_t i = 0; i < 8; i++)
	{
		if (data & (1 << i))
		{
			PORTA |= (1 << TX_PIN); // Send 1
		}
		else
		{
			PORTA &= ~(1 << TX_PIN); // Send 0
		}
		BIT_DELAY;
	}

	// Stop bit (set line high)
	PORTA |= (1 << TX_PIN);
	BIT_DELAY;
}

void ADC_init(void)
{
	// Clear PRADC bit to enable ADC power
	PRR &= ~(1 << PRADC);

	// Set the voltage reference to 1.1V internal reference (REFS1 = 1)
	ADMUXB = (1 << REFS1); // REFS1 = 1 selects 1.1V internal reference

	// Select differential input: PA0 (ADC0) - PA1 (ADC1), with gain 1x
	ADMUXA = 0x10; // 01 0000: Differential input on PA0 and PA1, gain 1x

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
	PORTA ^= (1 << LED_PIN); // Toggle PA3 to blink LED
}

int main(void)
{
	soft_serial_init();     // Initialize software serial on PA2 and LED on PA3
	ADC_init();             // Initialize ADC for differential input on PA0 and PA1
	SPI_init();             // Initialize SPI for FRAM communication
	magnetic_sensor_init(); // Initialize magnetic sensor with interrupt on PB3

	soft_serial_send(0x55); // send for baud rate detection

	_delay_ms(1000); // Delay between readings (and LED toggles)

	// Example of writing and reading from FRAM
	// FRAM_write(0x0000, 0xBA);              // Write 0xAA to address 0x0000
	// uint8_t read_data = FRAM_read(0x0000); // Read from address 0x0000
	//soft_serial_send(read_data);           // Send the read byte over software UART
	//_delay_ms(1000); // Delay between readings (and LED toggles)

	//sei(); // Enable global interrupts

	while (1)
	{
		int16_t adc_value = ADC_read(); // Read ADC value

		// Blink PA3 every time ADC is sampled
		blink_LED();

		// Send ADC high and low byte over software serial
		soft_serial_send((adc_value >> 8) & 0xFF); // Send high byte
		soft_serial_send(adc_value & 0xFF);        // Send low byte
		//soft_serial_send(0x55);

		_delay_ms(30); // Delay between readings (and LED toggles)
	}
}
