#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Define the CPU frequency for the delay functions
#define F_CPU 16000000UL

// Global flag to toggle the motor/buzzer functionality
// 'volatile' is used because this variable is modified within an ISR.
volatile uint8_t motor_enabled_state = 1;

/**
 * brief Initializes the Analog-to-Digital Converter (ADC).
 * * Configures the ADC for single conversion mode, using AVCC as the reference
 * voltage. The ADC prescaler is set to 128 to provide an ADC clock of 125KHz
 * (16MHz / 128), which is within the recommended range for maximum resolution.
 */
void ADC_Init(void) {
    // Set AVCC with external capacitor at AREF pin as the reference voltage
    ADMUX |= (1 << REFS0);
    // Enable the ADC (ADEN) and set the prescaler to 128 (ADPS2:0 = 111)
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

/**
 * brief Reads the value from a specified ADC channel.
 * param channel The ADC channel to read from (0-7).
 * return The 10-bit digital value from the ADC.
 */
uint16_t ADC_Read(uint8_t channel) {
    // Select the ADC channel by clearing the MUX bits first, then setting the new channel
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    // Start the single conversion
    ADCSRA |= (1 << ADSC);
    // Wait for the conversion to complete by polling the ADIF flag
    while (ADCSRA & (1 << ADSC));
    // Return the 10-bit result
    return ADC;
}

/**
 * * Configures Timer0 to generate a PWM signal on the OC0 pin (PB3) with a frequency
 * of 7.8125 KHz.
 * - Mode: Fast PWM (WGM01=1, WGM00=1)
 * - Output Compare Mode: Clear OC0 on compare match, set on TOP (COM01=1, COM00=0)
 * - Prescaler: 8 (CS01=1) with a 16MHz clock
 * - PWM Frequency = F_CPU / (N * 256) = 16,000,000 / (8 * 256) = 7812.5 Hz (7.8125 KHz)
 */
void PWM_Init(void) {
    // Set PB3 as an output for the PWM signal
    DDRB |= (1 << PB3);
    
    // Set Timer0 to Fast PWM mode (WGM01=1, WGM00=1)
    TCCR0 |= (1 << WGM00) | (1 << WGM01);
    
    // Set Compare Output Mode to non-inverting Fast PWM
    // Clear OC0 on Compare Match, set OC0 at BOTTOM (COM01=1, COM00=0)
    TCCR0 |= (1 << COM01);
    
    // Set the prescaler to 8 (CS01=1)
    TCCR0 |= (1 << CS01);
    
    // Initialize the duty cycle to 0% (motor off)
    OCR0 = 0;
}

/**
 * Initializes the GPIO pin for the buzzer control.
 * Sets PB5 as an output pin to drive the buzzer circuit.
 */
void Buzzer_Init(void) {
    // Set PB5 as an output pin
    DDRB |= (1 << PB5);
    // Ensure buzzer is off initially
    PORTB &= ~(1 << PB5);
}

/**
 * Configures INT1 on PD3 to trigger on a falling edge. The internal pull-up
 * resistor is enabled to provide a high logic level when the button is not
 * pressed.
 */
void External_Interrupt_Init(void) {
    // Set PD3 as an input
    DDRD &= ~(1 << PD3);
    // Enable internal pull-up resistor on PD3
    PORTD |= (1 << PD3);
    
    // Configure INT1 to trigger on a falling edge (ISC11=1, ISC10=0)
    MCUCR |= (1 << ISC11);
    MCUCR &= ~(1 << ISC10);
    
    // Enable External Interrupt 1
    GICR |= (1 << INT1);
}

/**
 * This ISR is executed when the push button is pressed (falling edge on PD3).
 * It toggles the global 'motor_enabled_state' flag and includes a small
 * debounce delay.
 */
ISR(INT1_vect) {
    _delay_ms(50); // Debounce delay
    // Only toggle the state if the button is still pressed
    if (bit_is_clear(PIND, PD3)) {
        motor_enabled_state = !motor_enabled_state;
    }
}

int main(void) {
    // 1. Initialize all peripherals
    ADC_Init();
    PWM_Init();
    Buzzer_Init();
    External_Interrupt_Init();

    // 2. Set initial DC motor direction (e.g., forward)
    // Connect PD0 to L293D IN1 and PD1 to IN2
    // To rotate forward, set IN1 (PD0) HIGH and IN2 (PD1) LOW
    DDRD |= (1 << PD0) | (1 << PD1);
    PORTD |= (1 << PD0);
    PORTD &= ~(1 << PD1);

    // 3. Enable global interrupts to allow the ISR to run
    sei();

    while (1) {
        // 4. Read temperature from the LM35 sensor connected to ADC channel 0
        uint16_t adc_value = ADC_Read(0);
        // Convert ADC value to temperature in Celsius
        // ADC_Read() returns a value from 0 to 1023 (10-bit).
        // V_out = (ADC_value / 1024) * V_ref = (ADC_value / 1024) * 5V
        // LM35 output is 10mV/°C, so Temp = V_out / 10mV
        // Temp = ((ADC_value / 1024) * 5000mV) / 10mV = (ADC_value * 500) / 1024
        float temperature = (float)adc_value * (500.0 / 1024.0);

        // 5. Apply the main control logic
        if (motor_enabled_state) {
            // Push button has not stopped the motor
            if (temperature < 25.0) {
                // Normal state: motor off, buzzer off
                OCR0 = 0; // 0% duty cycle
                PORTB &= ~(1 << PB5); // Buzzer off
            } else if (temperature >= 75.0) {
                // High temperature: motor max speed, buzzer on
                OCR0 = 255; // 100% duty cycle
                PORTB |= (1 << PB5); // Buzzer on
            } else {
                // Temperature is between 25°C and 75°C
                // Linearly scale duty cycle from 0% (at 25°C) to 100% (at 75°C)
                // Range: 25 to 75 degrees (a span of 50 degrees)
                // OCR0 range: 0 to 255 (a span of 255)
                // The linear formula is OCR0 = ( (Temp - 25) / 50 ) * 255
                uint8_t duty_cycle_value = (uint8_t)(((temperature - 25.0) / 50.0) * 255.0);
                
                // Set the duty cycle for the motor
                OCR0 = duty_cycle_value;
                // Buzzer should be off in this range
                PORTB &= ~(1 << PB5);
            }
        } else {
            // Push button has stopped the motor
            OCR0 = 0; // 0% duty cycle (motor off)
            PORTB &= ~(1 << PB5); // Buzzer off
        }

        // Small delay to prevent the loop from running too fast
        _delay_ms(100);
    }

    return 0;
}