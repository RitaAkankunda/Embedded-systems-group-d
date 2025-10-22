/*
 * Alcohol detector.c
 *
 * Created: 10/22/2025 4:04:30 PM (Modified for timed, always-responsive 3-second alert)
 * Author : Owner
 */ 

#define F_CPU 16000000UL  // 16 MHz clock speed

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h> // Include standard boolean types for clarity

// Pin definitions for Arduino Mega
#define MQ3_PIN 0        // ADC0 (PF0) - A0 on Mega
#define LED_PIN 7        // PH4 - Digital Pin 7 on Mega
#define BUZZER_PIN 8     // PH5 - Digital Pin 8 on Mega

int threshold = 400;     // Alcohol detection threshold (adjust as needed)

// STATE FLAG: Prevents the alert from continuously re-triggering
// 0 (false) = System is ready to detect
// 1 (true) = Alert has been triggered and is waiting for the sensor value to drop below threshold
bool alert_active = false; 

// Function to read ADC using registers
int readADC() {
	// Select ADC0 channel is already set in setup (ADMUX = (1 << REFS0))
	ADCSRA |= (1 << ADSC);  // Start ADC conversion
	while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
	return ADC;  // Return 10-bit ADC value (0-1023)
}

// Function to transmit data via UART using registers
void serialWrite(char c) {
	while (!(UCSR0A & (1 << UDRE0))); // Wait for empty transmit buffer
	UDR0 = c; // Put data into buffer, sends the data
}

void serialPrint(const char* str) {
	while (*str) {
		serialWrite(*str++);
	}
}

void serialPrintln(int value) {
	char buffer[10];
	itoa(value, buffer, 10);
	serialPrint(buffer);
	serialWrite('\r');
	serialWrite('\n');
}

void setup() {
	// Configure ADC pin (PF0/A0) as input
	DDRF &= ~(1 << DDF0);  // Clear bit 0 in DDRF (set as input)
	
	// Configure LED pin (PH4/Pin 7) as output
	DDRH |= (1 << DDH4);   // Set PH4 as output
	
	// Configure Buzzer pin (PH5/Pin 8) as output
	DDRH |= (1 << DDH5);   // Set PH5 as output
	
	// Initialize outputs to LOW (LED and Buzzer OFF initially)
	PORTH &= ~(1 << PORTH4); // LED OFF
	PORTH &= ~(1 << PORTH5); // Buzzer OFF
	
	// --- STARTUP LED DIAGNOSTIC TEST ---
	for (int i = 0; i < 2; i++) {
		PORTH |= (1 << PORTH4);  // LED ON
		_delay_ms(100);
		PORTH &= ~(1 << PORTH4); // LED OFF
		_delay_ms(100);
	}
    // Ensure LED is OFF after the test to meet the Active Low initial state
    PORTH &= ~(1 << PORTH4); 
	// --- END DIAGNOSTIC TEST ---
	
	// Configure ADC
	ADMUX = (1 << REFS0);  // AVcc as reference voltage, ADC0 channel selected
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// Enable ADC, prescaler = 128 (16MHz/128 = 125kHz)
	
	// Initialize Serial Communication using registers for 115200 baud
	
	// Set Double Speed Mode (U2X0 = 1) for higher accuracy at 115200
	UCSR0A |= (1 << U2X0); 
	
	// Set UBRR = 8 for 115200 baud at 16MHz in U2X mode
	UBRR0H = 0;
	UBRR0L = 8;
	
	// Enable transmitter
	UCSR0B = (1 << TXEN0);
	
	// Set frame format: 8 data bits, 1 stop bit, no parity
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

int main(void) {
	setup();
	
	while(1) {
		// Read alcohol concentration from MQ3 sensor
		int sensorValue = readADC();
		
		// Print sensor value to Serial Monitor
		serialPrintln(sensorValue);
		
		// Check if sensor value exceeds threshold AND if an alert is NOT currently active
		if (sensorValue > threshold && !alert_active) {
			
			// --- TIMED ALERT STATE: Run for 3 seconds ---
			
			// 1. Set flag to true to prevent immediate re-triggering
			alert_active = true;
			
			// 2. Activate LED and Buzzer
			PORTH |= (1 << PORTH4); // Turn ON LED (set PH4 HIGH)
			PORTH |= (1 << PORTH5); // Turn ON Buzzer (set PH5 HIGH)
			
			serialPrint("!!! ALCOHOL ALERT TRIGGERED (3s) !!!");
			serialWrite('\r');
			serialWrite('\n');

			// 3. Wait for exactly 3 seconds (This is a blocking delay)
			_delay_ms(3000); 

			// 4. Terminate Alert (Turn both off)
			PORTH &= ~(1 << PORTH4); // Turn OFF LED
			PORTH &= ~(1 << PORTH5); // Turn OFF Buzzer
			
			serialPrint("Alert outputs terminated. Waiting for concentration to drop below threshold to re-arm.");
			serialWrite('\r');
			serialWrite('\n');
			
			// Note: alert_active remains TRUE here until the else block is reached.
			
		} else if (sensorValue <= threshold && alert_active) {
			
			// --- SYSTEM RE-ARMED STATE: Sensor value dropped below threshold ---
			
			// 1. Reset the alert flag, making the system ready for the next detection
			alert_active = false;
			
			serialPrint("Concentration is normal. System re-armed.");
			serialWrite('\r');
			serialWrite('\n');
			
		} else {
			// NO ALCOHOL DETECTED (or still above threshold but alert already ran): Normal monitoring state
			
			// Ensure outputs are OFF (only relevant if the alert just terminated)
			PORTH &= ~(1 << PORTH4); // LED OFF
			PORTH &= ~(1 << PORTH5); // Buzzer OFF
			
			if (!alert_active) {
			    serialPrint("System Armed. Concentration Normal.");
			} else {
			    serialPrint("Concentration high, but alert is paused (waiting for clearance).");
			}
			serialWrite('\r');
			serialWrite('\n');
		}
		
		// Wait 500ms before next full check (Keeps the system responsive)
		_delay_ms(500);
	}
	
	return 0;
}
