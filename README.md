
Real-Time Alcohol Detection System (AVR Microcontroller)

This project implements an alcohol detection system using an AVR microcontroller (like the ATmega2560 found on an Arduino Mega) and an MQ-3 alcohol sensor. It provides a non-stop monitoring loop that triggers a 3-second visual (LED) and auditory (Buzzer) alarm upon detection. The system will only trigger the alarm once per detection event and will re-arm itself only after the alcohol concentration has dropped below the set threshold.

⚙️ Hardware & Components

Microcontroller: Arduino Mega 2560 (or any AVR running at 16MHz)

Sensor: MQ-3 Alcohol Sensor (Analog Output)

Output 1: LED (with a 220Ω - 330Ω current-limiting resistor)

Output 2: Active Buzzer

Power Supply: 5V

📌 Wiring Diagram

Component

Microcontroller Pin

Function

MQ-3 Sensor (Analog Out)

ADC0 (A0/PF0)

Alcohol Concentration Input

LED

Digital Pin 7 (PH4)

Visual Alarm Output

Buzzer

Digital Pin 8 (PH5)

Auditory Alarm Output

🛠️ Code Configuration

Clock Speed: F_CPU is set to 16000000UL (16 MHz).

Baud Rate: Serial communication is configured for 115200 baud.

Threshold: The detection level is set by int threshold = 400;. This value may need calibration based on your specific MQ-3 sensor and environment.

AVR Registers: The code uses direct register manipulation (e.g., DDRH, PORTH, ADCSRA) for high performance.

💻 How to Compile

IDE: Use Atmel Studio, Microchip Studio, or a similar AVR development environment.

Project: Create a new AVR C Executable Project targeting the ATmega2560 chip.

Source: Add Alcohol.c to your source files.

Build: Compile the project.

Upload: Flash the generated .hex file onto your Arduino Mega using a programmer (or the Arduino bootloader).

✨ Functionality

Startup Diagnostic: The LED flashes twice briefly upon power-up/reset to confirm pin functionality.

Detection: If readADC() returns a value greater than threshold (400).

Alarm: The LED and Buzzer turn ON simultaneously for 3000ms (3 seconds).

Termination: After 3 seconds, both outputs are immediately turned OFF.

Re-Arming: The system will not trigger the alarm again until the alcohol concentration has dropped below the threshold, resetting the alert_active flag. Once the environment is clear, the system is ready for the next detection event."# Embedded-systems-group-d" 
