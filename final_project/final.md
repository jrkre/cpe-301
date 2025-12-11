# CPE 301 Final Project

This is the final project for CPE 301 - Embedded Systems Design. The project involves creating a temperature-controlled fan system with an LCD display,buttons for user interaction, and a stepper motor for airflow adjustment. The system uses various sensors and components to monitor and control the environment effectively.

## Components Used

- DHT11 Temperature and Humidity Sensor
- DS3231 Real-Time Clock Module
- 16x2 LCD Display
- Stepper Motor
- Push Buttons (Start, Stop, Reset)
- LEDs (Red, Green, Yellow, Blue)
- Fan Motor
- Transistor for Fan Control

## GPIO Initialization

The GPIO pins are initialized to set the appropriate modes (input/output) for each connected component. Below is a summary of the GPIO configuration:

- **PORT B**: LCD RS (PB5), LCD EN (PB6), Fan (PB3) - outputs
- **PORT C**: LCD D4-D7 (PC0-PC3) - outputs
- **PORT D**: Start (PD3), Reset (PD2) buttons - inputs
- **PORT H**: ALL 4 LEDs - outputs
- **PORT J**: Stop button (PJ1) - input
- **PORT L**: Stepper motor (PL2-PL5) - outputs

## 