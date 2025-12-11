/*
 * Swamp Cooler Project - Arduino Mega 2560
 * CPE 301 - Embedded Systems Design
 *
 * Monitors water levels, temperature, humidity
 * Controls fan motor and manages LED state indicators
 * Logs all events with RTC timestamps
 */

// LIBRARY INCLUDES
#include "DHT.h"
#include "RTClib.h"
#include "Stepper.h"
#include "LiquidCrystal.h"

// UART REGISTERS (USART0)
volatile unsigned char *myUCSR0A = (volatile unsigned char *) 0xC0;
volatile unsigned char *myUCSR0B = (volatile unsigned char *) 0xC1;
volatile unsigned char *myUCSR0C = (volatile unsigned char *) 0xC2;
volatile unsigned int  *myUBRR0  = (volatile unsigned int *) 0xC4;
volatile unsigned char *myUDR0   = (volatile unsigned char *) 0xC6;

// ADC REGISTERS
volatile unsigned char *my_ADMUX   = (volatile unsigned char *) 0x7C;
volatile unsigned char *my_ADCSRB  = (volatile unsigned char *) 0x7B;
volatile unsigned char *my_ADCSRA  = (volatile unsigned char *) 0x7A;
volatile unsigned int  *my_ADC_DATA = (volatile unsigned int *) 0x78;

// GPIO PORT REGISTERS
// Port A (Water level sensor ADC)
volatile unsigned char *portDDRA = (volatile unsigned char *) 0x21;
volatile unsigned char *portA    = (volatile unsigned char *) 0x22;
volatile unsigned char *pinA     = (volatile unsigned char *) 0x20;

// Port B (LCD RS/EN, Fan Motor)
volatile unsigned char *portDDRB = (volatile unsigned char *) 0x24;
volatile unsigned char *portB    = (volatile unsigned char *) 0x25;
volatile unsigned char *pinB     = (volatile unsigned char *) 0x23;

// Port C (LCD D4-D7)
volatile unsigned char *portDDRC = (volatile unsigned char *) 0x27;
volatile unsigned char *portC    = (volatile unsigned char *) 0x28;
volatile unsigned char *pinC     = (volatile unsigned char *) 0x26;

// Port D (Buttons - Start/Reset)
volatile unsigned char *portDDRD = (volatile unsigned char *) 0x2A;
volatile unsigned char *portD    = (volatile unsigned char *) 0x2B;
volatile unsigned char *pinD     = (volatile unsigned char *) 0x29;

// Port H (STATE LEDs - Green/Blue/Red/Yellow)
volatile unsigned char *portDDRH = (volatile unsigned char *) 0x101;
volatile unsigned char *portH    = (volatile unsigned char *) 0x102;
volatile unsigned char *pinH     = (volatile unsigned char *) 0x100;

// Port J (Stop button)
volatile unsigned char *portDDRJ = (volatile unsigned char *) 0x104;
volatile unsigned char *portJ    = (volatile unsigned char *) 0x105;
volatile unsigned char *pinJ     = (volatile unsigned char *) 0x103;

// Port L (Stepper Motor)
volatile unsigned char *portDDRL = (volatile unsigned char *) 0x10A;
volatile unsigned char *portL    = (volatile unsigned char *) 0x10B;
volatile unsigned char *pinL     = (volatile unsigned char *) 0x109;

// EXTERNAL INTERRUPT REGISTERS
volatile unsigned char *myEICRA = (volatile unsigned char *) 0x69;  // INT0-3 control
volatile unsigned char *myEIMSK = (volatile unsigned char *) 0x3D;  // Interrupt mask
volatile unsigned char *myEIFR  = (volatile unsigned char *) 0x3C;  // Interrupt flags

// UART CONTROL BITS
#define RDA 0x80  // Receive Data Available
#define TBE 0x20  // Transmit Buffer Empty

// STATES
enum SystemState {
  STATE_DISABLED = 0,
  STATE_IDLE     = 1,
  STATE_RUNNING  = 2,
  STATE_ERROR    = 3
};

// GLOBAL STATE
volatile SystemState currentState = STATE_DISABLED;
volatile SystemState previousState = STATE_DISABLED;

// SENSORS & CONTROLS
float currentTemp = 0.0;
float currentHumidity = 0.0;
float tempThreshold = 15.0;  // TODO: configure temperature threshold
unsigned int currentVentPosition = 0;  // 0-100

// TIMING VARIABLES
unsigned long lastTempRead = 0;
unsigned long lastLCDUpdate = 0;
unsigned long lastWaterRead = 0;
unsigned long lastStartPress = 0;
unsigned long lastResetPress = 0;

// ISR FLAGS (for safe communication between ISR and main loop)
volatile unsigned char startButtonPressed = 0;
volatile unsigned char resetButtonPressed = 0;

// RTC STATUS
volatile unsigned char rtcAvailable = 0;  // Will be set to 1 if RTC initializes successfully

// TIMING CONSTANTS
const unsigned long TEMP_INTERVAL = 60000;      // 60 seconds
const unsigned long LCD_INTERVAL = 60000;       // 60 seconds
const unsigned long WATER_INTERVAL = 100;       // 100 milliseconds
const unsigned long DEBOUNCE_DELAY = 50;        // debounce delay 50 milliseconds

// DEBUG FLAG
#define DEBUG 0

// THRESHOLD VALUES
#define WATER_THRESHOLD 300  // TODO: configure ADC threshold

// PIN defs
#define WATER_SENSOR_PIN 22  // Pin 22 (PA0, ADC0)
#define DHT_PIN 24           // Pin 24 (PA2)
#define FAN_PIN 50           // Pin 50 (PB3)
#define START_BUTTON_PIN 18  // Pin 18 (PD3, INT3)
#define RESET_BUTTON_PIN 19  // Pin 19 (PD2, INT2)
#define STOP_BUTTON_PIN 14   // Pin 14 (PJ1)

// LED STATUS pin defs
#define GREEN_LED_PIN 6      // Pin 6 (PH3)
#define BLUE_LED_PIN 7       // Pin 7 (PH4)
#define RED_LED_PIN 8        // Pin 8 (PH5)
#define YELLOW_LED_PIN 9     // Pin 9 (PH6)

// LCD pin defs
#define LCD_RS 11            // Pin 11 (PB5)
#define LCD_EN 12            // Pin 12 (PB6)
#define LCD_D4 34            // Pin 34 (PC3)
#define LCD_D5 35            // Pin 35 (PC2)
#define LCD_D6 36            // Pin 36 (PC1)
#define LCD_D7 37            // Pin 37 (PC0)

// STEPPER MOTOR pin defs
// Try sequence: 44, 47, 46, 45 (reverse of sequential)
#define STEPPER_PIN_1 46    // Pin 44 (PL2)
#define STEPPER_PIN_2 48    // Pin 47 (PL5)
#define STEPPER_PIN_3 45    // Pin 46 (PL4)
#define STEPPER_PIN_4 47    // Pin 45 (PL3)

// LIBRARY OBJECTS
DHT dht(DHT_PIN, DHT11);
RTC_DS1307 rtc;
Stepper stepper(2048, STEPPER_PIN_1, STEPPER_PIN_2, STEPPER_PIN_3, STEPPER_PIN_4);
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);


// FUNCTION DEFS
void U0init(unsigned int U0baud);
void U0putchar(unsigned char U0pdata);
void U0puts(const char* s);
unsigned char U0kbhit();
unsigned char U0getchar();
unsigned char bitsToHex(unsigned char bits);

void adc_init();
unsigned int adc_read(unsigned char adc_channel_num);

void gpio_init();
void changeState(SystemState newState);
void updateStateLEDs();
void checkWaterLevel();
void readTemperatureHumidity();
void updateLCD();
void logTimestamp(const char* event);
void startFan();
void stopFan();
void checkStopButton();
void handleStepperControl();
void adjustVent(int targetPosition);
const char* getStateName(SystemState state);
void runHardwareTests();  // Comprehensive hardware test suite

// ISR FUNCTION DEFS
void startButtonISR();
void resetButtonISR();

// ====================== SETUP ======================
void setup() {
  //GPIO
  gpio_init();

  // UART (9600 baud)
  U0init(9600);
  U0puts("\n=== Swamp Cooler System ===\n");

  // ADC
  adc_init();

  // RTC - with diagnostic info
  U0puts("RTC Initialization...\n");
  if (!rtc.begin()) {
    U0puts("ERROR: RTC not found on I2C bus!\n");
    U0puts("Troubleshooting:\n");
    U0puts("  1. Check RTC module is connected to I2C (Pins 20=SDA, 21=SCL)\n");
    U0puts("  2. Verify +5V and GND connections to RTC module\n");
    U0puts("  3. Check for 4.7k pullup resistors on SDA/SCL lines\n");
    U0puts("  4. RTC address should be 0x68 (DS1307)\n");
    U0puts("  5. Sensor may be defective\n");
    U0puts("WARNING: Continuing without RTC - timestamps will not work\n\n");
    rtcAvailable = 0;
  } else {
    rtcAvailable = 1;
    if (!rtc.isrunning()) {
      U0puts("RTC not running, setting time\n");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    } else {
      U0puts("RTC initialized successfully\n");
    }
  }

  // DHT11
  dht.begin();
  delay(2000);  // Give DHT11 time to stabilize after power-on

  // LCD
  lcd.begin(16, 2);
  delay(100);
  lcd.clear();
  delay(100);
  lcd.print("Swamp Cooler");
  delay(100);
  lcd.setCursor(0, 1);
  delay(100);
  lcd.print("Initializing...");
  delay(500);

  // Stepper motor
  stepper.setSpeed(15);  // 15 RPM - faster to minimize interrupt interference

  // Run comprehensive hardware test suite
  if (DEBUG)
  {
    runHardwareTests();
  }
  
  // Button interrupts - configure registers directly for claritu
  *myEICRA |= 0xA0;   // 0b10100000 - INT2 and INT3 on fall

  // EIMSK: Enable INT2 (bit 2) and INT3 (bit 3)
  *myEIMSK |= 0x0C;   // 0b00001100

  // Clear any pending interrupt flags
  *myEIFR |= 0x0C;    // Write 1 to clear INT2 and INT3 flags

  // Attach ISRs (these should now work correctly)
  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), startButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RESET_BUTTON_PIN), resetButtonISR, FALLING);

  // Enable global interrupts (CRITICAL!)
  sei();

  // Read initial temperature for immediate state transition on startup
  readTemperatureHumidity();

  // Set initial state
  currentState = STATE_DISABLED;
  updateStateLEDs();
  
  // Display ready message
  lcd.clear();
  lcd.print("System Ready");
  lcd.setCursor(0, 1);
  lcd.print("Press START");
  
  logTimestamp("System initialized");
  U0puts("Setup complete. Waiting for Start button...\n");
}

static unsigned long lastDebugTime = 0;

// ====================== LOOP ======================
void loop() {
  unsigned long now = millis();

  // Priority 1: Water level check (every 100ms) - ERROR takes precedence
  if (currentState != STATE_DISABLED) {
    if (now - lastWaterRead >= WATER_INTERVAL) {
      lastWaterRead = now;
      checkWaterLevel();
    }
  }

  // Priority 2: Temperature/humidity check (every 60 seconds)
  if ((currentState == STATE_IDLE || currentState == STATE_RUNNING) &&
      (now - lastTempRead >= TEMP_INTERVAL)) {
    lastTempRead = now;
    readTemperatureHumidity();
  }

  // Priority 3: LCD update (every 60 seconds)
  if (currentState != STATE_DISABLED && (now - lastLCDUpdate >= LCD_INTERVAL)) {
    lastLCDUpdate = now;
    updateLCD();
  }

  // Priority 4: Stop button check (polled with debounce)
  checkStopButton();

  // Priority 5: Handle Start button press (from ISR)
  if (startButtonPressed) {
    startButtonPressed = 0;  // Clear flag
    if (now - lastStartPress >= DEBOUNCE_DELAY) {
      lastStartPress = now;
      if (currentState == STATE_DISABLED) {
        changeState(STATE_IDLE);
      }
    }
  }

  // Priority 6: Handle Reset button press (from ISR)
  if (resetButtonPressed) {
    resetButtonPressed = 0;  // Clear flag
    if (now - lastResetPress >= DEBOUNCE_DELAY) {
      lastResetPress = now;
      if (currentState == STATE_ERROR) {
        unsigned int waterLevel = adc_read(0);
        if (waterLevel >= WATER_THRESHOLD) {
          changeState(STATE_IDLE);
        } else {
          U0puts("ERROR: Water level still too low!\n");
        }
      }
    }
  }

  // Priority 7: Stepper/vent control
  if (currentState != STATE_DISABLED) {
    handleStepperControl();  // Guard inside function prevents repeated calls
  }

  // DEBUG: Print button states every 2 seconds
  if (millis() - lastDebugTime >= 5000 && DEBUG) {
    lastDebugTime = millis();

    unsigned char startState = (*pinD >> 3) & 0x01;  // PD3
    unsigned char resetState = (*pinD >> 2) & 0x01;  // PD2
    unsigned char stopState = (*pinJ >> 1) & 0x01;

    char debugMsg[150];
    sprintf(debugMsg, "Buttons - Stop:%d Start:%d Reset:%d", stopState, startState, resetState);
    U0puts(debugMsg);
    U0putchar('\n');

    char stateMsg[50];
    sprintf(stateMsg, "Current State: %s\n", getStateName(currentState));
    U0puts(stateMsg);

    // Print sensor data (manual formatting since sprintf doesn't support %f)
    U0puts("Temp: ");
    int tempInt = (int)currentTemp;
    int tempDec = (int)((currentTemp - tempInt) * 10);
    if (tempInt < 10) U0putchar('0');
    if (tempInt >= 10) U0putchar('0' + (tempInt / 10));
    U0putchar('0' + (tempInt % 10));
    U0puts(".");
    U0putchar('0' + tempDec);
    U0puts("C  Humidity: ");
    int humidInt = (int)currentHumidity;
    int humidDec = (int)((currentHumidity - humidInt) * 10);
    if (humidInt < 10) U0putchar('0');
    if (humidInt >= 10) U0putchar('0' + (humidInt / 10));
    U0putchar('0' + (humidInt % 10));
    U0puts(".");
    U0putchar('0' + humidDec);

    char sensorMsg[50];
    sprintf(sensorMsg, "%%  Water ADC: %d  Vent: %d%%\n",
            adc_read(0), currentVentPosition);
    U0puts(sensorMsg);
  }
}

// UART FUNCTIONS (FROM LAB 8)
void U0init(unsigned int U0baud) {
  unsigned long FCPU = 16000000UL;
  unsigned int tbaud = (FCPU / 16UL / U0baud - 1UL);
  *myUCSR0A = 0x20;  // U2X0 = 0
  *myUCSR0B = 0x18;  // RXEN0=1, TXEN0=1
  *myUCSR0C = 0x06;  // 8-bit
  *myUBRR0  = tbaud;
}

unsigned char U0kbhit() {
  return (*myUCSR0A & RDA) ? 1 : 0;
}

unsigned char U0getchar() {
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata) {
  while (((*myUCSR0A) & TBE) == 0);
  *myUDR0 = U0pdata;
}

void U0puts(const char* s) {
  while(*s) U0putchar(*s++);
}

unsigned char bitsToHex(unsigned char bits) {
  if (bits < 10)
    return '0' + bits;
  else
    return 'A' + (bits - 10);
}

// ADC FUNCTIONS (FROM LAB 8)
void adc_init() {
  // Enable ADC, no auto-trigger, no interrupt, prescaler = 128
  *my_ADCSRA |= 0x80;   // ADEN = 1
  *my_ADCSRA &= 0xDF;   // ADATE = 0
  *my_ADCSRA &= 0xF7;   // ADIE = 0
  *my_ADCSRA |= 0x07;   // Prescaler = 128

  // Free-running mode disabled
  *my_ADCSRB &= 0xF8;   // ADTS2:0 = 000
  *my_ADCSRB &= 0xF7;   // MUX5 = 0

  // AVCC reference, right-adjust, channel 0
  *my_ADMUX &= 0x7F;    // REFS1 = 0
  *my_ADMUX |= 0x40;    // REFS0 = 1 (AVCC with cap)
  *my_ADMUX &= 0xDF;    // ADLAR = 0 (right adjust)
  *my_ADMUX &= 0xF0;    // Channel = 0
}

unsigned int adc_read(unsigned char adc_channel_num) {
  // Clear and set channel selection bits
  *my_ADMUX &= 0xE0;
  *my_ADCSRB &= 0xF7;
  *my_ADMUX |= (adc_channel_num & 0x07);

  // Take multiple samples and average to filter noise
  unsigned long sum = 0;
  const int numSamples = 8;  // Average 8 readings

  for (int i = 0; i < numSamples; i++) {
    // Start conversion
    *my_ADCSRA |= 0x40;

    // Wait for completion
    while((*my_ADCSRA & 0x40) != 0);

    // Accumulate result
    sum += (*my_ADC_DATA & 0x03FF);
  }

  // Return averaged 10-bit result
  return sum / numSamples;
}

// GPIO INITIALIZATION
void gpio_init() {
  // PORT A: Water sensor (PA0) - input for ADC
  *portDDRA &= 0xFE;   // 0b11111110 - PA0 as input
  *portA &= 0xFE;      // 0b11111110 - no pullup for ADC input

  // PORT B: LCD RS (PB5), LCD EN (PB6), Fan (PB3) - outputs
  *portDDRB |= 0x68;   // 0b01101000
  *portB &= 0x97;      // 0b10010111 -> all low initially

  // PORT C: LCD D4-D7 (PC0-PC3) - outputs
  *portDDRC |= 0x0F;   // 0b00001111
  *portC &= 0xF0;      // 0b11110000 -> all low

  // PORT D: Start (PD3), Reset (PD2) buttons - inputs with pullups
  *portDDRD &= 0xF3;   // 0b11110011
  *portD |= 0x0C;      // 0b00001100 -> enable pullups

  // PORT H: ALL 4 LEDs - outputs
  *portDDRH |= 0x78;   // 0b01111000
  *portH &= 0x87;      // 0b10000111 -> all low
  *portH |= 0x40;      // Yellow LED ON initially (PH6)

  // PORT J: Stop button (PJ1) - input with pullup
  *portDDRJ &= 0xFD;   // 0b11111101
  *portJ |= 0x02;      // 0b00000010 -> enable pullup

  // PORT L: Stepper motor (PL2-PL5) - outputs
  *portDDRL |= 0x3C;   // 0b00111100
  *portL &= 0xC3;      // 0b11000011 -> all low
}

void changeState(SystemState newState) {
  if (newState == currentState) return;

  previousState = currentState;
  currentState = newState;

  char logMsg[50];
  sprintf(logMsg, "State changed from %s to %s",
          getStateName(previousState), getStateName(currentState));

  logTimestamp(logMsg);
  updateStateLEDs();

  // Handle state-specific actions
  switch(currentState) {
    case STATE_DISABLED:
      stopFan();
      break;
    case STATE_IDLE:
      stopFan();
      break;
    case STATE_RUNNING:
      startFan();
      break;
    case STATE_ERROR:
      stopFan();
      break;
  }
}

void updateStateLEDs() {
  *portH &= 0x87;  // Turn off all LEDs

  switch(currentState) {
    case STATE_DISABLED:
      *portH |= 0x40;  // Yellow (PH6)
      break;
    case STATE_IDLE:
      *portH |= 0x08;  // Green (PH3)
      break;
    case STATE_RUNNING:
      *portH |= 0x10;  // Blue (PH4)
      break;
    case STATE_ERROR:
      *portH |= 0x20;  // Red (PH5)
      break;
  }
}

void checkWaterLevel() {
  unsigned int waterLevel = adc_read(0);

  // Go to ERROR if water drops below threshold
  if (waterLevel < WATER_THRESHOLD && currentState != STATE_ERROR) {
    char logMsg[50];
    sprintf(logMsg, "ERROR: Water level low (ADC=%d < %d)", waterLevel, WATER_THRESHOLD);
    logTimestamp(logMsg);
    changeState(STATE_ERROR);
  }

  // Recover from ERROR if water level rises above threshold
  if (waterLevel >= WATER_THRESHOLD && currentState == STATE_ERROR) {
    char logMsg[50];
    sprintf(logMsg, "Water level recovered (ADC=%d >= %d)", waterLevel, WATER_THRESHOLD);
    logTimestamp(logMsg);
    changeState(STATE_IDLE);
  }
}

void readTemperatureHumidity() {
  currentTemp = dht.readTemperature();
  currentHumidity = dht.readHumidity();

  // Debug: print raw sensor values
  static unsigned long lastDhtDebug = 0;
  if (millis() - lastDhtDebug > 10000 && DEBUG) {  // Print every 10 seconds
    lastDhtDebug = millis();
    U0puts("DHT DEBUG: Temp raw value: ");
    U0putchar(isnan(currentTemp) ? 'N' : 'V');  // N=NaN, V=Valid
    U0puts(" Humidity raw value: ");
    U0putchar(isnan(currentHumidity) ? 'N' : 'V');
    U0puts("\n");
  }

  if (isnan(currentTemp) || isnan(currentHumidity)) {
    // If we get NaN, try reading again
    delay(500);
    currentTemp = dht.readTemperature();
    currentHumidity = dht.readHumidity();
    if (isnan(currentTemp) || isnan(currentHumidity)) {
      return;  // Still failed, give up this cycle
    }
  }

  if (currentState == STATE_IDLE && currentTemp > tempThreshold) {
    changeState(STATE_RUNNING);
  } else if (currentState == STATE_RUNNING && currentTemp <= tempThreshold) {
    changeState(STATE_IDLE);
  }
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (currentState == STATE_DISABLED) {
    lcd.print("System Disabled");
    return;
  }

  if (currentState == STATE_ERROR) {
    lcd.print("ERROR: Water Low");
    return;
  }

  lcd.print("T:");
  lcd.print(currentTemp, 1);
  lcd.print("C H:");
  lcd.print(currentHumidity, 0);
  lcd.print("%");

  lcd.setCursor(0, 1);
  if (rtcAvailable) {
    DateTime now = rtc.now();
    char timeStr[9];
    sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    lcd.print(timeStr);
  } else {
    lcd.print("NO TIME");
  }
}

void logTimestamp(const char* event) {
  if (rtcAvailable) {
    DateTime now = rtc.now();
    char buffer[100];
    sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d - %s\n",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second(), event);
    U0puts(buffer);
  } else {
    // RTC not available, just log without timestamp
    U0puts("[NO TIME] - ");
    U0puts(event);
    U0puts("\n");
  }
}

void startFan() {
  for (int i = 0; i <= 255; i += 5) {
    analogWrite(FAN_PIN, i);
    delay(10);  // Smooth ramp-up
  }
  logTimestamp("Fan started");
}

void stopFan() {
  for (int i = 255; i >= 0; i -= 5) {
    analogWrite(FAN_PIN, i);
    delay(10);  // Smooth ramp-down
  }
  logTimestamp("Fan stopped");
}

void checkStopButton() {
  static unsigned char lastStopState = 1;
  static unsigned char stableStopState = 1;
  static unsigned long lastStopChange = 0;

  unsigned char currentStopState = (*pinJ >> 1) & 0x01;

  // Detect state change and start debounce timer
  if (currentStopState != lastStopState) {
    lastStopChange = millis();
    lastStopState = currentStopState;
    return;  // Wait for debounce period
  }

  // After debounce period, check if stable state actually changed
  if ((millis() - lastStopChange) > DEBOUNCE_DELAY) {
    if (currentStopState != stableStopState) {
      stableStopState = currentStopState;

      // Handle button press (transition from 1 to 0)
      if (currentStopState == 0 && currentState != STATE_DISABLED) {
        changeState(STATE_DISABLED);
      }
    }
  }
}

int lastDesiredVentPosition = -1;  // Track target to avoid repeated commands

void handleStepperControl() {
  // Only adjust vent if position has changed (avoid repeated stepper.step calls)
  int targetPosition = (currentState == STATE_RUNNING) ? 512 : 0;  // 90 degrees (512 steps)

  if (targetPosition != lastDesiredVentPosition) {
    adjustVent(targetPosition);
    lastDesiredVentPosition = targetPosition;
  }
}

void adjustVent(int targetPosition) {
  if (currentState == STATE_DISABLED) return;

  int delta = targetPosition - currentVentPosition;

  // Only move stepper if there's an actual change needed
  if (delta == 0) return;

  int steps = (delta * 2048) / 100;
  stepper.step(steps);
  currentVentPosition = targetPosition;

  char msg[40];
  sprintf(msg, "Vent: %d%%", currentVentPosition);
  logTimestamp(msg);
}

const char* getStateName(SystemState state) {
  switch(state) {
    case STATE_DISABLED: return "DISABLED";
    case STATE_IDLE:     return "IDLE";
    case STATE_RUNNING:  return "RUNNING";
    case STATE_ERROR:    return "ERROR";
    default:             return "UNKNOWN";
  }
}

// ISR
void startButtonISR() {
  startButtonPressed = 1;
  U0puts("Start button ISR triggered\n");
}

void resetButtonISR() {
  resetButtonPressed = 1;
  U0puts("Reset button ISR triggered\n");
}

// HARDWARE TEST SUITE
void runHardwareTests() {
  U0puts("\n\n");
  U0puts("====================================\n");
  U0puts("  SWAMP COOLER HARDWARE TEST SUITE\n");
  U0puts("====================================\n\n");

  // TEST 1: LED TEST
  U0puts("TEST 1: LED INDICATORS\n");
  U0puts("  Testing Yellow LED (Pin 9, PH6)...");
  *portH |= 0x40;  // Yellow ON
  delay(500);
  *portH &= ~0x40;  // Yellow OFF
  U0puts(" [OK]\n");

  U0puts("  Testing Green LED (Pin 6, PH3)...");
  *portH |= 0x08;  // Green ON
  delay(500);
  *portH &= ~0x08;  // Green OFF
  U0puts(" [OK]\n");

  U0puts("  Testing Blue LED (Pin 7, PH4)...");
  *portH |= 0x10;  // Blue ON
  delay(500);
  *portH &= ~0x10;  // Blue OFF
  U0puts(" [OK]\n");

  U0puts("  Testing Red LED (Pin 8, PH5)...");
  *portH |= 0x20;  // Red ON
  delay(500);
  *portH &= ~0x20;  // Red OFF
  U0puts(" [OK]\n");
  U0puts("  [PASS] All LEDs working\n\n");

  // TEST 2: BUTTON TEST
  U0puts("TEST 2: BUTTON INPUT\n");
  U0puts("  Reading button states (should be LOW with active-HIGH wiring):\n");
  unsigned char startBtn = (*pinD >> 3) & 0x01;  // PD3
  unsigned char resetBtn = (*pinD >> 2) & 0x01;  // PD2
  unsigned char stopBtn = (*pinJ >> 1) & 0x01;   // PJ1
  U0puts("    Start Button (Pin 18): ");
  U0putchar('0' + startBtn);
  U0puts(!startBtn ? " [OK]\n" : " [FAIL - Check wiring]\n");
  U0puts("    Reset Button (Pin 19): ");
  U0putchar('0' + resetBtn);
  U0puts(!resetBtn ? " [OK]\n" : " [FAIL - Check wiring]\n");
  U0puts("    Stop Button (Pin 14): ");
  U0putchar('0' + stopBtn);
  U0puts(!stopBtn ? " [OK]\n" : " [FAIL - Check wiring]\n\n");

  // TEST 3: ADC TEST (Water Sensor)
  U0puts("TEST 3: ADC WATER SENSOR (Pin 22/A0)\n");
  U0puts("  Reading ADC 5 times (should show similar values):\n");
  for (int i = 0; i < 5; i++) {
    unsigned int adc_val = adc_read(0);
    U0puts("    Read ");
    U0putchar('0' + i + 1);
    U0puts(": ");
    char adc_str[20];
    sprintf(adc_str, "%d\n", adc_val);
    U0puts(adc_str);
    delay(200);
  }
  U0puts("  [PASS] ADC working\n\n");

  // TEST 4: DHT11 TEST
  U0puts("TEST 4: DHT11 TEMP/HUMIDITY SENSOR (Pin 24/PA2)\n");
  float temp = NAN;
  float humid = NAN;
  unsigned char dhtPassed = 0;

  for (int attempt = 1; attempt <= 5; attempt++) {
    delay(2000);
    temp = dht.readTemperature();
    humid = dht.readHumidity();

    U0puts("    Attempt ");
    U0putchar('0' + attempt);
    U0puts(": ");

    if (!isnan(temp) && !isnan(humid)) {
      // Print temp
      int tempInt = (int)temp;
      int tempDec = (int)((temp - tempInt) * 10);
      if (tempInt < 10) U0putchar('0');
      U0putchar('0' + tempInt);
      U0puts(".");
      U0putchar('0' + tempDec);
      U0puts("C  ");

      // Print humidity
      int humidInt = (int)humid;
      int humidDec = (int)((humid - humidInt) * 10);
      if (humidInt < 10) U0putchar('0');
      U0putchar('0' + humidInt);
      U0puts(".");
      U0putchar('0' + humidDec);
      U0puts("%  [SUCCESS]\n");
      dhtPassed = 1;
      break;
    } else {
      U0puts("NaN (retrying...)\n");
    }
  }

  if (!dhtPassed) {
    U0puts("  [FAIL] DHT11 not responding after 5 attempts\n");
    U0puts("         Check Pin 24 wiring and 10k pullup resistor\n");
  } else {
    U0puts("  [PASS] DHT11 working\n");
  }
  U0puts("\n");

  // TEST 5: LCD TEST
  U0puts("TEST 5: LCD DISPLAY (Pins 11,12,34-37)\n");
  U0puts("  Clearing LCD and writing test pattern...\n");
  lcd.clear();
  delay(100);
  lcd.setCursor(0, 0);
  lcd.print("TEST:OK?");
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("Line2:OK?");
  delay(2000);
  U0puts("  [CHECK] Does LCD show 'TEST:OK?' on line 1?\n");
  U0puts("          and 'Line2:OK?' on line 2?\n");
  U0puts("  (If blank, adjust contrast pot on LCD module)\n");
  U0puts("  Waiting 5 seconds...\n");
  for (int i = 5; i > 0; i--) {
    delay(1000);
    U0putchar('0' + i);
    U0puts("...");
  }
  U0puts("\n\n");

  // TEST 6: STEPPER MOTOR TEST
  U0puts("TEST 6: STEPPER MOTOR (Pins 44,46,45,47)\n");
  U0puts("  Rotating stepper motor forward 1 full turn...\n");
  U0puts("  [LISTEN] Can you hear/feel the motor moving?\n");
  stepper.step(2048);  // Full rotation
  delay(1000);
  U0puts("  Rotating stepper motor backward 1 full turn...\n");
  stepper.step(-2048);
  delay(1000);
  U0puts("  [PASS] Stepper motor test complete\n\n");

  // TEST 7: FAN MOTOR TEST
  U0puts("TEST 7: FAN MOTOR (Pin 50, PWM)\n");
  U0puts("  Turning fan ON at full power...\n");
  analogWrite(FAN_PIN, 255);
  delay(2000);
  U0puts("  [LISTEN] Can you hear the fan running?\n");
  U0puts("  Turning fan OFF...\n");
  analogWrite(FAN_PIN, 0);
  U0puts("  [PASS] Fan motor test complete\n\n");

  U0puts("====================================\n");
  U0puts("  TEST SUITE COMPLETE\n");
  U0puts("  System starting in 3 seconds...\n");
  U0puts("====================================\n\n");
  delay(3000);
}
