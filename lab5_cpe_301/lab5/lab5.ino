/*
  lab5.ino
  Created by: Joshua Knight, Nicky Victoriano
  Date: 10/10/25
  Description: This program reads from the serial port, and plays notes (A-G) based on the keyboard input.
*/

volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1  = (unsigned char *) 0x36;
volatile unsigned char *portDDRB = (unsigned char *) 0x24;
volatile unsigned char *portB    = (unsigned char *) 0x25;

// make sure to always use volatile keyword

void setup() 
{
  Serial.begin(9600);
  // PB6 output
  *portDDRB |= (1 << 6);
}

void loop() 
{
  if (!Serial.available())
  {
    return;
  }

  char c = Serial.read();
  double freq = 0;

  // pick the right frequency
  switch (c) {
    case 'A': case 'a': freq = 440; break;
    case 'B': case 'b': freq = 494; break;
    case 'C': case 'c': freq = 523; break;
    case 'D': case 'd': freq = 587; break;
    case 'E': case 'e': freq = 659; break;
    case 'F': case 'f': freq = 698; break;
    case 'G': case 'g': freq = 784; break;
    case 'q': case 'Q':
      *portB &= ~(1 << 6); // turn off output
      *myTCCR1B &= 0xF8;   // stop timer
      Serial.println("Stopped.");
      return;
    default:
      Serial.println("Invalid key. Use A-G or q.");
      return;
  }

  Serial.print("Playing note ");
  Serial.println(c);

  // keep playing this note until another key is pressed
  while (!Serial.available()) {
    my_delay(freq);
    *portB ^= (1 << 6);  // toggle PB6 every half period
  }
}




void my_delay(unsigned int freq)
{
  // calc full period and half period
  double period = 1.0 / (double)freq;
  double half_period = period / 2.0;

  // prescaler = 8 → 0.5 microseconds per tick
  double clk_period = 8.0 / 16000000.0;

  // figure out how many ticks per half period
  unsigned int ticks = (unsigned int)(half_period / clk_period);

  // make sure we don't overflow
  if (ticks > 65535) ticks = 65535;

  // normal mode, no PWM
  *myTCCR1A = 0x00;

  // preload counter so it overflows after 'ticks' counts
  *myTCNT1 = (unsigned int)(65536 - ticks);

  // clear overflow flag
  *myTIFR1 |= 0x01;

  // start timer (prescaler = 8)
  *myTCCR1B = (1 << CS11);

  // wait for overflow flag
  while((*myTIFR1 & 0x01) == 0);

  // clear the overflow flag again
  *myTIFR1 |= 0x01;
}
