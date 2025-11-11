/* Lab09 - Timer1 overflow ISR tone generator (ATmega2560)
 * Authors: Jay Knight, Nicky Victoriano
 * Uses: UART (manual registers) + Timer1 Normal mode + ISR(TIMER1_OVF_vect)
 *
 * Press A..G to play notes. Press Q to stop.
 */

#define RDA (1 << 7)
#define TBE (1 << 5)

// UART Pointers (USART0)
volatile unsigned char *myUCSR0A  = (volatile unsigned char *) 0xC0;
volatile unsigned char *myUCSR0B  = (volatile unsigned char *) 0xC1;
volatile unsigned char *myUCSR0C  = (volatile unsigned char *) 0xC2;
volatile unsigned int  *myUBRR0   = (volatile unsigned int *) 0xC4;
volatile unsigned char *myUDR0    = (volatile unsigned char *) 0xC6;

// GPIO Pointers (Port B)
volatile unsigned char *portB     = (volatile unsigned char *) 0x25;
volatile unsigned char *portDDRB  = (volatile unsigned char *) 0x24;

// Timer1 Pointers (16-bit)
volatile unsigned char *myTCCR1A  = (volatile unsigned char *) 0x80;
volatile unsigned char *myTCCR1B  = (volatile unsigned char *) 0x81;
volatile unsigned char *myTCCR1C  = (volatile unsigned char *) 0x82;
volatile unsigned char *myTIMSK1  = (volatile unsigned char *) 0x6F;
volatile unsigned char *myTIFR1   = (volatile unsigned char *) 0x36;
volatile unsigned int  *myTCNT1   = (volatile unsigned int *) 0x84;

// UART prototypes
void U0init(int baud);
unsigned char U0kbhit();
unsigned char U0getchar();
void U0putchar(unsigned char data);

byte in_char;

// ticks[] = number of timer ticks for half-period (counts) for prescaler = 1
// counts = F_CPU / (2 * prescaler * freq)
unsigned int ticks[12]= {18181, 17167, 16194, 15296, 14440, 13628, 12820, 12139, 11461, 10810, 10204};

unsigned char input[12]= {'a', 'A', 'b', 'c', 'C', 'd', 'D', 'e', 'f', 'F', 'g', 'G'};
// state
unsigned int currentCounts = 65535;
unsigned char timer_running = 0;

void setup()
{
  // PB6 output (Arduino Mega pin 12)
  *portDDRB |= 0x40;
  *portB &= 0xBF; // drive low initially

  // Timer1 -> Normal mode, interrupts enabled (overflow)
  *myTCCR1A = 0x00;
  *myTCCR1B = 0x00;
  *myTCCR1C = 0x00;

  // Clear pending overflow flag and enable Timer1 overflow interrupt (TOIE1)
  *myTIFR1 |= 0x1;   // write 1 to clear TOV1
  *myTIMSK1 |= 0x1;  // enable TOIE1

  // UART
  U0init(9600);
  U0putchar('\n');
  U0putchar('>');
  // enable global interrupts
  sei();
}

void loop()
{
  if (U0kbhit())
  {
    in_char = U0getchar();
    U0putchar(in_char); // echo

    if (in_char == 'q' || in_char == 'Q')
    {
      // Stop tone
      currentCounts = 65535;
      if (timer_running)
      {
        *myTCCR1B &=0xF8; // clear CS12..CS10 -> stop timer
        timer_running = 0;
      }
      *portB &= 0xBF; // ensure output low
      // U0putchar('\n'); U0putchar('S'); U0putchar('T'); U0putchar('O'); U0putchar('P'); U0putchar('\n');
    }
    else
    {
      // lookup key
      for (int i = 0; i < 7; i++)
      {
        if (in_char == input[i])
        {
          currentCounts = ticks[i];

          // load counter now so first interval is correct
          *myTCNT1 = (unsigned int)(65536 - (unsigned long)currentCounts);

          if (!timer_running)
          {
            // start the timer
            *myTCCR1B |= 0x01;
            // set the running flag
            timer_running = 1;
          }

          U0putchar('\n'); U0putchar('O'); U0putchar('N'); U0putchar('\n');
          break;
        }
      }
    }
  }
}

// Timer1 Overflow ISR - handles toggling PB6 and reloading timer
ISR(TIMER1_OVF_vect)
{
  // if no tone requested, do nothing (but clear potential flags)
  if (currentCounts == 0)
    return;

  // stop timer (clear prescaler)
  *myTCCR1B &= 0xF8;

  // preload timer so that overflow happens after 'currentCounts' ticks
  *myTCNT1 = (unsigned int)(65536 - (unsigned long)currentCounts);

  *myTCCR1B |= 0x01;

  // toggle PB6 (mask 0x40)
  *portB ^= 0x40;
}

// ---------------- UART Functions ----------------
void U0init(int U0baud)
{
  unsigned long FCPU = 16000000UL;
  unsigned int tbaud = (unsigned int)(FCPU / 16UL / U0baud - 1UL);
  *myUCSR0A = 0x20;  // U2X0 = 0, clear status bits except TXC0 set
  *myUCSR0B = 0x18;  // RXEN0=1, TXEN0=1
  *myUCSR0C = 0x06;  // UCSZ01:0 = 3 -> 8-bit
  *myUBRR0  = tbaud;
}

unsigned char U0kbhit()
{
  return (*myUCSR0A & RDA) ? 1 : 0;
}

unsigned char U0getchar()
{
  return *myUDR0;
}

void U0putchar(unsigned char data)
{
  while(((*myUCSR0A) & TBE) == 0);
  *myUDR0 = data;
}
