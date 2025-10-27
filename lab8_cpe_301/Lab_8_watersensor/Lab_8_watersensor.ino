
#define RDA 0x80
#define TBE 0x20  

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

void setup() 
{

  // setup the UART
  U0init(9600);
  // Serial.begin(9600);
  // setup the ADC
  adc_init();
}
void loop() 
{
  int threshold = 200;
  // read the water sensor value by calling adc_read() and check the threshold to display the message accordingly
  unsigned int read = adc_read(0);
  
  //if the value is less than the threshold display the value on the Serial monitor
  if ( read < threshold )
  {
    char *out = convertToString(read);
    for (int i =0; i < 3; i++)
    {
      U0putchar(out[i]);
    }
    U0putchar('\n');
  }
  // Serial.println(read);

  //if the value is over the threshold display "Water Level: High" message on the Serial monitor.
  if (read >= threshold)
  {
    char string [] = "Water Level: High";
    
    U0puts(string);
    U0putchar('\n');

  }

  //Use a threshold value that works for you with your sensor. There is no fixed value as sensor's sensitivity can differ.
}

// void intToStr(int N, char *str) {
//     int i = 0;
  
//     // Save the copy of the number for sign
//     int sign = N;

//     // If the number is negative, make it positive
//     if (N < 0)
//         N = -N;

//     // Extract digits from the number and add them to the
//     // string
//     while (N > 0) {
      
//         // Convert integer digit to character and store
//       	// it in the str
//         str[i++] = N % 10 + '0';
//       	N /= 10;
//     } 

//     // If the number was negative, add a minus sign to the
//     // string
//     if (sign < 0) {
//         str[i++] = '-';
//     }

//     // Null-terminate the string
//     str[i] = '\0';

//     // Reverse the string to get the correct order
//     for (int j = 0, k = i - 1; j < k; j++, k--) {
//         char temp = str[j];
//         str[j] = str[k];
//         str[k] = temp;
//     }
// }

char* convertToString(int i)
{
  static char out[4];
  out[3] = '\0';
  out[2] = (i % 10) + '0';
  i = i / 10;
  out[1] = (i % 10) + '0';
  i = i / 10;
  out[0] = (i % 10) + '0';
  return out;
  
}

void adc_init() //write your code after each commented line and follow the instruction 
{
  // setup the A register
  // set bit 7 to 1 to enable the ADC 
  *my_ADCSRA |= 0b10000000;
  
  // clear bit 5 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11011111;

  // clear bit 3 to 0 to disable the ADC interrupt 
  *my_ADCSRA &= 0b11111011;
  

  // clear bit 0-2 to 0 to set prescaler selection to slow reading
  *my_ADCSRA |= 0b00000111;

  // setup the B register
  // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11110111;

  // clear bit 2-0 to 0 to set free running mode
  *my_ADCSRB &= 0b11111000;

  // setup the MUX Register
  *my_ADMUX &= 0b11011111;
  *my_ADMUX &= 0b11100000;

  // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX &= 0b01111111;

  // set bit 6 to 1 for AVCC analog reference
  *my_ADMUX |= 0b01000000;

  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11011111;

  // clear bit 4-0 to 0 to reset the channel and gain bits
  *my_ADMUX &= 0b11110000;

}

unsigned int adc_read(unsigned char adc_channel_num) //work with channel 0
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0b11100000;
  // clear the channel selection bits (MUX 5) hint: it's not in the ADMUX register
  *my_ADCSRB &= 0b11110111;

 
  // set the channel selection bits for channel 0
  *my_ADMUX |= (adc_channel_num & 0x07);
 

  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0b01000000;

  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register and format the data based on right justification (check the lecture slide)
  
  return *my_ADC_DATA & 0x03FF;
}

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void U0puts(const char* s) {
  while(*s) U0putchar(*s++);
}


unsigned char bitsToHex(unsigned char bits){
  if (bits < 10) // 0 - 9
    return '0' + bits;
  else // A - F
    return 'A' + (bits - 10);
}