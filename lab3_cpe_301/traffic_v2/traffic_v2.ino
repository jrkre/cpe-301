/*
  lab3_cpe_301.ino
  Created by: Joshua Knight, Nicky Victoriano
  Date: 9/23/25
  Description: This program reads from the pedestrian crosswalk buttons and lights up the corresponding LED.
*/

unsigned char* ddr_b = (unsigned char*) 0x24;
unsigned char* port_b = (unsigned char*) 0x25;
unsigned char* ddr_h = (unsigned char*) 0x101;
unsigned char* port_h = (unsigned char*) 0x102;

void setup()
{
  //pinMode(4, INPUT); // button
  ddr_b = 0b00001000; 
  ddr_h = 0b00001110; //

  Serial.begin(9600);
}

void loop()
{
//    At the beginning of your code, add the following global variables (outside of
// setup() or loop() ):
// unsigned char* ddr_b = (unsigned char*) 0x24;
// unsigned char* port_b = (unsigned char*) 0x25;
// unsigned char* ddr_h = (unsigned char*) 0x101;
// unsigned char* port_h = (unsigned char*) 0x102;
// These correspond to the memory addresses of the AVR registers:
// • DDRB → Data Direction Register for PORTB
// • PORTB → Output register for PORTB
// • DDRH → Data Direction Register for PORTH
// • PORTH → Output register for PORTH
// Being global variables, they should be declared outside of the setup() and loop()
// functions so that they can be used anywhere in your code.
    //int buttonState = digitalRead(4);

    //if (buttonState == HIGH)
    {
      // Green light
      Serial.println("Green Light ON");

      *port_h = 0b00010000; //green
      delay(5000);

      // Yellow light
      Serial.println("Yellow Light ON");
      *port_h = 0b00100000; //yellow


      delay(3000);

      // Red light
      Serial.println("Red Light ON");
      *port_h = 0b01000000; //red


      // blink blue light
      for (int i = 0; i < 5; i++) 
      {
        *port_b = 0b00010000;
        delay(500);
        *port_b = 0b00000000;
        delay(500);
      }

    }
    // else
    // {
    //   Serial.println("Waiting for button press...");
    //   // Default state: Red light on, others off
    //   digitalWrite(7, HIGH);  // green on
    //   digitalWrite(8, LOW);  // yellow off
    //   digitalWrite(9, LOW);// red off
    //   digitalWrite(10, LOW); // blue off
    // }
  

}
