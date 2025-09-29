/*
  lab4_cpe_301.ino
  Created by: Joshua Knight, Nicky Victoriano
  Date: 9/29/25
  Description: This program reads from the pedestrian crosswalk buttons and lights up the corresponding LED.
*/

unsigned char* ddr_b = (unsigned char*) 0x24; // Data Direction Register for PORTB
unsigned char* port_b = (unsigned char*) 0x25; // Output register for PORTB
unsigned char* pin_b = (unsigned char*) 0x23;
unsigned char* ddr_h = (unsigned char*) 0x101; // Data Direction Register for PORTH
unsigned char* port_h = (unsigned char*) 0x102; // Output register for PORTH


void setup() {
  // put your setup code here, to run once:
  //pinMode(4, INPUT); // button
  ddr_b = 0b00001000; 
  ddr_h = 0b00001110;


  Serial.begin(9600);
}

int counter = 0;

void loop() {
  // put your main code here, to run repeatedly:

  // *port_h = 0b00010000; //green
  // *port_h = 0b00100000; //yellow
  // *port_h = 0b01000000; //red

  /// if button

  int button = *pin_b;

  // Serial.println(button);


  if (button != 228){
    counter += 1;
    switch (counter % 4)
      {
        case 0:
          Serial.println("OFF");
          *port_h = 0b00000000;
          break;
        case 1:
          Serial.println("RED");
          *port_h = 0b01000000; //red
          break;
        case 2:
          Serial.println("YELLOW");
          *port_h = 0b00100000; //yellow
          break;
        case 3:
          Serial.println("GREEN");
          *port_h = 0b00010000; //green
          break;
      }
      delay(300);
  }
}
