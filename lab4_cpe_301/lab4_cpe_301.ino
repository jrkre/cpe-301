/*
  lab4_cpe_301.ino
  Created by: Joshua Knight, Nicky Victoriano
  Date: 9/29/25
  Description: This program reads from the pedestrian crosswalk buttons and lights up the corresponding LED.
*/

void setup() {
  // put your setup code here, to run once:
  pinMode(10, INPUT); // button
  pinMode(7, OUTPUT); // green light
  pinMode(8, OUTPUT); // yellow light
  pinMode(9, OUTPUT); // red light


  Serial.begin(9600);
}

int counter = 0;

void loop() {
  // put your main code here, to run repeatedly:

  int state = digitalRead(10);


  if (state == HIGH)
  {
    counter ++;

    switch (counter % 4)
    {
      case 0:
        Serial.println("OFF");
        digitalWrite(9, LOW);
        digitalWrite(8, LOW);
        digitalWrite(7, LOW);
        break;
      case 1:
        Serial.println("RED");
        digitalWrite(9, HIGH);
        digitalWrite(8, LOW);
        digitalWrite(7, LOW);
        break;
      case 2:
        Serial.println("YELLOW");
        digitalWrite(9, LOW);
        digitalWrite(8, HIGH);
        digitalWrite(7, LOW);
        break;
      case 3:
        Serial.println("GREEN");
        digitalWrite(9, LOW);
        digitalWrite(8, LOW);
        digitalWrite(7, HIGH);
        break;
    }

    

    delay (200); //debounce
  }



}
