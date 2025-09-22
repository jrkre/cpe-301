/*
  lab3_cpe_301.ino
  Created by: Joshua Knight, Nicky Victoriano
  Date: 9/23/25
  Description: This program reads from the pedestrian crosswalk buttons and lights up the corresponding LED.
*/

void setup()
{
  pinMode(4, INPUT); // button
  pinMode(8, OUTPUT); // green light
  pinMode(9, OUTPUT); // yellow light
  pinMode(10, OUTPUT); // red light
  pinMode(11, OUTPUT); // blue light
  Serial.begin(9600);
}

void loop()
{
  /*
    You are required to design and implement a simple traffic light controller using an Arduino.
    The system should control three LEDs (red, yellow, green) to simulate a standard traffic light
    cycle. Additionally, there will be a pedestrian crossing LED that will blink during the red light
    duration.
  */
    int buttonState = digitalRead(4);

    if (buttonState == HIGH)
    {
      Serial.println("Button Pressed");
      // Green light
      Serial.println("Green Light ON");
      digitalWrite(8, HIGH); // green on
      digitalWrite(9, LOW); 
      digitalWrite(10, LOW);
      digitalWrite(11, LOW);
      delay(5000);

      // Yellow light
      Serial.println("Yellow Light ON");
      digitalWrite(8, LOW); 
      digitalWrite(9, HIGH); // yellow on
      digitalWrite(10, LOW);
      digitalWrite(11, LOW);
      delay(3000);

      // Red light
      Serial.println("Red Light ON");
      digitalWrite(8, LOW);
      digitalWrite(9, LOW);
      digitalWrite(10, HIGH);// red on

      // blink blue light
      for (int i = 0; i < 5; i++) 
      {
        digitalWrite(11, HIGH);
        delay(500);
        digitalWrite(11, LOW);
        delay(500);
      }

      digitalWrite(10, LOW); // red
      digitalWrite(8, HIGH);// green
    }
    else
    {
      Serial.println("Waiting for button press...");
      // Default state: Red light on, others off
      digitalWrite(8, LOW);  // green off
      digitalWrite(9, LOW);  // yellow off
      digitalWrite(10, HIGH);// red on
      digitalWrite(11, LOW); // blue off
    }
  

}
