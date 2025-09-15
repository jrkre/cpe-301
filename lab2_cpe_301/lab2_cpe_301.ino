/*
  lab2_cpe_301.ino
  Created by: Joshua Knight, Nicky Victoriano
  Date: 9/15/25
  Description: This program reads two digital inputs and lights up one of four LEDs based on the input combination.
*/

void setup()
{
  pinMode(2,INPUT);
  pinMode(4, INPUT);
  pinMode(5, OUTPUT); //led 1
  pinMode(6, OUTPUT); //led 2
  pinMode(7, OUTPUT); //led 3
  pinMode(8, OUTPUT); //led 4
  Serial.begin(9600);
}

void loop()
{
  char input1 = digitalRead(2);
  char input2 = digitalRead(4);
  int sum = 0;
  if (input1 == HIGH)
  {
    sum +=2;
  }
  if(input2 == HIGH)
  {
    sum +=1;
  }

  if (sum == 0)
  {
    Serial.println("LED1");
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
  }
  else if (sum == 1)
  {
    Serial.println("LED2");
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
  }
  else if (sum == 2)
  {
    Serial.println("LED3");
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
  }
  else if (sum == 3)
  {
    Serial.println("LED4");
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
  }



}
