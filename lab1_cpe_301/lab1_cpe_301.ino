//Lab Group: Joshua Knight, Nicky Victoriano


void setup() {

  Serial.begin(9600);
  Serial.println("Hello from the Serial monitor!");
  pinMode(4, OUTPUT); // Output is the macro
  pinMode(2, INPUT);

}

void loop() {
  char input = digitalRead(2);
  if(input == HIGH){
    digitalWrite(4,HIGH); Serial.println("Button has been pressed"); delay(1000);
  } else {
    digitalWrite(4,LOW);
  }
}
