void setup()
{
pinMode(2,INPUT);
pinMode(5,OUTPUT);
pinMode(6, OUTPUT);
}
void loop()
{
char input1 = digitalRead(2);
if(input1 == LOW ){
digitalWrite(5,HIGH);
digitalWrite(6,LOW);
}
else if(input1 == HIGH){
digitalWrite(6,HIGH);
digitalWrite(5,LOW);
}
}
