String data;

void setup() 
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() 
{
  while(Serial.available() > 0)
  {
    char s = Serial.read();
    data += s; 
    if(s == '\n')
    {
      if(data == "ON\n")
      {
        digitalWrite(13, HIGH);
        Serial.write("ON E");
      }
      else if(data == "OFF\n")
      {
        digitalWrite(13, LOW);
        Serial.write("OFF E");
      }
      data = "";
    }
  }
//  Serial.write("test");
//  delay(1000);
}
