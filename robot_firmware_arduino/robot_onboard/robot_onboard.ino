#include <Encoder.h>

#define ENCAF 2
#define ENCAR A1
#define PWMA 5
#define MAF A5
#define MAR A4

Encoder encoder(ENCAF, ENCAR);

int pwm_value = 0, forward = 0, reverse = 0;
long encoder_reading = 0;
String data;
long elapsed_time = 0;

void setup() 
{
  Serial.begin(9600);
  pinMode(PWMA, OUTPUT);
  pinMode(MAF, OUTPUT);
  pinMode(MAR, OUTPUT);
}

void loop() 
{
  if(millis() - elapsed_time >= 50)
  {
    analogWrite(PWMA, pwm_value);
    digitalWrite(MAF, forward);
    digitalWrite(MAR, reverse);
    
    while(Serial.available() > 0)
    {
      char s = Serial.read();
      data += s; 
      if(s == '\n')
      {
        int prev = 0, count = 0;
        for (int  i = 0; i < data.length(); i++)
        {
          if(data[i] == ';')
          {
            if(count == 0)
            {
              pwm_value = data.substring(prev, i).toInt();
            }
            else if(count == 1)
            {
              forward = data.substring(prev, i).toInt();
            }
            else if(count == 2)
            {
              reverse = data.substring(prev, i).toInt();
            }
            prev = i + 1;
            count = count + 1;
          }
        }
        data = "";
      }
    }
  
    encoder_reading = encoder.read();
    Serial.println(encoder_reading);
  
    elapsed_time = millis();
  }
}
