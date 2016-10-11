#include "Timer.h"

Timer t;
int pin = 13;

void setup()
{
  Serial.begin(9600);
  pinMode(pin, OUTPUT);
  t.every(5000, takeReading);
}

void loop()
{
  digitalWrite(pin,LOW);
  while(1){
  t.update();
  digitalWrite(pin,HIGH);
  delay(300);
  digitalWrite(pin,LOW);
  delay(300);
  }
}

void takeReading()
{
  digitalWrite(pin,HIGH);
  delay(100);
  digitalWrite(pin,LOW);
}
