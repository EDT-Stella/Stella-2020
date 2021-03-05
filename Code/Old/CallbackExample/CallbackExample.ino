#include <MsTimer2.h>

const byte buttonPin = 5;

void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop()
{
  if(buttonFivePressed())
  {
   flashLedThirteen(5000);
  }
}

bool buttonFivePressed(void)
{
  static unsigned long lastMillis = 0;
  static byte lastPress = HIGH;
  byte currentPress = digitalRead(buttonPin);
  if(currentPress != lastPress)
  {
    if(millis() - lastMillis < 200) return false;
    lastPress = currentPress;
    if(currentPress == LOW)
    {
      Serial.println(" button press detected!");
      lastMillis = millis();
      return true;
    }
  }
  return false;
}

void flashLedThirteen(int ledTime)
{
  digitalWrite(13, HIGH);
  Serial.println("LED ON");
  // sets a new timer callback function
  MsTimer2::set(ledTime, [] {  // the square brackets define the start of the anonymous 
                               // callback function which is executed after 5000 milliseconds 
                               // in this example
    digitalWrite(13, LOW);
    Serial.println("Timer expired...");
    Serial.println("LED OFF");
    MsTimer2::stop();
  }); // the curly brace defines the end of the anonymous callback function
  MsTimer2::start();
}
