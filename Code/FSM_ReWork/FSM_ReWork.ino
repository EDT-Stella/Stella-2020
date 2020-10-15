//Stella Receiver
// Authors:
//  Dan Even
//  Shane J


#include "Adafruit_TCS34725.h"

//===============Color Sensor Globals============
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
//===============================================

volatile bool RotateAuger = false;
volatile bool ColorSense = false;
volatile bool RotateBarrel = false;
volatile bool ExtendingBarrel = false;

enum colorList{ 
  Red, 
  Orange,
  Yellow,
  Green,
  LightBlue,
  DarkBlue,
  Purple,
  Pink
}storeColor, dropColor;

void setSortColor(){
  if(storeColor == Red) {
    dropColor = Purple; 
  }
  else if (storeColor == Orange){
    dropColor = Yellow; 
  }
  else if (storeColor == Yellow){
    dropColor = Orange;
  }
  else if (storeColor == Green){
    dropColor = LightBlue;
  }
  else if (storeColor == LightBlue){
    dropColor = Green;
  }
  else if (storeColor == Purple) 
  {
    dropColor = Red;
  }
}

bool rotateAugerUp() {
  if (!ColorSense) {
    if (!RotateBarrel) {
      //Rotate Auger
    }
  }
  return true;
}

bool rotateAugerDown() {
  if (!ColorSense) {
    if (!RotateBarrel) {
      //Rotate Auger
    }
  }
  return true;
}

//Uses the sensor to get the color of the ball to be sorted
bool getColor() {
  if (!RotateAuger) {
    if (!RotateBarrel) {
      //Call Color sensor
      float red, green, blue;
      tcs.setInterrupt(false);  //Turn on LED
      delay(60);
      tcs.setInterrupt(true);  //Turn off LED
      tcs.getRGB(&red, &green, &blue); // Read color values
    }
  }
}




// Rotates the barrel to dispense/sort the correct color
// Parameters:
// colorList c - the color to be dispensed/sorted to
// bool sort - if TRUE: move to appropriate color for sorting
//                FALSE: move to appropriate color for dispensing
bool rotateBarrel(colorList c, bool sort) {
    if (!RotateAuger) {
      if (!ColorSense) {
        
      }
    }
}

void setup() {
  //-----------------------------------------
  //Color Sensor initialization and settings
  // Ensures that the color sensor is connected
  if (tcs.begin()) { // begin() starts the color sensor
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
  //-----------------------------------------
  
  storeColor = Red ;
  dropColor = Purple;

}

void loop() {
  // put your main code here, to run repeatedly:

}
