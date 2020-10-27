//Stella Receiver
// Authors:
//  Dan Even
//  Shane J
//  Aden Prince


#include "Adafruit_TCS34725.h"

//===============Color Sensor Globals============
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
//===============================================

volatile bool RotateAuger = false;
volatile bool ColorSense = false;
volatile bool RotateBarrel = false;
volatile bool ExtendingBarrel = false;

enum color { 
  Red, 
  Orange,
  Yellow,
  Green,
  LightBlue,
  DarkBlue,
  Purple,
  Pink
} storeColor, dropColor;

// RGB values for each color
int colorMap[8][3] = {{145, 53, 45},   // Red
                      {147, 60, 34},   // Orange
                      {125, 128, 45},  // Yellow
                      {76, 113, 50},   // Green
                      {39, 97, 113},   // Light Blue
                      {28, 78, 136},   // Dark Blue
                      {72, 67, 105},   // Purple
                      {104, 54, 81}};  // Pink

void setDropColor() {
  if (storeColor == Red) {
    dropColor = Purple; 
  }
  else if (storeColor == Orange) {
    dropColor = Yellow; 
  }
  else if (storeColor == Yellow) {
    dropColor = Orange;
  }
  else if (storeColor == Green) {
    dropColor = LightBlue;
  }
  else if (storeColor == LightBlue) {
    dropColor = Green;
  }
  else if (storeColor == Purple) {
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

// Use the sensor to get the color of the ball to be sorted
bool getColor() {
  ColorSense = true;
  if (!RotateAuger) {
    if (!RotateBarrel) {
      // Call Color sensor
      float red, green, blue;
      tcs.setInterrupt(false); // Turn on LED
      delay(60);
      tcs.setInterrupt(true);  // Turn off LED
      tcs.getRGB(&red, &green, &blue); // Read color values
      storeColor = RGBToColor(red, green, blue);
      setDropColor();
    }
  }
}

// Check if the first parameter is within +/- 10 of the second parameter
// Parameters:
// int val1, val2 - values to compare
// Return:
// bool of of whether the first value is within +/- 10 of the second value
bool within10(int val1, int val2) {
  return (val1 <= val2 + 10 && val1 >= val2 - 10);
}

// Get a color based on passed RGB values using the color map
// Parameters:
// float red, green, blue - color values from the color sensor
// Return:
// color corresponding to the passed RGB values
color RGBToColor(float red, float green, float blue) {
  // Iterate through color map
  for (int i = 0; i < 8; ++i) {
    // Check if the passed RGB values are close to the current color
    if (within10(red, colorMap[i][0]) &&
        within10(green, colorMap[i][1]) &&
        within10(blue, colorMap[i][2])) {
      // Return current color
      return (color)i;
    }
  }

  // FIX: Return Red (0) when color not detected
  return Red;
}

// Rotates the barrel to dispense/sort the correct color
// Parameters:
// color c - the color to be dispensed/sorted to
// bool sort - if TRUE: move to appropriate color for sorting
//                FALSE: move to appropriate color for dispensing
bool rotateBarrel(color c, bool sort) {
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
  
  storeColor = Red;
  dropColor = Purple;

}

void loop() {
  // put your main code here, to run repeatedly:

}
