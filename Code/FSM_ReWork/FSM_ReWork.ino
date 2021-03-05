//Stella Receiver
// Authors:
//  Dan Even
//  Shane J
//  Aden Prince


#include "Adafruit_TCS34725.h"
#include <nRF24L01.h>
#include <RF24.h>

//===============Pin Definitions=================
#define CE_PIN   9
#define CSN_PIN 10
#define address2 0x80 // Address to Roboclaw
//===============================================

//===============Radio Globals===================
const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
RF24 radio(CE_PIN, CSN_PIN);
//===============================================

//===============Color Sensor Globals============
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
//===============================================

struct dataReceived { // Data from the controller
  byte jX;            // this must match dataToSend in the TX
  byte jY;
  char pass[7];
  bool button1, button2, button3, button4, rocker, jsButton;
  //byte ballColor;  
};
dataReceived data; 

struct ackData { //Acknowledgement data
  byte ballColor;
  bool ballDispensed;
};
ackData aData; //Acknowledgement data to be returned to controller

volatile bool RotateAuger = false;
volatile bool ColorSense = false;
volatile bool RotateBarrel = false;
volatile bool ExtendingAuger = false;

enum color { 
  Red, 
  Orange,
  Yellow,
  Green,
  Blue,
  Purple,
  Pink
} storeColor, dropColor;

// RGB values for each color
int colorMap[8][3] = {{145, 53, 45},   // Red
                      {147, 60, 34},   // Orange
                      {125, 128, 45},  // Yellow
                      {76, 113, 50},   // Green
                      {28, 78, 136},   // Blue
                      {72, 67, 105},   // Purple
                      {104, 54, 81}};  // Pink

//Interrupt Flag - set to true if RF24 indicates new message
volatile byte mssg = false;
//==================================================================
//======================dataReceived()==============================
void dataReceived_Interrupt() {
  //Serial.println("Interrupt called");
  mssg = true;
  //getData();
}

void setDropColor(){
  if(storeColor == Red) {
    dropColor = Blue; 
  }
  else if (storeColor == Yellow) {
    dropColor = Purple;
  }
  else if (storeColor == Green) {
    dropColor = Red;
  }
  else if (storeColor == Blue) {
    dropColor = Yellow;
  }
  else if (storeColor == Purple) {
    dropColor = Green;
  }
}

bool rotateAuger() {
  if (!ColorSense) {
    if (!RotateBarrel) {
      delay(20);
      if (actuatorForward) {
      roboclaw.ForwardM1(address2, data.jY);
      }
      else {
        roboclaw.BackwardM1(address2, data.jY);
      }  
      delay(20);
      if (motorForward) {
        roboclaw.ForwardM2(address2, data.jX);
      }
      else {
        roboclaw.BackwardM2(address2, data.jX);
      }
      delay(20);
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

      // Get storeColor from RGB values
      if (!RGBToColor(red, green, blue, storeColor)) {
        return false; // Color not found
      }
      setDropColor();
      return true; // Color found
    }
  }
}

// Check if the first parameter is within +/- 10 of the second parameter
// Parameters:
// int val1, val2 - values to compare
// Return:
// bool of whether the first value is within +/- 10 of the second value
bool within10(int val1, int val2) {
  return (val1 <= val2 + 10 && val1 >= val2 - 10);
}

// Get a color based on passed RGB values using the color map
// Parameters:
// float red, green, blue - color values from the color sensor
// color result reference - color converted from the passed RGB values
// Return:
// bool of whether a color was found or not
bool RGBToColor(float red, float green, float blue, color &result) {
  // Iterate through color map
  for (int i = 0; i < 7; ++i) {
    // Check if the passed RGB values are close to the current color
    if (within10(red, colorMap[i][0]) &&
        within10(green, colorMap[i][1]) &&
        within10(blue, colorMap[i][2])) {
          
      // Set result to current color
      result = (color)i;
      return true; // Color found
    }
  }

  return false; // Color not found
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
  //Radio initialization and settings
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.enableAckPayload();
  radio.startListening();
  radio.writeAckPayload(1, &aData, sizeof(ackData)); // pre-load data
  attachInterrupt(digitalPinToInterrupt(2), dataReceived_Interrupt, FALLING);
  Serial.println("Radio is starting");
  //-----------------------------------------
  
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
