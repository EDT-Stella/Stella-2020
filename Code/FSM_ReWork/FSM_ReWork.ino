//Stella Receiver
// Authors:
//  Dan Even
//  Shane J


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

//Interrupt Flag - set to true if RF24 indicates new message
volatile byte mssg = false;
//==================================================================
//======================dataReceived()==============================
void dataReceived_Interrupt() {
  //Serial.println("Interrupt called");
  mssg = true;
  //getData();
}

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
  
  storeColor = Red ;
  dropColor = Purple;

}

void loop() {
  // put your main code here, to run repeatedly:

}
