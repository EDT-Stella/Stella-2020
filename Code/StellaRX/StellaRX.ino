//Stella Receiver

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "RoboClaw.h" // Roboclaw 
#include <Wire.h>     // Wire
#include <Stepper.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "Adafruit_TCS34725.h"
#include <Tic.h>

//==================Pin Definitions==============
#define CE_PIN   9
#define CSN_PIN 10
#define address2 0x80 // Address to Roboclaw
//===============================================

//=================Radio Globals=================
const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
RF24 radio(CE_PIN, CSN_PIN);
//===============================================

//=================RoboClaw Globals==============
RoboClaw roboclaw(&Serial3, 10000); // Create a Roboclaw object
bool actuatorForward = true;
bool motorForward = true;
//===============================================

//=================ColorSensor Globals==============
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define ticSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial ticSerial(10, 11);
#endif

// Each color compartment corresponds to a numerical ID from 1 to 6, with the index increasing as you go clockwise
// and looping back to 1 when you go clockwise from 6
short currentColor = 1; // The current index the motor is at
short targetColor = 1; // The target index for the motor to move to

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
TicSerial tic(ticSerial);
//===============================================

struct dataReceived {
    byte jX;
    byte jY;
    char pass[7];
    bool button1, button2, button3, button4, rocker, jsButton;
    byte ballColor;  
};
dataReceived data; // this must match dataToSend in the TX

struct ackData { //Acknowledgement data
    byte ballColor;
    bool ballDispensed;
};
ackData aData; //Acknowledgement data to be returned to controller

bool newData = false;

//Only needed for debugging
int counter = 0;

//Interrupt Flag
volatile byte mssg = false;
//==================================================================
//======================dataReceived()==============================
void dataReceived_Interrupt() {
  Serial.println("Interrupt called");
  mssg = true;
  //getData();
}
//==================================================================
//==========================setup()=================================
void setup() {
    Serial.begin(9600);
    Serial.println("Stella Receiver Starting -- Edit");

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
    //RoboClaw initialization and settings
    Serial3.begin(57600); // Wire communication with Roboclaw
    //-----------------------------------------

    //-----------------------------------------
    //Color Sensor initialization and settings
    ticSerial.begin(9600);
    // Read the current color index from address 0 of the EEPROM
    currentColor = EEPROM.read(0);
    targetColor = currentColor; // Makes sure the stepper motor doesn't move to a target

    delay(20); // Give time for stepper motor to start

    // Set up stepper motor
    tic.haltAndSetPosition(0);
    tic.exitSafeStart();

    // Ensures that the color sensor is connected
    // Seemingly necessary for some reason (will not run properly without this)
    if (tcs.begin()) {
      Serial.println("Found sensor");
    } else {
      Serial.println("No TCS34725 found ... check your connections");
      while (1); // halt!
    }
    //-----------------------------------------
}
//==================================================================
//===========================mapInput()=================================
void mapInput(byte& joystickData, bool& forwardVar) {
  // Joystick Value|Sent to Motor
  // 0               127
  //                 67
  // 128             0
  //                 67
  // 255             127
  
  // joystickData's 7th bit is 1 if > 127 and 0 otherwise
  forwardVar = !(joystickData >> 7);

  
  if(joystickData > 205) {
    joystickData = 127;
  }
  else if(joystickData > 154) {
    joystickData = 67;
  }
  else if(joystickData > 103) {
    joystickData = 0;
  }
  else if(joystickData > 51) {
    joystickData = 67;
  }
  else {
    joystickData = 127;
  }
}
//==================================================================
//===========================sendMotorControl()=====================
void sendMotorControl() {
  // Motor commands to roboclaw
  // Values for DutyM1, Duty M2, signed int 16 byte from -32767 to 32767 or (unsigned int)65536
  // -32767 is full speed "backwards"  and 32767 is full speed "forward"
  // WE ARE ADOPTING THE 2'S COMPLIMENT OF THIS FUNCTION SO ONLY USE -32767 TO 32767
  delay(20);
  if(actuatorForward) {
    roboclaw.ForwardM1(address2, data.jY);
  }
  else {
    roboclaw.BackwardM1(address2, data.jY);
  }  
  delay(20);
  if(motorForward) {
    roboclaw.ForwardM2(address2, data.jX);
  }
  else {
    roboclaw.BackwardM2(address2, data.jX);
  }  
  delay(20);
}
//==================================================================
//===========================loop()=================================
void loop() {
  //---------Loop must start with this function calls--------------
  if (mssg) {
    getData();
    mssg = false;
  }
  
  //Comment out showData() for final code
  //showData();
  //-----------------add code below here---------------------------

  //joysticks();  // call joystick assignments from analog pins
  mapInput(data.jX, actuatorForward); // Gets values that can be sent to motors
  mapInput(data.jX, motorForward);
  sendMotorControl(); // Runs motors

  float red, green, blue; // Variables for color values
  
  tcs.setInterrupt(false);  // turn on LED

  if (mssg) {
    getData();
    mssg = false;
  }
  delay(60);  // takes 50ms to read
  if (mssg) {
    getData();
    mssg = false;
  }
  
  tcs.getRGB(&red, &green, &blue); // Reads color values
  
  if (mssg) {
    getData();
    mssg = false;
  }
  
  tcs.setInterrupt(true);  // turn off LED

  if (mssg) {
    getData();
    mssg = false;
  }

  // Prints RGB values
  // Serial.print("R:\t"); Serial.print(int(red)); 
  // Serial.print("\tG:\t"); Serial.print(int(green)); 
  // Serial.print("\tB:\t"); Serial.print(int(blue));
  // Serial.print("\n");

  findBallColor(red, green, blue);
  
  if (mssg) {
    getData();
    mssg = false;
  }
  
  // Moves the stepper motor until the destination is reached
  // Finds the distance between the staring and target indexes
  short distance = targetColor - currentColor;

  // Moves the opposite direction if the distance is greater than 3
  if(distance > 3) {
    distance -= 6;
  }
  else if(distance < -3) {
    distance += 6;
  }

  //==========temp removal=============
  int targetPos = tic.getCurrentPosition() + (distance * 33.0);
  tic.setTargetPosition(targetPos);
  
  waitForPosition(targetPos);
  //==========temp removal=============
  
  currentColor = targetColor;
  // Write the current color index to address 0 of the EEPROM
  // update function only writes if the value differs from what is stored at the address
  EEPROM.update(0, currentColor); 
    
  //delayWhileResettingCommandTimeout(1000); 
}
//==================================================================
//========================getData()=================================
void getData() {
  //Serial.print("Attempting Read\n");
    if ( radio.available() ) {
        radio.read( &data, sizeof(data) );
        updateReplyData();
        showData();
        newData = true;
        Serial.print("Data Received\n");
    }
}
//==================================================================
//========================showData()================================
//Called each loop to update the reply data, takes no parameters.
//Status: Currently has no outside connections to read from and so
//is generating sequential values to send as acknowledgement data
//Needs: To read information from Stella and format it
void showData() {
  if (newData == true) {
    if (data.button1 || data.button2 || data.button3 || data.button4) {
      Serial.print("Data received:\nButtons: ");
      Serial.print(data.button1);
      Serial.print(" ");
      Serial.print(data.button2);
      Serial.print(" ");
      Serial.print(data.button3);
      Serial.print(" ");
      Serial.println(data.button4);
    }
  newData = false;
  }
}
//==================================================================
//===================updateReplyData================================
//Called each loop to update the reply data, takes no parameters.
//Status: Currently has no outside connections to read from and so
//is generating sequential values to send as acknowledgement data
//Needs: To read information from Stella and format it
void updateReplyData() {
  if (counter > 10) {
    if (aData.ballColor < 8) {
      aData.ballColor++;
    } else {
      aData.ballColor = 0;
    }
    counter = 0;
  } else {
    counter++;
  }

  if (aData.ballDispensed == true) {
    aData.ballDispensed = false;
  }
  else {
    aData.ballDispensed = true;
  }
  radio.writeAckPayload(1, &aData, sizeof(ackData)); // load the payload for the next time
}
//==================================================================
//=============delayWhileResettingCommandTimeout====================
// Must use this delay when stepper motor is connected
void delayWhileResettingCommandTimeout(uint32_t ms) {
  uint32_t start = millis();
  do {
    tic.resetCommandTimeout();
  } while ((uint32_t)(millis() - start) <= ms);
}
//==================================================================
//=======================waitForPosition============================
void waitForPosition(int32_t targetPosition) {
  do {
    tic.resetCommandTimeout();
  } while (tic.getCurrentPosition() != targetPosition);
}
//==================================================================
//===============================around=============================
bool around(int sensorInput, int value) {
  return (sensorInput > value - 10 && sensorInput < value + 10);
}
//==================================================================
//========findBallColor(float red, float green, float blue)=========
void findBallColor(float red, float green, float blue) {
  if(around(red, 145) && around(green, 53) && around(blue, 45)) {
    Serial.println("Red");
    targetColor = 1;
  }
  else if(around(red, 147) && around(green, 60) && around(blue, 34)) {
    Serial.println("Orange");
    targetColor = 2;
  }
  else if(around(red, 125) && around(green, 128) && around(blue, 45)) {
    Serial.println("Yellow");
    targetColor = 3;
  }
  else if(around(red, 76) && around(green, 113) && around(blue, 50)) {
    Serial.println("Green");
    targetColor = 4;
  }
  else if(around(red, 39) && around(green, 97) && around(blue, 113)) {
    Serial.println("Light Blue");
    targetColor = 5;
  }
  else if(around(red, 28) && around(green, 78) && around(blue, 136)) {
    Serial.println("Dark Blue");
    targetColor = 5;
  }
  else if(around(red, 72) && around(green, 67) && around(blue, 105)) {
    Serial.println("Purple");
    targetColor = 6;
  }
  else if(around(red, 104) && around(green, 54) && around(blue, 81)) {
    Serial.println("Pink");
    targetColor = 2;
  }
}
