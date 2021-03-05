//Stella Receiver

#include <SPI.h>
//#include "nRF24L01.h"
//#include "RF24.h"
#include <RF24.h>
#include "RoboClaw.h" // Roboclaw 
#include <Wire.h>     // Wire
#include <Stepper.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "Adafruit_TCS34725.h"
#include "Tic.h"
#include "Task.h"
#include "TaskScheduler.h"
#include <Servo.h>

#define RATE_BLINKER_BLINK    500   //Blink LED_BLINKER - Timed Task

//===============Pin Definitions=================
#define CE_PIN  5 // Radio CE pin
#define CSN_PIN 6 // Radio CSN pin
#define DROP_DOOR_PIN 12 // Drop door servo pin
#define TCS_INTERRUPT_PIN 3 // Color sensor interrupt pin
#define STEPPER_RX 10 // Stepper motor controller RX pin
#define STEPPER_TX 11 // Stepper motor controller TX pin
#define address2 0x80 // Address to Roboclaw
//===============================================

//===============State Flags=====================
// Condition for opening ball drop door
volatile byte BARREL_IN_POSITION = false; 

//Radio interrupt Flag
volatile byte MSSG = false;

// Color sensor interrupt flag
volatile byte TCS_STATE = false;

//Radio new data flag - only for debugging
bool newData = false;

//Auger Rotate Flag
bool ROTATE_AUGER = false;

//Barrel rotate flag.
bool ROTATE_BARREL = false;
//===============================================


//===============Radio Globals===================
const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
RF24 radio(CE_PIN, CSN_PIN);
//===============================================

//===============RoboClaw Globals================
//RoboClaw roboclaw(&Serial3, 10000); // Create a Roboclaw object
bool actuatorForward = true;
bool motorForward = true;
//===============================================

//===============Stepper Motor Globals===========
TicSerial tic(SERIAL_PORT_HARDWARE_OPEN);
//===============================================

//===============Color Sensor Global=============
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
//===============================================

// Each color compartment corresponds to a numerical ID from 1 to 6, with the index increasing as you go clockwise
// and looping back to 1 when you go clockwise from 6
short currentColor = 1; // The current index the motor is at
short targetColor = 1; // The target index for the motor to move to
short NUM_COLORS = 6;


struct dataReceived { // Data from the controller
  //Unused
  byte jX;

  //Auger Extend/Retract
  byte jY;

  //Unknown
  char pass[7];

  //Button 1 Pickup: Unused
  //Button 1 Drop:   Drop Ball
  //Button 2 Pick/Drop: Shift Color Right
  //Button 3 Pick/Drop: Shift Color Left
  //Button 4 Pick/Drop: Unused
  //Button JS Pickup: Rotate Auger Toggle
  //Rocker: Switch Between Pickup and Drop States
  bool button1, button2, button3, button4, rocker1, rocker2, jsButton;

  byte ballColor;
};

// Struct will hold the data from the controller
dataReceived data; 
// Struct will hold the previous data from the controller
dataReceived lastData;



struct ackData { //Acknowledgement data
  byte ballColor;
  bool ballDispensed;
};
ackData aData; //Acknowledgement data to be returned to controller

//Only needed for debugging
int counter = 0;

//======================dataReceived()==============================
void dataReceived_Interrupt() {
  MSSG = true;
}

//======================AugerRotateMotor==============================
class AugerRotateMotor : public TriggeredTask
{
public:
  virtual void run(uint32_t schTime);
  virtual bool canRun(uint32_t now);
  AugerRotateMotor(uint8_t _pin);
  
  private:
  uint8_t pin;
  bool isRotating;
};

AugerRotateMotor::AugerRotateMotor(uint8_t _pin) : TriggeredTask(), pin(_pin)
{
   pinMode(pin, OUTPUT);   
}

bool AugerRotateMotor::canRun(uint32_t now)
{
  if (data.rocker1 == true && data.jsButton == true) {
    return true;
  }
  
  return false;
}

void AugerRotateMotor::run(uint32_t now)
{
  data.jsButton = false;

  while (!TCS_STATE && !MSSG) {
    //Rotate motor incrementally until the color sensor senses a ball
  }
}

//======================AugerMoveActuator===========================
class AugerMoveActuator : public TimedTask
{
public:
  AugerMoveActuator();
  virtual void run(uint32_t schTime);
  virtual bool canRun(uint32_t now);

private:
  int pin;
  bool actuatorForward;
  
};

AugerMoveActuator::AugerMoveActuator() : TimedTask(millis()) {}
  
void AugerMoveActuator::run(uint32_t now){
  if (actuatorForward) {
    Serial.println("Running actuator forward.");
    //roboclaw.ForwardM1(address2, data.jY);
  }
  else {
    Serial.println("Running actuator backward.");
    //roboclaw.BackwardM1(address2, data.jY);
  }

  data.jY = 127;
}

bool AugerMoveActuator::canRun(uint32_t now) {
  bool canActuate = false;

  if (data.rocker1 && data.jY != 127) {
    canActuate = true;
  }

  return canActuate;
}
//==================================================================


//===========================DropBall===============================
class DropBall : public TriggeredTask
{
public:
  DropBall(uint8_t _pin);
  virtual void run(uint32_t now);
  virtual bool canRun(uint32_t now);
  
private:
  uint8_t pin;
  bool on;
  bool dropCondition;
  Servo dropDoor;
};

DropBall::DropBall(uint8_t _pin) : TriggeredTask(), pin(_pin), dropCondition(false) 
{
  pinMode(pin, OUTPUT); 
  dropDoor.attach(pin);
  dropDoor.write(0);
  
}

void DropBall::run(uint32_t now) {
  data.button1 = false;
  
  Serial.println("Dropping ball now.");
  dropDoor.write(180);
  delay(500);
  dropDoor.write(0);
  
}

bool DropBall::canRun(uint32_t now) {
  bool canDrop = false;

  if (data.rocker1 == false && data.rocker2 == false && data.button1 == true) {
    canDrop = true;
  }
  
  return canDrop;
}
//==================================================================


//===================BallShooter====================================
class BallShooter : public TriggeredTask
{
public:
  BallShooter();
  virtual void run(uint32_t now);
  virtual bool canRun(uint32_t now);
};

BallShooter::BallShooter() : TriggeredTask() {}

void BallShooter::run(uint32_t now) {
  data.button1 = false;

  Serial.println("Shooting ball now.");
}

bool BallShooter::canRun(uint32_t now) {
  bool canShoot = false;

  if (!data.rocker1 && data.rocker2 && data.button1) {
    canShoot = true;
  }

  return canShoot;
}
//==================================================================


//===================BarrelRotateStepper============================
/* Niraj Salunkhe
 * Pins used for stepper motor:
           pin1 .... STEPPER_RX ..... 10
           pin2 .... STEPPER_TX ..... 11
*/
class BarrelRotateStepper : public TriggeredTask
{
public:
  BarrelRotateStepper(uint32_t _pin1, uint32_t _pin2, short _currentColor, short _targetColor);
  void waitForPosition(int32_t targetPosition);
  virtual void run(uint32_t now);
  virtual bool canRun(uint32_t now);
  
private:
  uint32_t pin1;
  uint32_t pin2;
  short currentColor; // The current index the motor is at
  short targetColor; // The target index for the motor to move to
  
};

BarrelRotateStepper::BarrelRotateStepper(uint32_t _pin1, uint32_t _pin2, short _currentColor, short _targetColor):TriggeredTask(), pin1(_pin1), pin2(_pin2) {
  ticSerial.begin(9600);
  delay(20);
  ti
  currentColor = _currentColor;
  targetColor = _targetColor;
}

void BarrelRotateStepper::run(uint32_t now){
  if (data.button2 == true) {
    data.button2 = false;
    if (targetColor >= NUM_COLORS - 1) {
      targetColor = 0;
    }
    else {
      targetColor++;
    }
  } 
  else if(data.button3 == true) {
    data.button3 = false;
    if (targetColor <= 0) {
      targetColor = NUM_COLORS - 1;
    }
    else {
      targetColor--;
    }
  }
  
  short distance = targetColor - currentColor;
  
  // Moves the opposite direction if the distance is greater than 3
  if (distance > 3) {
    distance -= 6;
  }
  else if (distance < -3) {
    distance += 6;
  }
  
  int32_t targetPos = tic.getCurrentPosition() + (distance * 33.0);
  tic.setTargetPosition(targetPos);
  
  waitForPosition(targetPos);

  currentColor = targetColor;

  if (data.rocker1 == true) {
    ROTATE_AUGER = true;
  }

  Serial.print("Color moved by: ");  Serial.println(distance);
  Serial.print("  Target Color: ");  Serial.println(targetColor);
}

bool BarrelRotateStepper::canRun(uint32_t now) {
  if ( (data.rocker1 == true) && (data.rocker2 == false) ) {
    if ( (data.button2 == true) || (data.button3 == true) ) {
      Serial.println("Rotating barrel");
      return true;
    }
  }
  
  return false;
}

void BarrelRotateStepper::waitForPosition(int32_t targetPosition) {
  do {
    tic.resetCommandTimeout();
  } while (tic.getCurrentPosition() != targetPosition);
}

//==================================================================
// class Radio - Author: Dan
//  Holds all code for communicating through NRF24 tranceiver. 
//  Uses pins:
//     SCK  - 52
//     MISO - 50
//     MOSI - 51
//     CE   - 5
//     CSN  - 6
//  Status - Complete
//==================================================================
class Radio : public TriggeredTask
{
public:
  Radio(uint8_t _pin);
  virtual void run(uint32_t now);
  virtual bool canRun(uint32_t now);
  virtual void updateReplyData();
  virtual void showData();
private:
  uint8_t pin;
  bool on;
  bool readCondition;
  Servo dropDoor;
};

Radio::Radio(uint8_t _pin) : TriggeredTask(), pin(_pin), readCondition(false) 
{
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.enableAckPayload();
  radio.startListening();
  radio.writeAckPayload(1, &aData, sizeof(ackData)); // pre-load data
  attachInterrupt(digitalPinToInterrupt(2), dataReceived_Interrupt, FALLING);
  Serial.println("Radio is starting");
}

void Radio::run(uint32_t now) {
  //Serial.println("Attempting Read");
  if (radio.available()) {
    lastData = data;
    radio.read(&data, sizeof(data));
    updateReplyData();
    showData();
    newData = true;
    MSSG = false;
    //Serial.println("Data Received");
  }
}

void Radio::updateReplyData() {
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

void Radio::showData() {
  if (newData == true) {
    if (data.button1 || data.button2 || 
        data.button3 || data.button4 || data.jsButton) {
      Serial.print("Buttons: ");
      Serial.print(data.button1);
      Serial.print(" ");
      Serial.print(data.button2);
      Serial.print(" ");
      Serial.print(data.button3);
      Serial.print(" ");
      Serial.print(data.button4);
      Serial.print(" ");
      Serial.println(data.jsButton);
    }

    if (data.rocker1 != lastData.rocker1) {
      Serial.print("Rocker1 switched to ");
      if (data.rocker1) {
        Serial.println("ON");
      } 
      else {
        Serial.println("OFF");
      }
    }
    if (data.rocker2 != lastData.rocker2) {
      Serial.print("Rocker2 switched to ");
      if (data.rocker2) {
        Serial.println("ON");
      } 
      else {
        Serial.println("OFF");
      }
    }
    newData = false;
  }
}

bool Radio::canRun(uint32_t now) {
  return MSSG;
}
//==================================================================

//==================================================================
// class ColorSensor - Author: Aden
//  Contains code for using the Adafruit TCS34725 color sensor.
//  Uses pins:
//     INT - 3
//     SDA - 20
//     SCL - 21
// Status - Incomplete
//==================================================================
class ColorSensor : public TriggeredTask
{
public:
  ColorSensor();
  virtual bool canRun(uint32_t now);
  virtual void run(uint32_t now);

private:
  const int colorValThreshold = 1000;
  // Placeholder RGB values for each color
  int colorMap[7][3] = {{11000, 1200, 1500},   // Red
                        {12000, 6500, 2000},   // Yellow
                        {2500, 4400, 1800},    // Green
                        {1500, 2900, 5000},    // Blue
                        {4000, 3000, 5000},    // Purple
                        {20000, 14000, 16000}, // Pink
                        {33000, 7000, 6000}};  // Orange
  int RGBToColor(float r, float g, float b);
  bool withinThreshold(int val1, int val2);
};

// Color sensor setup
ColorSensor::ColorSensor() {
  // Start color sensor interrupt
  pinMode(TCS_INTERRUPT_PIN, INPUT_PULLUP); //TCS interrupt output is Active-LOW and Open-Drain
  attachInterrupt(digitalPinToInterrupt(TCS_INTERRUPT_PIN), tcsISR, FALLING);
  
  if (tcs.begin()) {  
    Serial.println("Found sensor"); 
  } else {  
    Serial.println("No TCS34725 found ... check your connections"); 
    while (1);  
  }
        
  // Set persistence filter to generate an interrupt for every RGB Cycle, regardless of the integration limits  
  tcs.write8(TCS34725_PERS, TCS34725_PERS_NONE); 
  tcs.setInterrupt(true);

  Serial.flush();
}

bool ColorSensor::canRun(uint32_t now)
{
  return TCS_STATE;
}

void ColorSensor::run(uint32_t now)
{
  // TODO: Check if ball is being detected or not
  // Get detected color
  uint16_t r, g, b, c;  
  getRawData_noDelay(&r, &g, &b, &c); 
  //Serial.print("R: "); Serial.print(r); 
  //Serial.print("\tG: "); Serial.print(g); 
  //Serial.print("\tB: "); Serial.print(b);

  int color = RGBToColor(r, g, b);

  //Serial.print("\tColor: "); Serial.println(color);

  if (color != -1) {
    targetColor = color;
  }
  
  // Clear interrupt flag
  tcs.clearInterrupt();
  TCS_STATE = false;
}

// Color sensor interrupt service routine
void tcsISR() 
{
  TCS_STATE = true;
}

/* tcs.getRawData() does a delay(Integration_Time) after the sensor readout.
We don't need to wait for the next integration cycle because we receive an interrupt when the integration cycle is complete*/
void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}

// Return whether two numbers are within the color value threshold of each other
bool ColorSensor::withinThreshold(int val1, int val2) {
  return (val1 <= val2 + colorValThreshold && val1 >= val2 - colorValThreshold);
}

// Convert RGB values to a color number from 0 to 5 based on the color map
int ColorSensor::RGBToColor(float r, float g, float b) {
  // Iterate through color map
  for (int i = 0; i < 7; ++i) {
    // Return matching color
    if (withinThreshold(r, colorMap[i][0]) &&
        withinThreshold(g, colorMap[i][1]) &&
        withinThreshold(b, colorMap[i][2])) {
      return min(i, 5); // Orange (color 6) also returns 5
    }
  }

  // No color found
  return -1;
}
//==================================================================

//==================================================================
// class KeyboardInput - Author: Dan
//  Uses pins:
//     None
// Status - Incomplete
//==================================================================
class KeyboardInput : public TriggeredTask
{
public:
  KeyboardInput(uint8_t _pin);
  virtual bool canRun(uint32_t now);
  virtual void run(uint32_t now);
  void printData(String input);

private:
  
};

KeyboardInput::KeyboardInput(uint8_t _pin) {
  data.jX = 127;
  data.jY = 127;
}

void KeyboardInput::run(uint32_t now)
{
  String input = Serial.readString();
  int input_size = input.length();
  input[input_size - 1] = '\0';

  if (input == "1") {
    data.button1 = true;
  }
  else if (input == "2") {
    data.button2 = true;
  }
  else if (input == "3") {
    data.button3 = true;
  }
  else if (input == "4") {
    data.button4 = true;
  }
  else if (input == "r") {
    data.rocker1 = !data.rocker1;
  }
  else if (input == "t") {
    data.rocker2 = !data.rocker2;
  }
  else if (input == "j") {
    data.jsButton = true;
  }
  else if (input == "UP") {
    data.jY = 255;
  }
  else if (input == "DOWN") {
    data.jY = 0;
  }
  else if (input == "LEFT") {
    data.jY = 0;
  }
  else if (input == "RIGHT") {
    data.jY = 255;
  }
  else if (input == "CENTER") {
    data.jX = 127;
    data.jY = 127;
  }
  
  printData(input);
}

void KeyboardInput::printData(String input) {
  String rocker1;
  String rocker2;
  
  if (data.rocker1 == true) {
    rocker1 = "Pickup Mode";
  } else {
    rocker1 = "Drop Mode";
  }

  if (data.rocker2 == true) {
    rocker2 = "Shooter Mode";
  } else {
    rocker2 = "Sorted Mode";
  }
  
  Serial.println("================================Current Control Status============================");
  Serial.print("    Input: "); Serial.println(input);
  Serial.print(" Button 1: ");  Serial.print(data.button1); Serial.print("\t");  Serial.print("Button 2: ");  Serial.println(data.button2);
  Serial.print(" Button 3: ");  Serial.print(data.button3); Serial.print("\t");  Serial.print("Button 4: ");  Serial.println(data.button4);
  Serial.print("JS Button: ");
  Serial.println(data.jsButton);
  Serial.println();
  
  Serial.print(" Rocker 1: ");  Serial.println(rocker1);
  Serial.print(" Rocker 2: ");  Serial.println(rocker2);
  Serial.print("     JS X: ");  Serial.print(data.jX); Serial.print("\t");  Serial.print("JS Y: ");  Serial.println(data.jY);
  Serial.println("==================================================================================");
}

bool KeyboardInput::canRun(uint32_t now)
{
  return (Serial.available() > 0);
}

//==================================================================

void setup() {
  Serial.begin(9600);

//  //-----------------------------------------
//  //RoboClaw initialization and settings
//  Serial3.begin(57600); // Wire communication with Roboclaw
//  //-----------------------------------------
//
//  //-----------------------------------------
//  //Stepper motor initialization and settings
//  ticSerial.begin(9600);
//  // Read the current color index from address 0 of the EEPROM
//  currentColor = EEPROM.read(0);
//  targetColor = currentColor; // Makes sure the stepper motor doesn't move to a target
//
//  delay(20); // Give time for stepper motor to start
//
//  // Set up stepper motor
//  tic.haltAndSetPosition(0);
//  tic.exitSafeStart();
//  //-----------------------------------------
}

void loop() {
  //--------------Scheduler Init-----------------
  //ColorSensor colorSensor;
  AugerMoveActuator augerMoveActuator;
  AugerRotateMotor augerRotateMotor(1);
  BarrelRotateStepper barrelRotateStepper(1, 1, 1, 1);
  DropBall dropBall(DROP_DOOR_PIN);
  BallShooter ballShooter;
  //Radio radio(1);
  KeyboardInput input(1);
  
  Task *tasks[] = {
    //&colorSensor,
    &augerMoveActuator,
    &augerRotateMotor,
    &barrelRotateStepper,
    &dropBall,
    &ballShooter,
    //&radio,
    &input
    //...Add task objects here
  };

  
  // ***
  // *** Instantiate the TaskScheduler and fill it with tasks.      
  // ***
  TaskScheduler scheduler(tasks, NUM_TASKS(tasks));
  
  // GO! Run the scheduler - it never returns.
  scheduler.runTasks();
  //--------------Scheduler End------------------

}
