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

// Barrel Stepper Motor interrupt flag.
volatile byte ROTATE_BARREL = false;

//Radio new data flag
bool newData = false;
//===============================================


//===============Radio Globals===================
const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
RF24 radio(CE_PIN, CSN_PIN);
//===============================================

//===============RoboClaw Globals================
RoboClaw roboclaw(&Serial3, 10000); // Create a Roboclaw object
bool actuatorForward = true;
bool motorForward = true;
//===============================================

//===============Stepper Motor Globals===========
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define ticSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial ticSerial(STEPPER_RX, STEPPER_TX);
#endif
TicSerial tic(ticSerial);
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
dataReceived data; // this must match dataToSend in the TX

void printData() {
  Serial.print("Button 1: ");
  Serial.println(data.button1);
  Serial.print("Button 2: ");
  Serial.println(data.button2);
  Serial.print("Button 3: ");
  Serial.println(data.button3);
  Serial.print("Button 4: ");
  Serial.println(data.button4);
  Serial.print("JS Button: ");
  Serial.println(data.jsButton);

  Serial.print("Rocker 1: ");
  Serial.println(data.rocker1);
  Serial.print("Rocker 2: ");
  Serial.println(data.rocker2);
  
  Serial.print("JS X: ");
  Serial.println(data.jX);
  Serial.print("JS Y: ");
  Serial.println(data.jY);
}

struct ackData { //Acknowledgement data
  byte ballColor;
  bool ballDispensed;
};
ackData aData; //Acknowledgement data to be returned to controller

//Only needed for debugging
int counter = 0;

//======================dataReceived()==============================
void dataReceived_Interrupt() {
  //Serial.println("Interrupt called");
  MSSG = true;
  //getData();
}
//==================================================================


//======================AugerRotateMotor==============================
class AugerRotateMotor : public TimedTask
{
public:
  AugerRotateMotor(/**/);
  virtual void run(uint32_t schTime);

  private:
};
//==================================================================


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

  if (data.rocker2 == true && data.button1 == true) {
    canDrop = true;
  }
  
  return canDrop;
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
  SoftwareSerial ticSerial(pin1, pin2);
  TicSerial tic(ticSerial);
  currentColor = _currentColor;
  targetColor = _targetColor;
}

void BarrelRotateStepper::run(uint32_t now){
  short distance = targetColor - currentColor;
  Serial.print("Color moved by: ");
  Serial.println(distance);

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
}

bool BarrelRotateStepper::canRun(uint32_t now) {
  bool canRun = false;

  if (data.rocker1 == true && data.rocker2 == false) {
    if (data.button2 == true) {
      canRun = true;
      data.button2 = false;
      if (targetColor == NUM_COLORS - 1) {
        targetColor = 0;
      } else {
        targetColor++;
      }
    }
    if (data.button3 == true) {
      canRun = true;
      data.button3 = false;
      if (targetColor == 0) {
        targetColor = NUM_COLORS - 1;
      } else {
        targetColor--;
      }
    }
  }
  
  return canRun;
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
  //Serial.print("\tB: "); Serial.println(b);

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

private:
  
};

KeyboardInput::KeyboardInput(uint8_t _pin) {
  //Serial.begin(9600);
  Serial.println("Stella Receiver Starting");
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
  Serial.println(input);
  printData();
}

bool KeyboardInput::canRun(uint32_t now)
{
  return (Serial.available() > 0);
}

//==================================================================

void setup() {
  Serial.begin(9600);
  Serial.print("Print");

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
  DropBall dropBall(DROP_DOOR_PIN);
  Radio radio(1);
  KeyboardInput input(1);
  
  Task *tasks[] = {
    //&colorSensor,
    &augerMoveActuator,
    &dropBall,
    &radio,
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
