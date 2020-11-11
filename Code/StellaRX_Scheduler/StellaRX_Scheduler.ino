//Stella Receiver

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
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
#define CE_PIN   9
#define CSN_PIN 10
#define DROP_DOOR_PIN 2
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
SoftwareSerial ticSerial(10, 11);
#endif
TicSerial tic(ticSerial);
//===============================================

//===============Color Sensor Globals============
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
const int tcsInterruptPin = 3;
//===============================================

// Each color compartment corresponds to a numerical ID from 1 to 6, with the index increasing as you go clockwise
// and looping back to 1 when you go clockwise from 6
short currentColor = 1; // The current index the motor is at
short targetColor = 1; // The target index for the motor to move to

struct dataReceived { // Data from the controller
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



//Only needed for debugging
int counter = 0;

//==================================================================
//======================dataReceived()==============================
void dataReceived_Interrupt() {
  //Serial.println("Interrupt called");
  MSSG = true;
  //getData();
}

/*****************************************************************************************
*  Class:    Debugger
* Task Type:  Task (always runs)
* Purpose:  This expands on Alan Burlison's original example code which demonstrates
*       a task that reads from the serial port and echoes to the Serial Monitor.
*       I've expanded it so that other classes use a pointer to the debugger object
*       to output simple debug messages while this example executes.
*       Classes that use the debugger object are passed a reference to &debugger
*       in their respective constructors.
*
*       For example: Blinker(uint8_t _pin, uint32_t _rate, Debugger *_ptrDebugger);
*
*       To output debug information use: ptrDebugger->debugWrite("debug info");
*
* Notes:    Yeah, I lazily used the String() function in this demonstration. Suedfbvbvfbfvvvvvvvb  me.            
******************************************************************************************/

// ***
// *** Define the Debugger Class as type Task
// ***
class Debugger : public Task
{
public:
  Debugger();
  void debugWrite(String debugMsg); //Used for simple debugging of other tasks
  virtual void run(uint32_t now);   //Override the run() method
  virtual bool canRun(uint32_t now);
};

// ***
// *** Debugger Constructor
// ***
Debugger::Debugger()
  : Task()
  {
    Serial.begin(57600);
  }

// ***
// *** Debugger::canRun() <--checked by TaskScheduler
// ***
bool Debugger::canRun(uint32_t now)
{
  return Serial.available() > 0;
}

// ***
// *** Debugger::run() <--executed by TaskScheduler as a result of canRun() returning true.
// ***
void Debugger::run(uint32_t now)
{
  uint16_t byteCount = 0;
  
  Serial.println("-----------------");
  Serial.println("Input Received...");
  Serial.println("-----------------");
  while (Serial.available() > 0) {
    int byte = Serial.read();
    Serial.print("'") ;
    Serial.print(char(byte));
    Serial.print("' = ");
    Serial.print(byte, DEC);
    Serial.println(" ");
    if (byte == '\r') {
      Serial.print('\n', DEC);
    }
    
    byteCount++;
  }
  
  Serial.println("-----------------");
  Serial.print("Bytes Received: "); Serial.println(String(byteCount));
  Serial.println("-----------------");
  
}

// ***
// *** Debugger::debugWrite() <--provides basic debug info from other tasks
// ***
void Debugger::debugWrite(String debugMsg)
{
  Serial.println(debugMsg);
}

//======================Example Task==============================
class Blinker : public TimedTask
{
public:
  // Create a new blinker for the specified pin and rate.
  Blinker(uint8_t _pin, uint32_t _rate, Debugger *_ptrDebugger);
  virtual void run(uint32_t now);
private:
  uint8_t pin;        // LED pin.
  uint32_t rate;        // Blink rate.
  bool on;          // Current state of the LED.
  Debugger *ptrDebugger;    // Pointer to debugger
};

// ***
// *** Blinker Constructor
// ***
Blinker::Blinker(uint8_t _pin, uint32_t _rate, Debugger *_ptrDebugger)
  : TimedTask(millis()),
  pin(_pin),
  rate(_rate),
  on(false),
  ptrDebugger(_ptrDebugger)
  {
    pinMode(pin, OUTPUT);     // Set pin for output.
  }

// ***
// *** Blinker::run() <--executed by TaskScheduler as a result of canRun() returning true.
// ***
void Blinker::run(uint32_t now)
{
  // If the LED is on, turn it off and remember the state.
  if (on) {
    digitalWrite(pin, LOW);
    on = false;
    //ptrDebugger->debugWrite("BLINKER: OFF");
    // If the LED is off, turn it on and remember the state.
  } else {
    digitalWrite(pin, HIGH);
    on = true;
    //Send output to Serial Monitor via debugger
    //ptrDebugger->debugWrite("BLINKER: ON");
  }
  // Run again in the specified number of milliseconds.
  incRunTime(rate);
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
class AugerMoveActuator : public TimedTask
{
public:
  AugerMoveActuator(/**/);
  virtual void run(uint32_t now);

  
private:
  
};
//==================================================================

//==================================================================
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
  dropDoor.write(180);
  delay(500);
  dropDoor.write(0);
}

bool DropBall::canRun(uint32_t now) {
  return BARREL_IN_POSITION;
}
//==================================================================

//===================BarrelRotateStepper============================
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
  return ROTATE_BARREL;
}

void BarrelRotateStepper::waitForPosition(int32_t targetPosition) {
  do {
    tic.resetCommandTimeout();
  } while (tic.getCurrentPosition() != targetPosition);
}
//==================================================================

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
class ColorSensor : public TriggeredTask
{
public:
  virtual bool canRun(uint32_t now);
  virtual void run(uint32_t now);
};

bool ColorSensor::canRun(uint32_t now)
{
  return TCS_STATE;
}

void ColorSensor::run(uint32_t now)
{
  // TODO: Check if ball is being detected or not

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

void setup() {
  Serial.begin(9600);
  Serial.println("Stella Receiver Starting");

  // Start color sensor interrupt
  attachInterrupt(digitalPinToInterrupt(tcsInterruptPin), tcsISR, FALLING);
  tcs.setInterrupt(true);
//
//  //-----------------------------------------
//  //Radio initialization and settings
//  radio.begin();
//  radio.setDataRate( RF24_250KBPS );
//  radio.openReadingPipe(1, thisSlaveAddress);
//  radio.enableAckPayload();
//  radio.startListening();
//  radio.writeAckPayload(1, &aData, sizeof(ackData)); // pre-load data
//  attachInterrupt(digitalPinToInterrupt(2), dataReceived_Interrupt, FALLING);
//  Serial.println("Radio is starting");
//  //-----------------------------------------
//
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
//
//  //-----------------------------------------
//  //Color Sensor initialization and settings
//  // Ensures that the color sensor is connected
//  if (tcs.begin()) { // begin() starts the color sensor
//    Serial.println("Found sensor");
//  } else {
//    Serial.println("No TCS34725 found ... check your connections");
//    while (1); // halt!
//  }
//  //-----------------------------------------
}

void loop() {
  //--------------Scheduler Init-----------------
  Debugger debugger;
  Blinker example(LED_BUILTIN, RATE_BLINKER_BLINK, &debugger);
  DropBall dropBall(DROP_DOOR_PIN);
  Radio radio(1);
  
  Task *tasks[] = {
    &debugger,
    &example,
    &dropBall,
    &radio
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
