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

#define RATE_BLINKER_BLINK    500   //Blink LED_BLINKER - Timed Task

//===============Pin Definitions=================
#define CE_PIN   9
#define CSN_PIN 10
#define address2 0x80 // Address to Roboclaw
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
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
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
    ptrDebugger->debugWrite("BLINKER: OFF");
    // If the LED is off, turn it on and remember the state.
  } else {
    digitalWrite(pin, HIGH);
    on = true;
    //Send output to Serial Monitor via debugger
    ptrDebugger->debugWrite("BLINKER: ON");
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
  virtual void run(uint32_t schTime);
  
  private:
  
};

class BarrelRotateStepper : public TimedTask
{
  public:
  BarrelRotateStepper(short _currentColor, short _targetColor);
  virtual void run(uint32_t schTime);

  private:
  short currentColor; // The current index the motor is at
  short targetColor; // The target index for the motor to move to
};

BarrelRotateStepper::BarrelRotateStepper
(short _currentColor, short _targetColor, Debugger *_ptrDebugger){
  TimedTask(millis());
  currentColor(_currentColor);
  targetColor(_targetColor);
}

void BarrelRotateStepper::run(uint32_t schTime){
  short distance = targetColor - currentColor;

  // Moves the opposite direction if the distance is greater than 3
  if (distance > 3) {
    distance -= 6;
  }
  else if (distance < -3) {
    distance += 6;
  }

  int targetPos = tic.getCurrentPosition() + (distance * 33.0);
  tic.setTargetPosition(targetPos);
  
  waitForPosition(targetPos);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Stella Receiver Starting");

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
  //Stepper motor initialization and settings
  ticSerial.begin(9600);
  // Read the current color index from address 0 of the EEPROM
  currentColor = EEPROM.read(0);
  targetColor = currentColor; // Makes sure the stepper motor doesn't move to a target

  delay(20); // Give time for stepper motor to start

  // Set up stepper motor
  tic.haltAndSetPosition(0);
  tic.exitSafeStart();
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
}

void loop() {
  //--------------Scheduler Init-----------------
  Debugger debugger;
  Blinker example(LED_BUILTIN, RATE_BLINKER_BLINK, &debugger);
  
  Task *tasks[] = {
    &debugger,
    &example
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
