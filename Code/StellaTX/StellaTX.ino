//Program: Controller/Transmitter Code
//Function: Will run on arduino nano in Stella controller. Supports 4 buttons, 1 rocker switch
//          an analog joystick, and 5 display LEDs. Two directional communication with the receiving
//          arduino.
//Date: 2/24/2020

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

bool DEBUGGING = true;

//Controller/Nano GPIO Pin List:
//D2-4: Buttons
//D5: CE pin - radio
//D6: CSN pin - radio
//D7: Button
//D8: Rocker Switch
//D9-10: Free
//D11: Radio MOSI
//D12: Radio MISO
//D13: Radio SCK
//A0-4: LEDs (will be used as digital pins with digitalWrite(A0-4, val))
//A5: Joystick Button (will be used as a digital pin with digitalRead(A5))
//A6: Joystick Y Direction
//A7: Joystick x Direction

#define BUTTON_1 2
#define BUTTON_2 3
#define BUTTON_3 4
#define BUTTON_4 7
#define CE_PIN 5
#define CSN_PIN 6
#define ROCKER 8
#define R_MOSI 11
#define R_MISO 12
#define R_SCK 13

#define LED_1 A0
#define LED_2 A1
#define LED_3 A2
#define LED_4 A3
#define LED_5 A4
#define JOY_BUT A5
#define JOY_X A6
#define JOY_Y A7

const byte slaveAddress[5] = { 'R', 'x', 'A', 'A', 'A' };

RF24 radio(CE_PIN, CSN_PIN); // Create radio

bool newData = false;

//Contains control data to be send to Stella.
struct dataToSend {
  byte jX;
  byte jY;
  char pass[7];
  bool button1, button2, button3, button4, rocker, jsButton;
  byte ballColor;
};

dataToSend data;
dataToSend data_Last;

//Expected structure for acknowledgement data from Stella.
struct ackData {
  byte ballColor;
  bool ballDispensed;
};

ackData aData;

//Timing variables
unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 100; //Transmit frequency in ms, currently at 100 times/sec but can be optimized later.

//Debounce variables
unsigned long debounceDelay = 225;
unsigned long lastRead_But1 = 0;
unsigned long lastRead_But2 = 0;
unsigned long lastRead_But3 = 0;
unsigned long lastRead_But4 = 0;
unsigned long lastRead_JoyBut = 0;
unsigned long lastRead_Rocker = 0;
unsigned long debounceCurrent;
bool _button1, _button2, _button3, _button4, _rocker, _jsButton;

//enum for ballColor
enum color { Green, Blue, Red, Purple, Yellow, Orange, Pink };



//===========================================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("SimpleTxAckPayload Starting");

  //Set LEDs to OUTPUT mode
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);

  //Set buttons to INPUT_PULLUP mode. This will use the arduinos
  //built in pullup resistors to keep them at +5V while in OFF state
  //and invert logic so they can be read normally.
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_3, INPUT_PULLUP);
  pinMode(BUTTON_4, INPUT_PULLUP);
  pinMode(JOY_BUT, INPUT_PULLUP);
  pinMode(ROCKER, INPUT_PULLUP);

  //Set joysticks to INPUT
  pinMode(JOY_X, INPUT);
  pinMode(JOY_Y, INPUT);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);

  //Required for two way communication.
  radio.enableAckPayload();

  radio.setRetries(3, 5); // delay, count -- LOOK INTO
  radio.openWritingPipe(slaveAddress);

  //Acknowledged data - aData
  aData.ballColor = 1;
  aData.ballDispensed = true;

  //Received data - data;
  byte jX = 127;
  byte jY = 127;
  char pass[7] = { 'x', 'x', 'x', 'x', 'x', 'x', 'x' };
  bool button1 = false, button2 = false, button3 = false,
    button4 = false, rocker = false, jsButton = false;
  byte ballColor = 1;

}
//================================================================================================
//{Green, Blue, Red, Purple, Yellow, Orange, Pink};
//LED_1 A0
//LED_2 A1
//LED_3 A2
//LED_4 A3
//LED_5 A4
void updateLEDS() {
  if (aData.ballColor == Green) {
    digitalWrite(A0, LOW);
    digitalWrite(A1, HIGH);
    digitalWrite(A2, HIGH);
    digitalWrite(A3, HIGH);
    digitalWrite(A4, HIGH);
  }
  else if (aData.ballColor == Blue) {
    digitalWrite(A0, HIGH);
    digitalWrite(A1, LOW);
    digitalWrite(A2, HIGH);
    digitalWrite(A3, HIGH);
    digitalWrite(A4, HIGH);
  }
  else if (aData.ballColor == Red) {
    digitalWrite(A0, HIGH);
    digitalWrite(A1, HIGH);
    digitalWrite(A2, LOW);
    digitalWrite(A3, HIGH);
    digitalWrite(A4, HIGH);
  }
  else if (aData.ballColor == Purple) {
    digitalWrite(A0, HIGH);
    digitalWrite(A1, HIGH);
    digitalWrite(A2, HIGH);
    digitalWrite(A3, LOW);
    digitalWrite(A4, HIGH);
  }
  else if (aData.ballColor == Yellow) {
    digitalWrite(A0, HIGH);
    digitalWrite(A1, HIGH);
    digitalWrite(A2, HIGH);
    digitalWrite(A3, HIGH);
    digitalWrite(A4, LOW);
  }
  else if (aData.ballColor == Orange) {
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);
    digitalWrite(A2, LOW);
    digitalWrite(A3, LOW);
    digitalWrite(A4, LOW);
  }
  else if (aData.ballColor == Pink) {
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);
    digitalWrite(A2, LOW);
    digitalWrite(A3, LOW);
    digitalWrite(A4, LOW);
  }
  else {
    digitalWrite(A0, HIGH);
    digitalWrite(A1, HIGH);
    digitalWrite(A2, HIGH);
    digitalWrite(A3, HIGH);
    digitalWrite(A4, HIGH);
  }

}
//================================================================================================
//Should return false when data does not match old data
bool cmpData(dataToSend& data1, dataToSend& data2) {
  bool result = true;
  int xDiff = (int)data1.jX - (int)data2.jX;
  int yDiff = (int)data1.jY - (int)data2.jY;
  
  if (xDiff < -5 || xDiff > 5) {
//    Serial.println("jY\n");
//    Serial.println(yDiff);
//    Serial.println(data1.jY);
//    Serial.println(data2.jY);
    result = false;
  } else if (xDiff < -5 || xDiff > 5) {
//    Serial.println("jX\n");
//    Serial.println(xDiff);
//    Serial.println(data1.jX);
//    Serial.println(data2.jX);
    result = false;
  } else if (false) {
    Serial.println("jX\n");
    result = false;
  } else if (data1.button1 != data2.button1) {
    Serial.println("b1\n");
    result = false;
  } else if (data1.button2 != data2.button2) {
    Serial.println("b2\n");
    result = false;
  } else if (data1.button3 != data2.button3) {
    Serial.println("b3\n");
    result = false;
  } else if (data1.button4 != data2.button4) {
    Serial.println("b4\n");
    result = false;
  } else if (data1.rocker != data2.rocker) {
    Serial.println("rocker\n");
    result = false;
  } else if (data1.jsButton != data2.jsButton) {
    Serial.println("jsButton\n");
    result = false;
  } else if (data1.ballColor != data2.ballColor) {
    Serial.println("ballColor\n");
    result = false;
  } else {
    result = true;
  }

  return result;
}
//================================================================================================
void loop() {
  updateMessage();
    
  currentMillis = millis();
  if (currentMillis - prevMillis >= txIntervalMillis) {
    if (!cmpData(data_Last, data)) {
      Serial.println("New Data: Sending\n");
      send();
    }
    
    clearButtons();
  }

  updateLEDS();

  if (DEBUGGING) {
    showData();
  }
}
//===========================================================================================
void send() {
  //radio.write accepts the memory location of the data to be send to Stella and
  //the size in bytes of data to be sent. It returns TRUE if the data is acklowedged
  //by Stella.
  bool rslt;
  rslt = radio.write(&data, sizeof(data));

  if (rslt) {
    Serial.println("  Tx succeeded:\t");
    if (radio.isAckPayloadAvailable()) {
      radio.read(&aData, sizeof(ackData));
      newData = true;
    }
    else {
      if (DEBUGGING) {
        Serial.println("Acknowledge but no data ");
      }
    }
    //updateMessage();
  }
  else {      
    if (DEBUGGING) {
      Serial.println("  Tx failed");
    }
  }

  prevMillis = millis();
}
//===========================================================================================
void showData() {
  //DEBUGGING, not included in final code.
  if (newData == true) {
    Serial.print("Acknowledge data ");
    Serial.print(aData.ballColor);
    Serial.print(": ");
    Serial.println(aData.ballDispensed);
    newData = false;
  }
}
//===========================================================================================
void clearButtons() {
  data.button1 = false;
  data.button2 = false;
  data.button3 = false;
  data.button4 = false;
  data.jsButton = false;
}
//===========================================================================================
void updateMessage() {
  //Save the last data to compare before sending
  data_Last = data;
  
  //Prepare next data to be send to Stella. 
  data.jX = map(analogRead(JOY_X), 0, 1024, 0, 255);
  data.jY = map(analogRead(JOY_Y), 0, 1024, 0, 255);
  
  debounceCurrent = millis();
  if ((debounceCurrent - lastRead_But1 > debounceDelay)) {
    if (digitalRead(BUTTON_1) == LOW) {
      if (DEBUGGING) {
        Serial.println("Button 1 was pressed");
      }
      
      data.button1 = true;
      lastRead_But1 = millis();
    }
    else {
      data.button1 = false;
    }
  }

  debounceCurrent = millis();
  if ((debounceCurrent - lastRead_But2 > debounceDelay)) {
    if (digitalRead(BUTTON_2) == LOW) {
      if (DEBUGGING) {
        Serial.println("Button 2 was pressed");
      }
      
      data.button2 = true;
      lastRead_But2 = millis();
    }
    else {
      data.button2 = false;
    }
  }

  debounceCurrent = millis();
  if ((debounceCurrent - lastRead_But3 > debounceDelay)) {
    if (digitalRead(BUTTON_3) == LOW) {
      if (DEBUGGING) {
        Serial.println("Button 3 was pressed");
      }
      
      data.button3 = true;
      lastRead_But3 = millis();
    }
    else {
      data.button3 = false;
    }
  }

  debounceCurrent = millis();
  if ((debounceCurrent - lastRead_But4 > debounceDelay)) {
    if (digitalRead(BUTTON_4) == LOW) {
      if (DEBUGGING) {
        Serial.println("Button 4 was pressed");
      }
      data.button4 = true;
      lastRead_But4 = millis();
    }
    else {
      data.button4 = false;
    }
  }

  debounceCurrent = millis();
  if ((debounceCurrent - lastRead_JoyBut > debounceDelay)) {
    if (digitalRead(JOY_BUT) == LOW) {
      data.jsButton = true;
      lastRead_JoyBut = millis();
    }
    else {
      data.jsButton = false;
    }
  }

  bool newRockerRead = (digitalRead(ROCKER) ==  LOW);
    
  if (data.rocker == true && newRockerRead == false) {
    Serial.println("Rocker switched to OFF");
    data.rocker = newRockerRead;
  } else if (data.rocker == false && newRockerRead == true) {
    Serial.println("Rocker switched to ON");
    data.rocker = newRockerRead;
  }
}
