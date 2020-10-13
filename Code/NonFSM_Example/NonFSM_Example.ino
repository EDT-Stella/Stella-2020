
bool screwRotateUp;
bool screwRotateDown;
bool senseColor;

enum Color {

enum penStates {
  Retracted,
  Extended
} pen;

void setup() {
  pinMode(2, INPUT);
}

void loop() {
  bool clickPress = digitalRead(2);

  if (clickPress == true) {
    if (pen == Retracted) {
      pen = Extended;
    }
    else {
      pen = Retracted;
    }
  }
}
