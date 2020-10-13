
enum penStates {
  Retracted,
  Extended
} pen;

void clickPress() {
  if (pen == Retracted) {
    pen = Extended;
  }
  else {
    pen = Retracted;
  }
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(2), clickPress, CHANGE);

}

void loop() {

}
