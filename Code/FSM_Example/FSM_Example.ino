
enum penStates {
  Retracted,
  Extended
} pen = Retracted;

void clickPress() {
  if (pen == Retracted) {
    pen = Extended;
  }
  else {
    pen = Retracted;
  }
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(2), clickPress, HIGH);

}

void loop() {

}
