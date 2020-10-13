
bool RotateAuger = false;
bool ColorSense = false;
bool RotateBarrel = false;

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

 bool rotateAugerUp() {
  if (!ColorSense) {
    if (!RotateBarrel) {
      //Rotate Auger
    }
  }
  return true;
}

bool rotateAugerDown() {
  if (!ColorSense) {
    if (!RotateBarrel) {
      //Rotate Auger
    }
  }
  return true;
}

bool getColor() {
  if (!RotateAuger) {
    if (!RotateBarrel) {
      //Call Color sensor
    }
  }
}


void setup() {
  storeColor = Red ;
  dropColor = Purple;

}

void loop() {
  // put your main code here, to run repeatedly:

}
