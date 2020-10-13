
bool RotateAuger = false;
bool ColorSense = false;
bool RotareBarrel = false;

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


void setup() {
  storeColor = Red ;
  dropColor = Purple;

 

  

    
  

  
}

void loop() {
  // put your main code here, to run repeatedly:

}
