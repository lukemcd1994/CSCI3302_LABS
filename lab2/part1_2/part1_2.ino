/*******************************************
 Sparki Line-following example
 
 Threshold is the value that helps you 
 determine what's black and white. Sparki's 
 infrared reflectance sensors indicate white 
 as close to 900, and black as around 200.
 This example uses a threshold of 500 for 
 the example, but if you have a narrow line, 
 or perhaps a lighter black, you may need to 
 adjust.
********************************************/
 
#include <Sparki.h> // include the sparki library
#include <math.h>
float wheel_speed = .0278;
int threshold = 500;
unsigned long loopStartTime;
unsigned long loopStopTime;
float x = 0;
float y = 0;
float theta = 0;
float diameter = .0857;

void setup() 
{
  
}


void update(){

}




void loop() {
 
  loopStartTime = millis();
 
  int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
  int lineCenter = sparki.lineCenter(); // measure the center IR sensor
  int lineRight  = sparki.lineRight();  // measure the right IR sensor
  
 
  if ( lineLeft < threshold ) // if line is below left line sensor
  {  
    sparki.moveLeft(); // turn left
    
  }
 
  if ( lineRight < threshold ) // if line is below right line sensor
  {  
    sparki.moveRight(); // turn right
  }
 
  // if the center line sensor is the only one reading a line
  if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
  {
    
    sparki.moveForward(); // move forward
  }  
 
  sparki.clearLCD(); // wipe the screen
 
  sparki.print("Line Left: "); // show left line sensor on screen
  sparki.println(lineLeft);
 
  sparki.print("Line Center: "); // show center line sensor on screen
  sparki.println(lineCenter);
 
  sparki.print("Line Right: "); // show right line sensor on screen
  sparki.println(lineRight);
 
  sparki.updateLCD(); // display all of the information written to the screen
  
  loopStopTime = millis();

  float arc_length = wheel_speed * .1;
  theta = theta + arc_length/(diameter/2);
  x = cos(theta)*(wheel_speed * .1);
  y = sin(theta)*(wheel_speed * .1);
  sparki.print("x: ");
  sparki.println(x);
  sparki.print("y: ");
  sparki.println(y);
  sparki.print("theta: ");
  sparki.println(theta);

   
  
  delay(100 - (loopStopTime - loopStartTime)); //wait < .1 seconds
}
