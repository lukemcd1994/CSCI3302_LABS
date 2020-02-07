#include <Sparki.h>

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long timeBeforeMove = millis();
  sparki.clearLCD();
  sparki.moveRight(180);
  unsigned long totTime = millis() - timeBeforeMove;
  float s = totTime / 1000.0;
  float spd = ((3.1415) * (.04285))/(s);
  sparki.clearLCD();

  
  sparki.print("speed of rotation: ");
  sparki.println(spd);
  sparki.updateLCD();
  delay(10000);
}
