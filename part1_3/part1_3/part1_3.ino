#include <Sparki.h>

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  sparki.clearLCD();
  sparki.moveForward(30);
  int totTime = millis();
  float s = totTime * 1000.0;
  float m = (30*1.0)/(100*1.0);
  float spd = m/s;
  sparki.print("speed: ");
  sparki.println(spd);
}
