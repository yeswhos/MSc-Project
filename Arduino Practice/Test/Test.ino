#include <Zumo32U4.h>
#include<Wire.h>
Zumo32U4LCD lcd;
Zumo32U4ProximitySensors proxSensors;
uint8_t frontleft;

void setup() {
  proxSensors.initFrontSensor();
  proxSensors.initThreeSensors();
}

void loop() {
  proxSensors.read();
  lcd.clear();
  lcd.gotoXY(0,0);
  /*  frontleft = map(proxSensors.countsLeftWithLeftLeds(), 0, 6, 0, 3);
  lcd.print(frontleft);*/
  lcd.print(proxSensors.countsLeftWithLeftLeds());
  lcd.gotoXY(2,0);
  frontleft = map(proxSensors.countsFrontWithLeftLeds(), 0, 6, 0, 3);
  lcd.print(frontleft);
  lcd.gotoXY(5,0);
  lcd.print(proxSensors.countsFrontWithRightLeds() );
  lcd.gotoXY(7,0);
  lcd.print(proxSensors.countsRightWithRightLeds() );
  delay(100);
}
