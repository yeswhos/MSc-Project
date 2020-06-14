#include <Zumo32U4.h>
Zumo32U4LCD lcd;
Zumo32U4ProximitySensors proxSensors;

void setup() {
  proxSensors.initThreeSensors();
}

void loop() {
  proxSensors.read();
  lcd.clear();
  lcd.gotoXY(0,1);
  lcd.print(proxSensors.countsLeftWithLeftLeds() );
  lcd.gotoXY(2,0);
  lcd.print(proxSensors.countsFrontWithLeftLeds() );
  lcd.gotoXY(5,0);
  lcd.print(proxSensors.countsFrontWithRightLeds() );
  lcd.gotoXY(7,1);
  lcd.print(proxSensors.countsRightWithRightLeds() );
  delay(100);
}
