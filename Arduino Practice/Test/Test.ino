#include <Zumo32U4.h>
#include<Wire.h>
Zumo32U4LCD lcd;
Zumo32U4ProximitySensors proxSensors;
uint8_t frontleft;

void setup() {
  proxSensors.initFrontSensor();
  //proxSensors.initThreeSensors();
}

void loop() {
  proxSensors.read();
  lcd.clear();
  //lcd.gotoXY(0,0);
  /*  frontleft = map(proxSensors.countsLeftWithLeftLeds(), 0, 6, 0, 3);
  lcd.print(frontleft);*/
  //lcd.print(proxSensors.countsLeftWithLeftLeds());
//  lcd.print(map(proxSensors.countsLeftWithLeftLeds(), 0, 6, 0, 3) );
  //Display position
  lcd.gotoXY(2,0); 
  //Store the sensor reading, and scale down the value from 0 - 6 to 0 - 3
  frontleft = map(proxSensors.countsFrontWithLeftLeds(), 0, 6, 0, 3); 
  //Display the sensor reading
  lcd.print(frontleft); 
  lcd.gotoXY(5,0);
  //lcd.print(proxSensors.countsFrontWithRightLeds() );
  lcd.print(map(proxSensors.countsFrontWithRightLeds(), 0, 6, 0, 3) );
  //lcd.gotoXY(7,0);
  //lcd.print(proxSensors.countsRightWithRightLeds() );
  //lcd.print(map(proxSensors.countsRightWithRightLeds(), 0, 6, 0, 3) );
  delay(100);
}
