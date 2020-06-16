#include<Wire.h>
#include<Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttionA;
Zumo32U4LCD lcd;

#define NUM_SENSOR 5
bool useEmitters = true;
uint16_t lineSensorValues[NUM_SENSOR];
uint16_t newdata;

void setup() {
  // put your setup code here, to run once:
  lineSensors.initFiveSensors();
}

void loop() {
  // put your main code here, to run repeatedly:
  lineSensors.read(lineSensorValues);
//  lcd.gotoXY(0, 1);
//  lcd.print(lineSensorValues[0]);
  newdata = map(lineSensorValues[0], 0, 1600, 0, 6);
  Serial.println(newdata);
  Serial.println("-----------------------");
  delay(1000);
}
