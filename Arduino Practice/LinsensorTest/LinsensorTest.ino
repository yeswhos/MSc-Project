#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4LCD lcd;

uint16_t lineSensorValues[5] = { 1, 0, 0, 0, 0 };

void setup() {
  // put your setup code here, to run once:
  lineSensors.initFiveSensors();
  lcd.clear();
}

void loop() {
  // put your main code here, to run repeatedly:
  //lineSensors.read();
  int16_t position = lineSensors.readLine(lineSensorValues);
  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print(position);
  lcd.print(' ');
//  lcd.gotoXY(3, 0);
//  lcd.print(lineSensors.readLine(lineSensorValues[3]));
//  lcd.print(' ');
//  lcd.gotoXY(5, 0);
//  lcd.print(lineSensors.readLine(lineSensorValues[5]));
//  lcd.print(' ');
  delay(100);
}
