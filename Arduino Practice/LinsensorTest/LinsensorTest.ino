#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;

#define NUM_SENSOR 5
bool useEmitters = true;
uint8_t selectedSensorIndex = 0;

uint16_t lineSensorValues[NUM_SENSOR];

void setup() {
  // put your setup code here, to run once:
  lineSensors.initFiveSensors();
}

void printReadingsToLCD()
{

  // Print "E" if the emitters are on, "e" if they are off.
  lcd.gotoXY(7, 0);
  lcd.print(useEmitters ? 'E' : 'e');

  // On the second line of the LCD, display one raw reading.
  lcd.gotoXY(0, 1);
  lcd.print(selectedSensorIndex);
  lcd.print(F(": "));
  lcd.print(lineSensorValues[selectedSensorIndex]);
  lcd.print(F("    "));
}

void loop() {
  // put your main code here, to run repeatedly:
  //lineSensors.read();
//  int16_t position = lineSensors.readLine(lineSensorValues);
  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
  printReadingsToLCD();
  if (buttonA.getSingleDebouncedPress())
  {
    selectedSensorIndex = (selectedSensorIndex + NUM_SENSOR - 1) % NUM_SENSOR;
  }
}
