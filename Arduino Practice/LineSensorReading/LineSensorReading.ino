#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;

#define NUM_SENSOR 5
uint8_t selectedSensorIndex = 0;

uint16_t lineSensorValues[NUM_SENSOR];
uint8_t counter = 0;

void setup() {
  // put your setup code here, to run once:
  lineSensors.initFiveSensors();
  lcd.clear();
}


void loop() {
  // put your main code here, to run repeatedly:
  lineSensors.read(lineSensorValues);
  lcd.gotoXY(0, 1);
  lcd.print(selectedSensorIndex);
  lcd.print(F(": "));
  lcd.print(map(lineSensorValues[selectedSensorIndex], 0, 2000, 0, 2));
  lcd.print(F("    "));
  if (buttonA.getSingleDebouncedPress())
  {
    counter = counter + 2;
    if(counter == 6){
      counter = 0;
    }
    
    selectedSensorIndex = counter;
  }
}
