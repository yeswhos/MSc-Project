#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;

uint16_t lineSensorValues[5];
uint16_t lineSensor1;
uint16_t lineSensor2;
uint16_t lineSensor3;

const uint16_t sensorThreshold = 800;

boolean warnRight = false;
boolean warnLeft = false;
boolean warnCenter = false;

void setup() {
  // put your setup code here, to run once:
  lcd.clear();
  lcd.print(F("Press A"));
  buttonA.waitForButton();
  lcd.clear();
  lineSensors.initFiveSensors();
  
}

void loop() {
  Serial.begin(9600);
  lineSensors.read(lineSensorValues);
  lineSensor1 = lineSensorValues[0];
  lineSensor2 = lineSensorValues[2];
  lineSensor3 = lineSensorValues[4];
  //Serial.println(lineSensor1);
  warnRight = (lineSensor1 <= sensorThreshold) && (lineSensor3 >= sensorThreshold);
  warnLeft = (lineSensor1 >= sensorThreshold) && (lineSensor3 <= sensorThreshold);
  warnCenter = (lineSensor1 <= sensorThreshold) && (lineSensor3 <= sensorThreshold);
  do{
    lineSensors.read(lineSensorValues);
    lineSensor1 = lineSensorValues[0];
    lineSensor2 = lineSensorValues[2];
    lineSensor3 = lineSensorValues[4];
    Serial.println(warnRight);
    Serial.println(lineSensor1);
  }while(warnRight);
//  while(warnRight){
//    lineSensors.read(lineSensorValues);
//    Serial.println(warnRight);
//    Serial.println(lineSensor1);
//  }
  if(warnLeft){
    Serial.println("左侧");
  }
  else if(warnCenter){
    Serial.println("中间");
  }
  Serial.println("直走");
}
