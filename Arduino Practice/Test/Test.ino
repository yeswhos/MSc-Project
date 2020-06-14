
#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;

//Zumo32U4LineSensors lineSensors;
//uint16_t lineSensorsValues[5] = {0, 0, 0, 0, 0};

bool proxLeftActive;
bool proxFrontActive;
bool proxRightActive;

void setup()
{
//  proxSensors.initFrontSensor();
//  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();
  
  lcd.clear();
  lcd.print(F("Press A"));
  buttonA.waitForButton();
  lcd.clear();
  loadCustomCharacters();

  calibrateLineSensors();
}

void calibrateLineSensors()
{
  // To indicate we are in calibration mode, turn on the yellow LED
  // and print "Line cal" on the LCD.
  ledYellow(1);
  lcd.clear();
  lcd.print(F("Line cal"));

  for (uint16_t i = 0; i < 400; i++)
  {
    lcd.gotoXY(0, 1);
    lcd.print(i);
//    lineSensors.calibrate();
  }

  ledYellow(0);
  lcd.clear();
}

void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  lcd.loadCustomCharacter(levels + 0, 0);  // 1 bar
  lcd.loadCustomCharacter(levels + 1, 1);  // 2 bars
  lcd.loadCustomCharacter(levels + 2, 2);  // 3 bars
  lcd.loadCustomCharacter(levels + 3, 3);  // 4 bars
  lcd.loadCustomCharacter(levels + 4, 4);  // 5 bars
  lcd.loadCustomCharacter(levels + 5, 5);  // 6 bars
  lcd.loadCustomCharacter(levels + 6, 6);  // 7 bars
}


void loop()
{

//  proxSensors.read();
//  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
//  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();
  proxSensors.read();
  proxLeftActive = proxSensors.readBasicLeft();
  proxFrontActive = proxSensors.readBasicFront();
  proxRightActive = proxSensors.readBasicRight();
  
//  Serial.println(LeftValue);
//  Serial.println(RightValue);
//  Serial.println(FrontValue);

  lcd.gotoXY(0, 0);
  lcd.print(proxLeftActive);
  lcd.print(' ');
  lcd.print(proxFrontActive);
  lcd.print(' ');
  lcd.print(proxRightActive);
}
