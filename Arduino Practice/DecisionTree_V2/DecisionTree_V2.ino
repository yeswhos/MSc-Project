#include <ArduinoJson.h>
#include<Wire.h>
#include<Zumo32U4.h>

Zumo32U4ProximitySensors proxSensors;
Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

void setup() {
  // put your setup code here, to run once:
  proxSensors.initFrontSensor();
  lcd.clear();
  lcd.print(F("Press A"));
  buttonA.waitForButton();
  lcd.clear();
}

void runAway()
{
  motors.setSpeeds(-200, -200);
}

void hitIt(){
  motors.setSpeeds(200, 200);
}
void stopIt(){
  motors.setSpeeds(0, 0);
}

void turnRight()
{
  motors.setSpeeds(100, -100);
}

void turnLeft()
{
  motors.setSpeeds(-100, 100);
}

void loop() {

  Serial.begin(9600);
  while (!Serial) continue;
  const size_t capacity = 0 + 390;
  DynamicJsonDocument doc(capacity);  
  char json[] = "{\"Contact\": [{\"0\": [{\"0\": [{\"0\": \"No object seeing\", \"1\": \"No object seeing\", \"2\": \"Object seeing\", \"3\": \"Object seeing\"}], {\"1\": [{\"0\": \"No object seeing\", \"1\": \"No object seeing\"}, \"2\": \"Object seeing\", \"3\": \"Object seeing\"}]}, {\"2\": \"Object seeing\"}, {\"3\": \"Object seeing\"} }], {\"1\": \"Charge\"}]}";
  //char json[] = "{\"Front left Sensor Left\": [{\"0\": \"Hit it\"}, {\"1\": [{\"0\": \"run away\", \"1\": \"hit it\", \"2\": \"hit it\", \"3\": \"Hit it\"}]}, {\"2\": [{\"0\": \"run away\", \"1\": \"run away\", \"2\": \"Hit it\", \"3\": \"Hit it\"}]}, {\"3\": [{\"0\": \"run away\", \"1\": \"run away\", \"2\": \"run away\", \"3\": \"hit it\"}]}]}";
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  Serial.println("here");
  
  proxSensors.read();
  uint8_t a = map(proxSensors.countsFrontWithLeftLeds(), 0, 6, 0, 3);
  uint8_t b = map(proxSensors.countsFrontWithRightLeds(), 0, 6, 0, 3);
  //Serial.println(a);
  uint8_t d;
  const char *value;
  //Serial.println(value);
  if (a == 0){
    value = doc["Front left Sensor Left"][0]["0"];
    //Serial.println("0");
  }else{
    value = doc["Front left Sensor Left"][a][(String) a][b][(String) b];
    //Serial.println(">0");
    
  }
  //Serial.println(*value);
  if(*value == 'H'){
    if(a < b){
      turnRight();  
    }
    hitIt();
    delay(100);
    stopIt();  
    //Serial.println("here1");
  }else{
    if(a > b){
      turnLeft();
    } 
    runAway(); 
    delay(100);
    stopIt();
    //Serial.println("here2");  
  }
//  lcd.gotoXY(0, 1);
//  lcd.print(' ');
//  lcd.print(d);
//  
//  switch (d){
//    case 0:
//      Serial.println("switch");
//      hitIt();
//      delay(1000);
//      stopIt();
//      break;
//    case 1:
//      Serial.println("switch2");
//      runAway(); 
//      delay(1000);
//      stopIt();
//      break;
//  }
  lcd.gotoXY(0, 0);
  lcd.print(a);
  lcd.print(' ');
  lcd.print(b);
  delay(10);
}
