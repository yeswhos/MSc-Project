#include <ArduinoJson.h>
#include<Wire.h>
#include<Zumo32U4.h>

Zumo32U4ProximitySensors proxSensors;
Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

const uint16_t turnSpeedMax = 400;

// The minimum speed to drive the motors while turning.  400 is
// full speed.
const uint16_t turnSpeedMin = 100;

// The amount to decrease the motor speed by during each cycle
// when an object is seen.
const uint16_t deceleration = 10;

// The amount to increase the speed by during each cycle when an
// object is not seen.
const uint16_t acceleration = 10;

#define LEFT 0
#define RIGHT 1

// Stores the last indication from the sensors about what
// direction to turn to face the object.  When no object is seen,
// this variable helps us make a good guess about which direction
// to turn.
bool senseDir = RIGHT;

// True if the robot is turning left (counter-clockwise).
bool turningLeft = false;

// True if the robot is turning right (clockwise).
bool turningRight = false;

// If the robot is turning, this is the speed it will use.
uint16_t turnSpeed = turnSpeedMax;




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
//  turningLeft = true;
//  turningRight = true;
}

void hitIt(){
  motors.setSpeeds(200, 200);
//  turningLeft = true;
//  turningRight = true; 
}
void stopIt(){
  motors.setSpeeds(0, 0);
  turningLeft = true;
  turningRight = true; 
}
void loop() {
  uint16_t bettery = readBatteryMillivolts();
  lcd.gotoXY(0, 0);
  lcd.print(bettery);
  Serial.begin(9600);
  //Serial.println("begin");
  while (!Serial) continue;
  const size_t capacity = 0 + 390;
  DynamicJsonDocument doc(capacity);  
  char json[] = "{\"Front left Sensor Left\": [{\"0\": \"Hit it\"}, {\"1\": [{\"0\": \"run away\", \"1\": \"hit it\", \"2\": \"hit it\", \"3\": \"Hit it\"}]}, {\"2\": [{\"0\": \"run away\", \"1\": \"run away\", \"2\": \"Hit it\", \"3\": \"Hit it\"}]}, {\"3\": [{\"0\": \"run away\", \"1\": \"run away\", \"2\": \"run away\", \"3\": \"hit it\"}]}]}";
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  //Serial.println("here");
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
  Serial.println(*value);
  if(*value == 'H'){
    d = 0;  
    Serial.println("here1");
  }else{
    d = 1;
    Serial.println("here2");  
  }
  lcd.gotoXY(0, 1);
  lcd.print(' ');
  lcd.print(d);
  
  switch (d){
    case 0:
      Serial.println("switch");
      hitIt();
      delay(1000);
      stopIt();
      break;
    case 1:
      Serial.println("switch2");
      runAway(); 
      delay(1000);
      stopIt();
      break;
  }
  delay(1000);
}
