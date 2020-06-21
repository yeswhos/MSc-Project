#include<Wire.h>
#include<Zumo32U4.h>
#include<ArxContainer.h>

Zumo32U4LCD lcd;
Zumo32U4ProximitySensors proxSensors;

//arx::map<String, int> mp {{"one", 1}, {"two", 2}};
//arx::map<int, int> mp {{1, 10}, {2, 20}};
arx::map<int, int> mp;
arx::map <int, mp > multiMap {1, {2, 20}};

arx::map<int, int> mps;
uint16_t newdata;

void setup() {
  // put your setup code here, to run once:
  proxSensors.initFrontSensor();
//  map<int, int> m;
//  m.insert(pair<int, int>(1, 10));
//  m.insert(pair<int, int>(2, 20));
//  mp.insert(1, 10);
//  multiMap.insert(2, mp);
  Serial.println(multiMap);
}

void loop() {
  // put your main code here, to run repeatedly:
  proxSensors.read();
  newdata = proxSensors.countsFrontWithLeftLeds();

//  Serial.println(mp[newdata]);
  
  Serial.println("-----------------------");
  Serial.println(newdata);
  Serial.println("-----------------------");
  delay(1000);
}
