#include<pnew.cpp>

#include<Wire.h>
#include<Zumo32U4.h>
//#include <ArduinoSTL.h>
#include<map>
using namespace std;
Zumo32U4LCD lcd;
Zumo32U4ProximitySensors proxSensors;

//arx::map<String, int> mp {{"one", 1}, {"two", 2}};
//arx::map<int, int> mp {{1, 10}, {2, 20}};
//arx::map<int, int> mp;
//arx::map <int, map<int, int> > multiMap {1, {2, 20}};
//
//arx::map<int, int> mps;
uint16_t newdata;

struct TestMap {

  static void RunTest() {

    std::ohserialstream serial(Serial);

    std::map<int,const char *> days;
    int i;

    days[1]="Monday";
    days[2]="Tuesday";
    days[3]="Wednesday";
    days[4]="Thursday";
    days[5]="Friday";
    days[6]="Saturday";
    days[7]="Sunday";

    for(i=1;i<7;i++)
      serial << days[i] << std::endl;
  }
};

void setup() {
  // put your setup code here, to run once:
  //std::ohserialstream serial(Serial);
  proxSensors.initFrontSensor();
//  std::map<int, int> m;
//  m[1] = 10;
////  std::map<int, std::map<int, int> > multiMap;
////  multiMap[2] = m;
//  Serial.println(m[1]);
//  serial << multiMap[2] << std::endl;
//  m.insert(pair<int, int>(1, 10));
//  m.insert(pair<int, int>(2, 20));
//  mp.insert(1, 10);
//  multiMap.insert(2, mp);
  //Serial.println(multiMap);
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
