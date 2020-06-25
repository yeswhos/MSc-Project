#include <ArduinoJson.h>
#include<Wire.h>
#include<Zumo32U4.h>

Zumo32U4ProximitySensors proxSensors;
Zumo32U4LCD lcd;

char json[] = "{\"sensor\": [{\"0\": \"gostright\"}, {\"1\": \"goright\"}]}";
uint8_t value;
void setup() {
  proxSensors.initFrontSensor();
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.begin(9600);
  while (!Serial) continue;
  DynamicJsonDocument doc(1024);
  // put your setup code here, to run once:
  //char json[] = "{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}";
//  unsigned int json[] = "{"no surfacing": {"0": "NO", "1": {"flippers": {"0": "NO", "1": "YES"}}}}"
//  char json[] = "{\"sensor\": {\"0\": NO, \"1\": {\"sensor2\": {\"0\": Yes, \"1\": no}}}}";
  char json[] = "{\"sensor\": [{\"0\": \"gostright\"}, {\"1\": \"goright\"}]}";
  //StaticJsonDocument<200> doc;
  
  //deserializeJson(doc, json);
  DeserializationError error = deserializeJson(doc, json);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  proxSensors.read();
  lcd.gotoXY(0, 0);
  value = proxSensors.countsFrontWithLeftLeds();
  lcd.print(value);
  const char* sensor = doc["sensor"][value]["0"];
  Serial.println(sensor);
  delay(1000);
  const char* sensor2 = doc["sensor"][value]["1"];
  Serial.println(sensor2);
//  long time          = doc["time"];
//  double latitude    = doc["data"][0];
//  double longitude   = doc["data"][1];
  
}
