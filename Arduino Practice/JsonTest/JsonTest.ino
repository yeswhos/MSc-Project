#include <ArduinoJson.h>
void setup() {
  Serial.begin(9600);
  while (!Serial) continue;
  DynamicJsonDocument doc(1024);
  // put your setup code here, to run once:
  //char json[] = "{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}";
//  unsigned int json[] = "{"no surfacing": {"0": "NO", "1": {"flippers": {"0": "NO", "1": "YES"}}}}"
//  char json[] = "{\"sensor\": {\"0\": NO, \"1\": {\"sensor2\": {\"0\": Yes, \"1\": no}}}}";
  char json[] = "{\"sensor\": [{\"0\": \"gostright\"}, {\"1\": 2}]}";
  //StaticJsonDocument<200> doc;
  
  //deserializeJson(doc, json);
  DeserializationError error = deserializeJson(doc, json);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  Serial.println("here");
  const char* sensor = doc["sensor"][0]["0"];
//  long time          = doc["time"];
//  double latitude    = doc["data"][0];
//  double longitude   = doc["data"][1];
  Serial.println(sensor);
}

void loop() {
  // put your main code here, to run repeatedly:

}
