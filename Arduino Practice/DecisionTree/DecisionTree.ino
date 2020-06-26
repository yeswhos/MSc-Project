#include <ArduinoJson.h>
#include<Wire.h>
#include<Zumo32U4.h>

Zumo32U4ProximitySensors proxSensors;
Zumo32U4LCD lcd;

//char json[] = "{\"Front left Sensor Left\": [{\"0\": \"Hit it\", \"1\": [{\"Front Sensor Right\": [{\"0\": \"run away\", \"1\": \"hit it\", \"2\": \"hit it\", \"3\": \"Hit it\"}]}], \"2\": [{\"Front Sensor Right\": [{\"0\": \"run away\", \"1\": \"run away\", \"2\": \"Hit it\", \"3\": \"Hit it\"}]}], \"3\": [{\"Front Sensor Right\": [{\"0\": \"run away\", \"1\": \"run away\", \"2\": \"run away\", \"3\": \"hit it\"}]}]}]}";
//char json[] = "{\"sensor\": [{\"0\": \"gostright\"}, {\"1\": \"goright\"}]}";
//doc["sensor"][0]["0"]
void setup() {
  // put your setup code here, to run once:
  lcd.clear();
}

void loop() {
  Serial.begin(9600);
  Serial.println("begin");
  while (!Serial) continue;
  
  //StaticJsonDocument<2000> doc;
  const size_t capacity = 0 + 390;
  DynamicJsonDocument doc(capacity);  
  char json[] = "{\"Front left Sensor Left\": [{\"0\": \"Hit it\", \"1\": [{\"Front Sensor Right\": [{\"0\": \"run away\", \"1\": \"hit it\", \"2\": \"hit it\", \"3\": \"Hit it\"}]}], \"2\": [{\"Front Sensor Right\": [{\"0\": \"run away\", \"1\": \"run away\", \"2\": \"Hit it\", \"3\": \"Hit it\"}]}], \"3\": [{\"Front Sensor Right\": [{\"0\": \"run away\", \"1\": \"run away\", \"2\": \"run away\", \"3\": \"hit it\"}]}]}]}";
//  int *p = malloc(sizeof(json));
//  DynamicJsonDocument doc(*p);
  //char json[] = "{\"sensor\": [{\"0\": \"gostraight\"}, {\"1\": \"goright\"}]}";
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  Serial.println("here");
  //const char* value = doc["sensor"][0]["0"];
  const char* value = doc["Front left Sensor Left"][0]["0"];
  Serial.println(value);
}
