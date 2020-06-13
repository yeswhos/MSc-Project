int sensor = 12;
int sensorRead = 0;
int newdata = 0;

void setup() {
  Serial.begin(9600);

}

void loop() {
  sensorRead = analogRead(sensor);
//  newdata = map(sensorRead, 0, 1023, 0, 100);
//  Serial.println(newdata);
//  analogWrite(13, newdata);
//  delay(200);
  Serial.println(sensorRead);
}
