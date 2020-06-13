//最先执行
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

//会一直做下面的事情
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);                       
  digitalWrite(LED_BUILTIN, LOW);    
  delay(1000);                      
}
