//最先执行 LED_BUILTIN
void setup() {
  pinMode(13, OUTPUT);
}

//会一直做下面的事情
void loop() {
  digitalWrite(13, HIGH);
  delay(100);                       
  digitalWrite(13, LOW);    
  delay(300);                      
}
