int pushButton = 14;
int buttonState = 0;
int beforeState = 0;
int counter = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input pin:
  buttonState = digitalRead(pushButton);
  // print out the state of the button:
  if(buttonState == 1 && beforeState == 0){
    counter++;
    Serial.print("Serial Output: ");
    Serial.print(counter);
    Serial.println(buttonState);
    delay(1); 
  }
  if(buttonState != beforeState){
    delay(20);
  }
  beforeState = buttonState;
}
