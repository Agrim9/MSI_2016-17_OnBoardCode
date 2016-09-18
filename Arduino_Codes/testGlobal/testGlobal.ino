int myVar = 10;

void myCB() {
  myVar = 20;  
}

void setup() {
  Serial.begin(9600);
  myCB();
}

void loop() {
  Serial.println(myVar);
  delay(10);
}
