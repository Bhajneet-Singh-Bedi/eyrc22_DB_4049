/*
1 - 
2 - 
3 - 
4 - 0b
5 - 3a
6 - 4a
7 - 4b
8 - 4c
9 - 2b
*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(1) == HIGH){
    Serial.println("One");
  }
  else if (digitalRead(2) == HIGH){
    Serial.println("Two");
  }
  else if (digitalRead(3) == HIGH){
    Serial.println("Three");
  }
  else if (digitalRead(4) == HIGH){
    Serial.println("Four");
  }
  else if (digitalRead(5) == HIGH){
    Serial.println("Five");
  }
  else if (digitalRead(6) == HIGH){
    Serial.println("Six");
  }
  else if (digitalRead(7) == HIGH){
    Serial.println("seven");
  }
  else if (digitalRead(8) == HIGH){
    Serial.println("Eight");
  }
  else if (digitalRead(9) == HIGH){
    Serial.println("Nine");
  }
}
