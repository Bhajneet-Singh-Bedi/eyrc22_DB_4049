#define ENCA 0
#define ENCB 1
#define cw 2
#define pwm A0
#define brake 4
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  
  //pinMode(ENCA, INPUT);
  //pinMode(ENCB, INPUT);
  //pinMode(cw, OUTPUT);
  //pinMode(pwm, OUTPUT);
  //pinMode(brake, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly
  //Serial.println("Hello World");
  int a = digitalRead(ENCA);
//  int b = digitalRead(ENCB);
//  //digitalWrite(cw, HIGH);
//  //digitalWrite(brake,LOW);
//  //analogWrite(pwm, 250);
  Serial.print(a);
//  Serial.print(b);
//  Serial.println();
}
