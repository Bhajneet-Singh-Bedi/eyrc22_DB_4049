#define ENCA 0
#define ENCB 1
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly
  int a = digitalRead(ENCA);
  int b = digitalRead(ENCB);
  Serial.print(a);
  Serial.print(b);
  Serial.println();
}
