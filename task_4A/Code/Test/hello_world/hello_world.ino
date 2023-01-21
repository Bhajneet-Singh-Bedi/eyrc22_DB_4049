#define CHA 0
#define CHB 2
#define cw 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(CHA, INPUT);
  pinMode(CHB, INPUT);
  //delay(1000);
  //Serial.print("Done");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("Hello");
  int a = digitalRead(CHA);
  int b = digitalRead(CHB);
  Serial.print(a);
  Serial.print(b);
  Serial.println();
}
