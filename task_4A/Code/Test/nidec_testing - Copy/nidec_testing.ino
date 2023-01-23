#define ENCA 0
#define ENCB 1
#define cw 2
#define pwm A0
#define brake 4


int var1 = 0;
int lstVal = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  //pinMode(ENCA, INPUT);
  
  //pinMode(ENCA, INPUT);
  //pinMode(ENCB, INPUT);
  //pinMode(cw, OUTPUT);
  pinMode(pwm, INPUT);
  //pinMode(brake, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly
  //Serial.println("Hello World");
  int a = analogRead(pwm);
//  int b = digitalRead(ENCB);
//  //digitalWrite(cw, HIGH);
//  //digitalWrite(brake,LOW);
//  //analogWrite(pwm, 250);
  if (a <100){
    var1=0; 
  }
  else{
    var1=1;
  }

  if (var1 == lstVal) {
    
  }
  else {
    lstVal = var1;
    Serial.println(var1);
  }

  
  
  
//  Serial.print(b);
//  Serial.println();
}
