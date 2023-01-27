#define ENCA 19
#define ENCB 18
#define cw 11
#define pwm 9
#define brake 48


int var1 = 0;
int lstVal = 0;
int temp, mil_now_5, mil_then_5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  //pinMode(ENCA, INPUT);
  
  //pinMode(ENCA, INPUT);
  //pinMode(ENCB, INPUT);
  //pinMode(cw, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(cw, OUTPUT);
  //pinMode(brake, OUTPUT);
  pinMode(brake, OUTPUT);
  digitalWrite(brake, HIGH);
  digitalWrite(cw, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly


  mil_now_5=millis();
  if (mil_now_5-mil_then_5>=5000){
    analogWrite(pwm, 255-temp);
    temp+=5;
    mil_then_5=mil_now_5;
  }
  Serial.print("Temp: ");
  Serial.println(temp);
  //Serial.println("Hello World");
//  int a = analogRead(pwm);
//  int b = digitalRead(ENCB);
//  //digitalWrite(cw, HIGH);
//  //digitalWrite(brake,LOW);
//  //analogWrite(pwm, 250);
//  if (a <100){
//    var1=0; 
//  }
//  else{
//    var1=1;
//  }
//
//  if (var1 == lstVal) {
//    
//  }
//  else {
//    lstVal = var1;
//    Serial.println(var1);
//  }

  
  
  
//  Serial.print(b);
//  Serial.println();
}
