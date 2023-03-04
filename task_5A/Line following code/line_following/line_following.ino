// 9, 11, 13, 14, 15
#include <Servo.h>

#define sen0 A9
#define sen1 A11
#define sen2 A13
#define sen3 A14
#define sen4 A15

Servo myservo;

int poss=0, p_poss=90;

float value0, value1, value2, value3, value4, y;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(sen0, INPUT);
  pinMode(sen1, INPUT);
  pinMode(sen2, INPUT);
  pinMode(sen3, INPUT);
  pinMode(sen4, INPUT);
  myservo.attach(7);
}

void loop() {
  // put your main code here, to run repeatedly:
  value0=analogRead(sen0);
  value1=analogRead(sen1);
  value2=analogRead(sen2);
  value3=analogRead(sen3);
  value4=analogRead(sen4);
//  Serial.print(analogRead(sen1));
//  Serial.print("  ");
//  Serial.print(analogRead(sen2));
//  Serial.print("  ");
//  Serial.print(analogRead(sen3));
//  Serial.print("  ");
//  Serial.print(analogRead(sen4));
//  Serial.print("  ");
//  Serial.println(analogRead(sen5));

//  y = (0 * value0 + 1000 * value1 + 2000 * value2 + 3000 * value3 + 4000 * value4) / (value0 + value1 + value2 + value3 + value4);
  Serial.println(value0); /*Serial.print("   "); Serial.print(value2); Serial.print("   "); Serial.println(value3);*/
//  Serial.println(y);

//  if (value1>=900 && value3<=900){
//    pos=120;
//  }
////  else if (value
//  else {
//    pos=90;
//  }
//  pos=90;
//

// The threshold seems to be 920.
// So above it start turning the motor .
// Else keep the motor to 90 degrees.
// Change the angle of the motor in a for loop with a 5 ms delay so that the bike doesn't fall. (hahahahaha)

//  if (value0<=920 && value1>=920 && value2>=920 && value3<=920 && value4<=920){
//    poss=120;
//  }
//  else if (value0<=920 && value1<=920 && value2>=920 && value3>=920 && value4<=920)){
//    poss=70;
//  }
//  else{
//    poss=90;
//  }

//  Serial.print(digitalRead(sen1)); Serial.print("  "); Serial.print(digitalRead(sen2)); Serial.print("  "); Serial.println(digitalRead(sen3));
//  if (myservo.read()<=poss){
//    for (int i=myservo.read(); i<=poss; i++){
//    myservo.write(poss);  
//    delay(5);
//    }
//  }
//  else {
//    for (int i=myservo.read(); i>=poss; i--){
//    myservo.write(poss);  
//    delay(5);
//  }
//  }
  myservo.write(50);
  
}
