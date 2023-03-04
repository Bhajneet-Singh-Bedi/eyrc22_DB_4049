// 9, 11, 13, 14, 15
#include <Servo.h>

#define sen0 A9
#define sen1 A11
#define sen2 A13
#define sen3 A14
#define sen4 A15

Servo hServo;
Servo dServo;
int poss=0, threshold;
float v0, v1, v2, v3, v4, y;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(sen0, INPUT);
  pinMode(sen1, INPUT);
  pinMode(sen2, INPUT);
  pinMode(sen3, INPUT);
  pinMode(sen4, INPUT);
  hServo.attach(7);
  dServo.attach(6);
  threshold = 650;
}

void loop() {
  // put your main code here, to run repeatedly:
  v0=analogRead(sen0);
  v1=analogRead(sen1);
  v2=analogRead(sen2);
  v3=analogRead(sen3);
  v4=analogRead(sen4);

  // So, it seems like black value is like between 400-500 and white value is around 850-950.
  // So, probably threshold would be around 650 or so.
  // Zero degrees is left side when back wheel is near and front wheel is far from me. Let's write the code.
  // Try using a delay while giving motor the angle.
  
  // Straight line code down here. lalalala
  straight_line();

  
  // Start  code will be here detecting telling the bike to detect the starting line and go on.
  start_line();
  
}

void straight_line(){
  // Straight line code.
  if (v0<=threshold && v1>=threshold && v2>=threshold && v3>= threshold && v4<=threshold){
    poss = 90;
  }
  else if (v0<=threshold && v1>=threshold && v2>=threshold && v3<= threshold && v4<=threshold){
    poss = 105;
  }
  else if (v0<=threshold && v1<=threshold && v2>=threshold && v3>= threshold && v4<=threshold){
    poss = 75;
  }
  else if (v0>=threshold && v1>=threshold && v2>=threshold && v3<= threshold && v4<=threshold){
    poss = 105;
  }
  else if (v0<=threshold && v1<=threshold && v2>=threshold && v3>= threshold && v4>=threshold){
    poss = 75;
  }
  else if (v0>=threshold && v1>=threshold && v2<=threshold && v3<= threshold && v4<=threshold){
    poss = 120;
  }
  else if (v0<=threshold && v1<=threshold && v2<=threshold && v3>= threshold && v4>=threshold){
    poss = 60;
  }
  // Straight line code.

  // This is start bike code.
  else if (v0>=threshold && v1>=threshold && v2>=threshold && v3>= threshold && v4>=threshold){
    //start the motor.
    //provide some delay.
  }

  // Delivery node code.
  else if (v0<=threshold && v1>=threshold && v2>=threshold && v3>= threshold && v4>=threshold){
    //It means delivery node has been achieved.
    //Increase the counter.
    //This will specify that the bike has reached the node.
  }

  // 

  // Giving it to servo.
  if (hServo.read()<=poss){
    for (int i=hServo.read(); i<=poss; i++){
      hServo.write(poss);
      delay(15);    
    }
  }
  else if (hServo.read()>=poss){
    for (int i=hServo.read(); i>=poss; i--){
      hServo.write(poss);
      delay(15);
    }
  }
}

//void start_line(){
//  if (v0>=threshold && v1>=threshold && v2>=threshold && v3>= threshold && v4>=threshold){
//    // motor has started.
//
//    // give power to the motor and also the power will be given in the starting too, this is just for precautions. 
//    // Provide some delay to.
//    
//  }
//}
