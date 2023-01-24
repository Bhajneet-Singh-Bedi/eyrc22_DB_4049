// These two has to be interrupt pins.
#include<MPU6050_light.h>

#define ENCA 19 //21-PD0 //19-RX1
#define ENCB 18 //20-PD1 //18-TX1
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B

MPU6050 mpu(Wire);
float alpha=0.0, theta=0.0, rpm=0.0, lastPos=NULL;
volatile long pos=0.0;
int prevMillis, currentMillis, oldPos=0, newPos=0;
double M=0.0,rads=0,prevRads=0.0, angAcc=0.0, trq=0.0;
void readEncoder(){
  //Serial.print("read encoder");
  int b=digitalRead(ENCB);
  if (b>0){
    pos++;
  }
  else{
    pos--;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(pwm, OUTPUT);
  digitalWrite(brake, LOW); // Low means braking.
  digitalWrite(cw, HIGH);
  analogWrite(pwm, 10);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  
}

void loop() {
  // put your main code here, to run repeatedly
  //Serial.println(pos);
  M=(100*0.0975*0.0975)/2;
  theta=(pos*360)/100;
//  Serial.print("Theta: ");
//  Serial.print(theta);
  mpu.update();
  alpha=mpu.getAngleX();
//  Serial.print("\tAlpha: ");
//  Serial.print(alpha);
  currentMillis=millis();
  newPos=pos;
  if (currentMillis-prevMillis >= 50){
    rpm=float((newPos-oldPos)*20*60/(currentMillis-prevMillis));
  }
  rads=9.549296585513721*rpm;
  // torque code.
  // Formula: T=MOI*angular acc.
  angAcc=(rads-prevRads)/(currentMillis-prevMillis);
  //rpm=float((newPos-oldPos)*1000/(currentMillis-prevMillis));
  oldPos=newPos;
  prevMillis=currentMillis; 
  prevRads=rads;
  delay(50);
  Serial.print("\tRPM: ");
  Serial.print(rpm);
  Serial.print("Ang ACC: ");
  Serial.print(angAcc);
  trq=M*angAcc;
  Serial.print("\tTorque: ");
  Serial.println(trq);
  

}
