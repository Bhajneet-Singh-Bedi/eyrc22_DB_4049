// These two has to be interrupt pins.
#include<MPU6050_light.h>



#define ENCA 19 //21-PD0 //19-RX1
#define ENCB 18 //20-PD1 //18-TX1
//#define brake 8
#define cw 4
#define pwm 9
#define brake 49
MPU6050 mpu(Wire);
float pos=0.0,alpha=0.0, theta=0.0, rpm=0.0;

void readEncoderA(){
  //Serial.print("read encoder");
  delay(50);
  int b=digitalRead(ENCB);
  if (b>0){
    pos++;
  }
  else{
    pos--;
  }
}
//void readEncoderB(){
//  //Serial.print("read encoder");
//  int a=digitalRead(ENCA);
//  if (a>0){
//    pos--;
//  }
//  else{
//    pos++;
//  }
//}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(brake, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(cw, OUTPUT);
//  pinMode(PL0, OUTPUT);
  digitalWrite(PL0, LOW);
  digitalWrite(brake, LOW);
  digitalWrite(cw, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoderA, RISING);
}

void loop() {
  // put your main code here, to run repeatedly
  theta=(pos*360)/100;
//  Serial.print("Theta: ");
//  Serial.print(theta);
//  mpu.update();
//  alpha=mpu.getAngleX();
//  Serial.print("\tAlpha: ");
//  Serial.println(alpha);
  //delay(2000);
//  analogWrite(pwm,200);
  
  //Serial.print("PWM: ");
  //Serial.print(analogRead(pwm));
  rpm=(pos*60)/100;
  Serial.print("\tRPM: ");
  Serial.println(rpm);
}
