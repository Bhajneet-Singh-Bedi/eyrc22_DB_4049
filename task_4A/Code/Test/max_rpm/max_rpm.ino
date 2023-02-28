// Max rpms with all weights comes out to be around 3200, with battery of around 11.7 volts.

#include <util/atomic.h>

#define ENCA 2 //21-PD0 //19-RX1
#define ENCB 3 //20-PD1 //18-TX1
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B
#define pi = 3.141;


float prev = 0, K[4]={}, y[4]={}, y_setpoint[4]={}, M;
float thet, thet_dot, alp, alp_dot, prevAlp=0, vGive, prev_u=0;
long prevT=0;
int posPrev=0,  sz=4, trq;
volatile int pos_i=0; 
volatile float velocity_i=0;
volatile long prevT_i=0;


void readEncoder(){
  int b=digitalRead(ENCB);
  int increment=0;
  if (b>0){
    increment=1;
//    pos_i++;
  }
  else{
    increment=-1;
//    pos_i--;
  }
  pos_i=pos_i+increment;
//  Serial.println(pos_i);

  long currT=micros();
  float deltaT=((float) (currT-prevT_i))/1.0e6;
  velocity_i=increment/deltaT;
  prevT_i=currT;
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//  Serial.print("HII");
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(pwm, OUTPUT);
  digitalWrite(brake, HIGH); // Low means braking.
  digitalWrite(cw, HIGH); // gives positive value // Low will give negative
  analogWrite(pwm, 255);
  M = 0.134*0.097*0.097/2;
//  Serial.print("hi");
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  
  float curr = micros();
  // radians - 0.10471975512
  alp = (pos_i*360)/100; // radians
  float dtt = curr - prev;
  prev = curr;
  alp_dot = (alp-prevAlp)/dtt;
  prevAlp=alp;

  int pos=0;
  float velocity2=0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2=velocity_i;
  }

  float v2 = velocity2/100.0*60.0;
  Serial.println(v2);
  digitalWrite(brake, HIGH);
  analogWrite(pwm, 0);
}
