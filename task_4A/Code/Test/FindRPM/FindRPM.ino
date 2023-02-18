#include <util/atomic.h>

#define ENCA 2 //21-PD0 //19-RX1  //17-RX2
#define ENCB 3 //20-PD1 //18-TX1  //16-TX2
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 

// Global variables.
long prevT=0; int posPrev=0; volatile int pos_i=0;
volatile float velocity_i=0;
volatile long prevT_i=0;

//int thet, thet_dot, pos, alp, alp_dot, tm=1, curr=0, prev=0, prev_alp, sz=4;
//float K[4]={}, y[4]={}, y_setpoint[4]={}, trq;
//float rp, nw, prev_nw, dtt;
void readEncoder(){
  //Serial.print("read encoder");
  int b=digitalRead(ENCB);
  int incr=0;
  if (b>0){
    incr=1;
//    pos_i++;
  }
  else{
    incr=-1;
//    pos_i--;
  }
  pos_i+=incr;
//  long currT = micros();
//  float deltaT = ((float) (currT-prevT_i))/1.0e6;
//  velocity_i =incr/deltaT;
//  prevT_i=currT; 
}

void setMotor(int dir, float pwr){
  if (dir == 1){
    digitalWrite(cw, HIGH);
    digitalWrite(brake, HIGH);
  }
  else{
    digitalWrite(cw, LOW);
    digitalWrite(brake, HIGH);
  }
  analogWrite(pwm, 255-pwr);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(pwm, OUTPUT);
//  digitalWrite(ENCA, HIGH);
  digitalWrite(brake, LOW); // Low means braking.
//  digitalWrite(cw, HIGH); // gives positive value // Low will give negative
//  analogWrite(pwm, 255); // 255 means stop // 0 means go.
//  mpu.calcOffsets(true, true); 
//  M = 0.084*0.097*0.097/2; // Moment of Inertia.
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  int pwr = 100/3.0*micros()/1.0e6;
  setMotor(1, pwr);
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
  }

  // Compute velocity with method 1
  long currT=micros();
  float deltaT=((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos-posPrev)/deltaT;
  posPrev=pos;
  prevT = currT;

  Serial.print(velocity1);
  Serial.println();
}
