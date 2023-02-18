#include <util/atomic.h>

#define ENCA 2 //21-PD0 //19-RX1  //17-RX2
#define ENCB 3 //20-PD1 //18-TX1  //16-TX2
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 

// globals
long prevT=0;
int posPrev=0;
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


void setMotor(int dir, int pwr){

  if (dir == 1){
    digitalWrite(cw, HIGH);
  }
  else{
    digitalWrite(cw, LOW);
  }
//  Serial.println(255-pwr);
  digitalWrite(brake, HIGH);
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
  analogWrite(pwm, 255); // 255 means stop // 0 means go.
//  mpu.calcOffsets(true, true); 
//  M = 0.084*0.097*0.097/2; // Moment of Inertia.
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
//  Serial.println(pos_i);
  int pwr = 100/3.0*micros()/1.0e6;
  int dir = 1;
  setMotor(dir, pwr);

  int pos=0;
  float velocity2=0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2=velocity_i;
  }

//   Computing velocity with method 1
//  long currT = micros();
//  float deltaT = ((float) (currT-prevT))/1.0e6;
//  float velocity1 = (pos-posPrev)/deltaT;
//  posPrev=pos;
//  prevT = currT;  


  float v2 = velocity2/600.0*60.0;
//  Serial.print(velocity1);
//  Serial.print(" ");
  Serial.print(v2);
  Serial.println();
}
