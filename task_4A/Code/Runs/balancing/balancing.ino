#include <MPU6050_light.h>

#define ENCA 2 //21-PD0 //19-RX1  //17-RX2
#define ENCB 3 //20-PD1 //18-TX1  //16-TX2
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B
#define pi = 3.141;

MPU6050 mpu(Wire);

float pos, thet, thet_dot, alp, alp_dot, prev_alp=0, currentTM, prevTM=0, dtt=0, rpm;
float y[4]={}, y_setpoint[4]={} , cntr=1, vll, trq, sz=4, M, angAc;
int K[4]={};
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

int lqr_controller(float yy[], float yy_setpoint[]){
  float mul=0;
  K[0] = -49.71888;K[1]=-7.37913;K[2]=-0.70711;K[3]=-0.91972;
  for (int i=0; i<sz; i++){
    mul=mul-(K[i]*(yy[i]-yy_setpoint[i]));
  }
  return mul;

}


int st_trq(int tr){
  // Take the trq value convert it to ang acc.
  angAc=tr/M;
  
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
//  digitalWrite(ENCA, HIGH);
  digitalWrite(brake, LOW); // Low means braking.
//  digitalWrite(cw, HIGH); // gives positive value // Low will give negative
  analogWrite(pwm, 255); // 255 means stop // 0 means go.
//  mpu.calcOffsets(true, true); 
  M = (0.134*0.097*0.097)/2;
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {

  
  // This is alpha
  // 0.0174533
  alp = (pos*360)/100;
//  Serial.print(alp);
  // alpha_dot, change in theta of the reaction wheel.
  currentTM = millis();
  dtt = currentTM-prevTM;

  if (dtt>10){
    alp_dot = (alp-prev_alp)/dtt;
    prev_alp=alp;
    prevTM=currentTM;
  }
  
  // Theta.
  mpu.update();
  thet = mpu.getAngleY();
  // Theta_dot.
  thet_dot = mpu.getGyroY();

  // Now we will get the value of torque required from the lqr controller.
  y[0]=thet; y[1]=thet_dot; y[2]=alp;  y[3]=alp_dot;
  y_setpoint[0]=0;y_setpoint[1]=0;y_setpoint[2]=0;y_setpoint[3]=0;
  trq = lqr_controller(y, y_setpoint);
//  Serial.print("TRQ: ");
//  Serial.println(trq);

  // Finnding rpm of the motor.
  rpm = (alp*60)/100;
//  Serial.printoln(alp);
  // Now use this trq val to provide torque to my motor.
//  st_trq(trq);
  
}