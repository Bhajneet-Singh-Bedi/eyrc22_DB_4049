#include <MPU6050_light.h>
#define ENCA 2 //21-PD0 //19-RX1  //17-RX2
#define ENCB 3 //20-PD1 //18-TX1  //16-TX2
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B

MPU6050 mpu(Wire);
int thet, thet_dot, pos, alp, alp_dot, tm=1, curr, prev=0, prev_alp, sz=4;
float K[4]={}, y[4]={}, y_setpoint[4]={}, trq;


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

float lqrController(float yy[], float yy_setpoint[]){
  float mul=0;
//    K = [-311.4268,-85.9813,-1.0000,-1.7637];
  K[0]=-41.4663;K[1]=-6.4185;K[2]=-1.0000;K[3]=-1.3120;
//  K[0]=-49.32124;K[1]=-7.25900;K[2]=-1.00000;K[3]=-1.29629;
//  K[0]=-49.91982;K[1]=-7.31929;K[2]=-1.00000;K[3]=-1.29515;
//  K[0]=-69.90391;K[1]=-10.38223;K[2]=-1.00000;K[3]=-1.29991;
//  K[0]=-69.97770;K[1]=-10.38361;K[2]=-1.00000;K[3]=-1.30019;
//  K[0]=-69.89653;K[1]=-10.38210;K[2]=-1.00000;K[3]=-1.2998;4
//  K[0]=-77.60897;K[1]=-10.52474;K[2]=-1.00000;K[3]=-1.32904;
//  K[0] = -22.78308;K[1]=-3.37165;K[2]=-0.31623;K[3]=-0.41236;
//  K[0] = -49.71888;K[1]=-7.37913;K[2]=-0.70711;K[3]=-0.91972;
//    K[0]=-69.90391;  K[1]=-10.38223;   K[2]=-1.00000;   K[3]=-1.29991;
//    K[0]=-40.77681;   K[1]=-6.04874;   K[2]=-0.57735;   K[3]=-0.75130;
//    K[0]=-193.01827;   K[1]=-25.14391;    K[2]=-1.00000;    K[3]=-1.26103;
//    K[0]=-136.66472;   K[1]=-17.80188;    K[2]=-0.70711;    K[3]=-0.89180;
//    K[0]=-281.02199;   K[1]=-35.01541;    K[2]=-0.70711;    K[3]=-0.88350;
//    K[0]=-217.99154;   K[1]=-27.49369;    K[2]=-0.70711;    K[3]=-0.88573;
//    K[0]=-147.40983;   K[1]=-18.80253;    K[2]=-0.57735;    K[3]=-0.72496;
//    K[0]=-147.40983;   K[1]=-18.80253;    K[2]=-0.70711;    K[3]=-0.72496;
//    K[0]=-26.01250;   K[1]=-3.31595;   K[2]=-0.10000;   K[3]=-0.12580;
//    K[0]=-28.07205;   K[1]=-3.65106;   K[2]=-0.14142;   K[3]=-0.17865;
//    K[0]=-28.47513;   K[1]=-3.65753;   K[2]=-0.14142;   K[3]=-0.17913;
  for (int i=0; i<sz; i++){
//    Serial.println(i);
    mul=mul-(K[i]*(yy[i]-yy_setpoint[i]));
  }
  return mul;

}

void setTorque(){

//  T = M * angular accleration
  rp = ((tm*trq*dtt)/M)-prev_rp;  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets(true, true);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(pwm, OUTPUT);
//  digitalWrite(ENCA, HIGH);
  digitalWrite(brake, LOW); // Low means braking.
//  digitalWrite(cw, HIGH); // gives positive value // Low will give negative
  analogWrite(pwm, 255); // 255 means stop // 0 means go.
  mpu.calcOffsets(true, true); 
//  M = 0.084*0.097*0.097/2; // Moment of Inertia.
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu.update();
  thet = -mpu.getAngleY();
//  Serial.println(thet);
  // Iterate for some fixed time then note down thet_dot.
  curr = millis();
  alp = pos*360/100;
//  Serial.println(alp);
  float dtt = curr-prev;
  if (dtt>tm){
    thet_dot = mpu.getGyroY(); // degrees
    alp_dot = (prev_alp-alp)/dtt;
    y[0]=thet; y[1]=thet_dot; y[2]=alp;  y[3]=alp_dot;
    y_setpoint[0]=0;y_setpoint[1]=0;y_setpoint[2]=0;y_setpoint[3]=0;
    trq = lqrController(y, y_setpoint);
    setTorque();
//    Serial.println(trq);
    prev=curr;
  }
  
  
 // END
}
