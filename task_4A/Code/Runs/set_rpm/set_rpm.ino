#include <MPU6050_light.h>

#define ENCA 2 //21-PD0 //19-RX1  //17-RX2
#define ENCB 3 //20-PD1 //18-TX1  //16-TX2
#define brake 41 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B
#define pi = 3.141;

MPU6050 mpu(Wire);

float pos, po, thet, thet_dot, alp, alp_dot, prev_alp=0, prev_pos, currentTM, prevTM=0, dtt=0, rp, prev_rp=0, deltaT=0, prev_u=0;
float y[4]={}, y_setpoint[4]={} , cntr=1, vll, trq, sz=4, M, del_pos, angAc, prev_vel=0, vel_now, vel_give, target_pos, p_pos;
//int pos=0;
long prevT=0;
float eprev=0, eIntegral=0, dts=0;
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
  //  K = [-311.4268,-85.9813,-1.0000,-1.7637];
//  K[0]=-41.4663;K[1]=-6.4185;K[2]=-1.0000;K[3]=-1.3120;
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
    K[0]=-28.47513;   K[1]=-3.65753;   K[2]=-0.14142;   K[3]=-0.17913;
  for (int i=0; i<sz; i++){
    mul=mul-(K[i]*(yy[i]-yy_setpoint[i]));
  }
  return mul;

}

int set_rpm(){
  if (trq>0){
    if (prev_u<0){
      digitalWrite(brake, LOW);
    }
    else{
      digitalWrite(brake, HIGH);
    }
    char dir = "HIGH";
    setMotor(dir);
  }
  else if (trq<=0){
    if (prev_u>0){
      digitalWrite(brake, LOW);
    }
    else{
      digitalWrite(brake, HIGH);
    }
    char dir = "LOW";
    setMotor(dir);
  }
  prev_u=trq;
}
int setMotor(char *dir){
//  Serial.println(trq);
  int target_vel = 1000;// Going to make in interchangable from the serial monitor, later on.
  if (trq>=-2 && trq<=2){
    target_vel=0;
  }
  long currT = micros();
  float delT = ((float)(currT-prevT))/1.0e6;
  prevT = currT;
  int pos_needed = ((target_vel*delT*100)/0.10471975512) + p_pos;
  Serial.println(pos_needed);
  // PID constants
  float kp=1, kd=0, ki=0;

  // error
  int e = pos-pos_needed;
  
  // derivative
  float dedt = (e-eprev)/delT;

  // Integral
  eIntegral = eIntegral + e*dts;

  // Control Signal
  float u = kp*e + kd*dedt + ki*eIntegral;
//  Serial.println(u);
  float pws = fabs(u);
  if (pws>255){
    pws=255;
  }

  char drr = "HIGH";
  if (u<0){
    char drr = LOW;
  }
  eprev=e;
//  Serial.print(drr);
//  Serial.print("PWS: ");
//  Serial.println(pws);
  analogWrite(cw, drr);
  analogWrite(pwm, pws);
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
  mpu.calcOffsets(true, true); 
  M = 0.084*0.097*0.097/2; // Moment of Inertia.
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}


void loop() {


  // Theta.
  mpu.update();
  thet = mpu.getAngleY()*0.0174533; // radians
  // Theta_dot.
  thet_dot = mpu.getGyroY()*0.0174533; // radians
//  Serial.println(thet);
    // This is alpha
  // 0.0174533
  alp = (pos*360*0.10471975512)/100; // radians
//  Serial.print(alp);
  // alpha_dot, change in theta of the reaction wheel.
  currentTM = micros();
  dtt = currentTM-prevTM;
  dts = dtt/1.0e6;

  if (dtt>100){
    alp_dot = (alp-prev_alp)/dtt; // radians
    // Finnding rpm of the motor.
    del_pos = pos-prev_pos; // In ppr
    rp = (del_pos*0.10471975512)/(dts*100); // This will probably give me rpms in degrees for coverting it to radians(angular velocity) mult it with 0.10471975512
    // And to degrees mult. with 57.29578
    vel_now = rp;
    prev_alp=alp;
    p_pos = prev_pos; // to be used later.
    prev_pos=pos;
    prevTM=currentTM;
  

  
  // Now we will get the value of torque required from the lqr controller.
  y[0]=thet; y[1]=thet_dot; y[2]=alp;  y[3]=alp_dot;
  y_setpoint[0]=0;y_setpoint[1]=0;y_setpoint[2]=0;y_setpoint[3]=0;
  
  trq = lqr_controller(y, y_setpoint);

  set_rpm();
//  Serial.print("TRQ: ");
//  Serial.println(trq);

  }
  // Now use this trq val to provide torque to the motor.
//  st_trq(trq);
  prev_vel=vel_now;
  p_pos = prev_pos;
}
