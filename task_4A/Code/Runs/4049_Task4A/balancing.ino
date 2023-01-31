#include <MPU6050_light.h>

#define ENCA 2 //21-PD0 //19-RX1  //17-RX2
#define ENCB 3 //20-PD1 //18-TX1  //16-TX2
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B
#define pi = 3.141;

MPU6050 mpu(Wire);

float pos, po, thet, thet_dot, alp, alp_dot, prev_alp=0, prev_pos, currentTM, prevTM=0, dtt=0, rp, prev_rp=0;
float y[4]={}, y_setpoint[4]={} , cntr=1, vll, trq, sz=4, M, del_pos, angAc, prev_vel=0, vel_now, vel_give, target_pos, p_pos;
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
//  K[0] = -49.71888;K[1]=-7.37913;K[2]=-0.70711;K[3]=-0.91972;
  K[0] = -22.78308;K[1]=-3.37165;K[2]=-0.31623;K[3]=-0.41236;
  for (int i=0; i<sz; i++){
    mul=mul-(K[i]*(yy[i]-yy_setpoint[i]));
  }
  return mul;

}


int st_trq(int tr){
  // Take the trq value convert it to ang acc.
  // T = I(Moment of Inertial) * alpha(Angular Acceleration).
  angAc=tr/M;
//  Serial.print("Ang acc: ");
//  Serial.print(angAc);
  // angular acc is delv/delt.
  // for converting rpms to angular velocity mult it by 0.10471975512.
  vel_give = ((tr*dtt)/M)+(prev_vel);
//  Serial.print("\tVEL_give: ");
//  Serial.println(vel_give);
  // Now we will change velocity_give to position of the motor.
  target_pos = (vel_give*dtt)+p_pos;

  // Using PID controller to set the pwm value for the motor to move the RW wheel to the required position.
  set_pos();
  Serial.print("TargetPos: ");
  Serial.println(target_pos);
}

int set_pos(){
  float kp=1;
  float kd=0;
  float ki = 0;

  delaT = float(dtt)/1.0e6;

  //error
  int e = target_pos-pos;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // Integral
  eIntegral = eIntegral + e*deltaT;


  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // Powering the motor.
  float pwr = fabs(u);
  if (pwr>255){
    pwr = 255;
  }
  // Direction function.
  
  // Signal motor function
  
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
  M = 0.134*0.097*0.097/2; // Moment of Inertia.
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {

  
  // Theta.
  mpu.update();
  thet = mpu.getAngleY()*0.0174533; // radians
  // Theta_dot.
  thet_dot = mpu.getGyroY()*0.0174533; // radians

    // This is alpha
  // 0.0174533
  alp = (pos*360*0.10471975512)/100; // radians
//  Serial.print(alp);
  // alpha_dot, change in theta of the reaction wheel.
  currentTM = millis();
  dtt = currentTM-prevTM;

  if (dtt>10){
    alp_dot = (alp-prev_alp)/dtt; // radians
    // Finnding rpm of the motor.
    del_pos = pos-prev_pos; // In ppr
    rp = (del_pos*60*1000*0.10471975512)/(dtt*100); // This will probably give me rpms in degrees for coverting it to radians(angular velocity) mult it with 0.10471975512
    // And to degrees mult. with 57.29578
    vel_now = rp;
    prev_alp=alp;
    p_pos = prev_pos; // to be used later.
    prev_pos=pos;
    prevTM=currentTM;
  }

  
  // Now we will get the value of torque required from the lqr controller.
  y[0]=thet; y[1]=thet_dot; y[2]=alp;  y[3]=alp_dot;
  y_setpoint[0]=0;y_setpoint[1]=0;y_setpoint[2]=0;y_setpoint[3]=0;
  
  trq = lqr_controller(y, y_setpoint);
//  Serial.print("TRQ: ");
//  Serial.println(trq);


  // Now use this trq val to provide torque to the motor.
  st_trq(trq);
  prev_vel=vel_now;
  p_pos = prev_pos;
}
