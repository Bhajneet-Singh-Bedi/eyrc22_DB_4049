//#include <MPU6050_light.h>

#define ENCA 19 //21-PD0 //19-RX1
#define ENCB 18 //20-PD1 //18-TX1
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B
#define pi = 3.141;

//MPU6050 mpu(Wire);

float pos, alp, alp_dot, prev_alp=0, currentTM, prevTM=0, dtt=0;
float y[4]={}, y_setpoint[4]={}, K[4]={}, cntr=1, vll, trq, sz=4;
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

int lqr_controller(int yy, int yy_setpoint){
  int mul=0;
  K[0] = -49.71888;K[1]=-7.37913;K[2]=-0.70711;K[3]=-0.91972;
  for (int i=0; i<sz; i++){
    mul = mul - (K[i]*(yy[i]-yy_setpoint[i]));
  }
  return mul;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//  Wire.begin();
//  mpu.begin();
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(pwm, OUTPUT);
  digitalWrite(brake, LOW); // Low means braking.
//  digitalWrite(cw, HIGH); // gives positive value // Low will give negative
  analogWrite(pwm, 255); // 255 means stop // 0 means go.
//  mpu.calcOffsets(true, true); 
//  M = (0.134*0.097*0.097)/2;
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  // First we need to get the torque required from lqr.
//  mpu.update();
//  Serial.print("Y: ");
  // Gives me theta
//  Serial.print(mpu.getAngleY());
//  Serial.print("\tACC: ");
  // Gives me theta_dot
//  Serial.println(mpu.getGyroY());

  // Now we need to get angular position and velocity.
  // 0.0174533
  alp = (pos*360)/100;
//  Serial.println(alp);

  // alpha_dot, angular velocity.
  currentTM = micros(); // 1 milli = 1000 micro.
  dtt = currentTM-prevTM;
  if (dtt>50000){ // 50 milli seconds.
    alp_dot = (alp-prev_alp)/dtt;
  }

  // Now we will get the value of torque required from the lqr controller.
  if (cntr==1){
    vll=alp;
  }
  y[0]=0; y[1]=0; y[2]=alp;  y[3]=alp_dot;
  y_setpoint[0]=vll;y_setpoint[1]=0;y_setpoint[2]=0;y_setpoint[3]=0;
  trq = lqr_controller(y, y_setpoint);


  // Now use this trq val
  
  prev_alp=alp;
  prevTM=micros();
}
