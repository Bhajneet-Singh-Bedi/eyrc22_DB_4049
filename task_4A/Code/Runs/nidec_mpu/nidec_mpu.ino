// These two has to be interrupt pins.
#include<MPU6050_light.h>

#define ENCA 19 //21-PD0 //19-RX1
#define ENCB 18 //20-PD1 //18-TX1
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B
#define pi = 3.141;

MPU6050 mpu(Wire);
float alpha=0.0, theta=0.0;
volatile long pos=0.0;
const int siz=4;
int y[siz], y_setpoint[siz];
double prevMillis=0, currentMillis=0, mil_now=0, mil_then=0, prev_rpms=0, velc, cntr=1, vall;
double M=0.0, angAcc=0.0, trq=0.0, K[siz]={-41.4663,-6.4185,-1.0000,-1.3120}, prev_trq=0, rpms_then=0;
double dt=0, tor=0,dtt=0,theta_then, alpha_then, vel_Now, prev_rpm=0, max_rpm=0, pwmVal;
double theta_dot, alpha_dot, mil_now_5=0, mil_then_5=0, rpm=0, new_alpha=0,del_alpha=0, old_alpha=0, rpms=0;
int temp=0;
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

int rounds(){
  currentMillis=millis();
  new_alpha=(pos*360*0.0174533)/100;
  if (currentMillis-prevMillis>=10){
    dt=currentMillis-prevMillis;
    del_alpha=new_alpha-old_alpha;
    rpm=del_alpha*1000*60/(dt*6.28319);
    old_alpha=new_alpha;
    prevMillis=currentMillis;
    max_rpm=max(rpm, prev_rpm);
//    Serial.print("Max RPM: ");
//    Serial.print(max_rpm);
  }
  return rpm;
}

int lqr_controller( int y[],  int y_setpoint[]){
//  K = [-311.4268,-85.9813,-1.0000,-1.7637];
//  K[0]=-41.4663;K[1]=-6.4185;K[2]=-1.0000;K[3]=-1.3120;
//  K[0]=-49.32124;K[1]=-7.25900;K[2]=-1.00000;K[3]=-1.29629;
//  K[0]=-49.91982;K[1]=-7.31929;K[2]=-1.00000;K[3]=-1.29515;
//  K[0]=-69.90391;K[1]=-10.38223;K[2]=-1.00000;K[3]=-1.29991;
//  K[0]=-69.97770;K[1]=-10.38361;K[2]=-1.00000;K[3]=-1.30019;
//  K[0]=-69.89653;K[1]=-10.38210;K[2]=-1.00000;K[3]=-1.2998;4
//  K[0]=-77.60897;K[1]=-10.52474;K[2]=-1.00000;K[3]=-1.32904;
//  K[0] = -22.78308;K[1]=-3.37165;K[2]=-0.31623;K[3]=-0.41236;
  K[0] = -49.71888;K[1]=-7.37913;K[2]=-0.70711;K[3]=-0.91972;

//  K = {-41.4663,-6.4185,-1.0000,-1.3120};
  int mul;
  for (int i=0; i<siz; i++){
    mul = mul - (K[i]*(y[i]-y_setpoint[i]));
//    Serial.print("HI");
  }
  Serial.print("MUL: ");
  Serial.println(mul);
  return mul;
}



int set_torque(double tr, double rp, double rp_then, double dtt_1){
  // What do I have to do?
  // We need to set the motor to required torque.
  // We will do this by first converting the torque value to rpms.
  // Use T=MOI * angular acc.
  // Ang acc = dv/dt.
  // check that speed. If that speed value if greater than
  
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
  digitalWrite(cw, HIGH); // gives positive value // Low will give negative
  analogWrite(pwm, 255);
  mpu.calcGyroOffsets( ); 
  M = 0.134*0.097*0.097/2;
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
//  lqr_controller();
}

void loop() {
  // Alpha angle
  // 0.0174533
  alpha=(pos*360*0.01745)/100;

  // Alpha_dot
  mil_now=millis();
  dtt=mil_now-mil_then;
  if (dtt>0){
    alpha_dot=(alpha-alpha_then)/dtt;
  }

  // Theta
  mpu.update();
  theta=mpu.getAngleY();

  // Theta_Dot
  if (dtt>0){
    theta_dot=(theta-theta_then)/dtt;
  }
//  theta_dot=mpu.getGyroX();

  rpms=rounds();

  // Max RPMS, 2828.0
//  angAcc=rpms-rpms_then/dtt;
//
//  tor=M*angAcc;
//  Serial.print("\tTorque: ");
//  Serial.print(tor);
  
//  mil_now_5=millis();
//  if (mil_now_5-mil_then_5>=5000){
//    analogWrite(pwm, 255-temp);
//    temp+=5;
//    mil_then_5=mil_now_5;
//  }
//  Serial.print("\tTemp: ");
//  Serial.println(temp);
  // Angular Acceleration finding: 
  // acc is del v .
  // convert rpm to velocity.
//  vel_Now=rpms*2
//  angAcc=



  y[0]=int(theta);y[1]=int(theta_dot);y[2]=int(alpha);y[3]=int(alpha_dot);
//   y={theta,theta_dot,alpha,alpha_dot};
  if (cntr==1){
    vall=theta;
    cntr+=1;

  }
  y_setpoint[0]=vall;y_setpoint[1]=0;y_setpoint[2]=0;y_setpoint[3]=0;
//  y_setpoint=[0,0,0,0];
  trq = lqr_controller(y, y_setpoint);



  set_torque(trq, rpms, rpms_then, dtt);
//  Serial.print("Theta: ");
//  Serial.println(theta);
//  Serial.print("\tTheta_then: ");
//  Serial.println(theta_dot);
//  Serial.print("\tAlpha: ");
//  Serial.print(alpha);
//  Serial.print("\tAlpha_dot: ");
//  Serial.println(alpha_dot);
  alpha_then=alpha; 
  theta_then=theta;
  mil_then=mil_now;
  prev_rpm=rpm;
//  mil_then_5=mil_now_5;
  rpms_then=rpms;
//  alpha_then=alpha;
}
