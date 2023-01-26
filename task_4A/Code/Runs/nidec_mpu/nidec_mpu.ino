// These two has to be interrupt pins.
//#include<MPU6050_light.h>

#define ENCA 19 //21-PD0 //19-RX1
#define ENCB 18 //20-PD1 //18-TX1
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B
#define pi = 3.141;

//MPU6050 mpu(Wire);
float alpha=0.0, theta=0.0;
volatile long pos=0.0;
double prevMillis=0, currentMillis=0, mil_now=0, mil_then=0;
double M=0.0, angAcc=0.0, trq=0.0, y[4], y_setpoint[4], K[4], prev_trq=0, rpms_then=0;
double dt=0, tor=0,dtt=0,theta_then, alpha_then, vel_Now, prev_rpm=0, max_rpm=0;
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
    Serial.print("Max RPM: ");
    Serial.print(max_rpm);
  }
  return rpm;
}

//int lqr_controller( int y,  int y_setpoint){
//  K = [-311.4268,-85.9813,-1.0000,-1.7637];
//  int mul=0;
//  for (int i=0; i<length(K); i++){
//    mul=mul-(K[i]*(y[i]-y_setpoint[i]))
//  }
//  return mul
//}



//int set_torque(int trq){
//  vel_Now=(angAcc*dtt)+vel_Then;
//  rpms=(vel_Now*60)/(2*pi);
//}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//  Wire.begin();
//  mpu.begin();
//  timer1_init();
//  TCCR2B = TCCR2B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(pwm, OUTPUT);
  digitalWrite(brake, LOW); // Low means braking.
  digitalWrite(cw, HIGH);
  analogWrite(pwm, 255);
  M = 0.036*0.097*0.097/2;
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
//  lqr_controller();
}

void loop() {
  // put your main code here, to run repeatedly
  //M=(100*0.0975*0.0975)/2;
  // Alpha angle
  // 0.0174533
  alpha=(pos*360*0.0174533)/100;

  // Alpha_dot
  mil_now=millis();
  dtt=mil_now-mil_then;
  if (dtt>0){
    alpha_dot=(alpha-alpha_then)/dtt;
  }

  // Theta
//  mpu.update();
//  theta=(mpu.getAngleX())*0.0174533;

  // Theta_Dot
//  if (dtt>0){
//    theta_dot=(theta-theta_then)/dtt;
//  }

  rpms=rounds();
  Serial.print("\tRPMS: ");
  Serial.print(rpms);

  angAcc=rpms-rpms_then/dtt;

  tor=M*angAcc;
  Serial.print("\tTorque: ");
  Serial.print(tor);
  
  mil_now_5=millis();
  if (mil_now_5-mil_then_5>=5000){
    analogWrite(pwm, 255-temp);
    temp+=5;
    mil_then_5=mil_now_5;
  }
  Serial.print("\tTemp: ");
  Serial.println(temp);
  // Angular Acceleration finding: 
  // acc is del v .
  // convert rpm to velocity.
//  vel_Now=rpms*2
//  angAcc=



  
//  y=[theta,theta_dot,alpha,alpha_dot];
//  y_setpoint=[0,0,0,0];
//  tor = lqr_controller(y, y_setpoint);

//  Serial.print("Theta: ");
//  Serial.print(theta);
//  Serial.print("\tTheta_then: ");
//  Serial.println(theta_dot);
//  Serial.print("\tAlpha: ");
//  Serial.print(alpha);
//  Serial.print("\tAlpha_dot: ");
//  Serial.println(alpha_dot);
  alpha_then=alpha; 
  theta_then=theta;
  mil_then=mil_now;
//  mil_then_5=mil_now_5;
  rpms_then=rpms;
//  alpha_then=alpha;
}
