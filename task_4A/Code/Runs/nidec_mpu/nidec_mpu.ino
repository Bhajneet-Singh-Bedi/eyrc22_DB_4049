// These two has to be interrupt pins.
//#include<MPU6050_light.h>

#define ENCA 19 //21-PD0 //19-RX1
#define ENCB 18 //20-PD1 //18-TX1
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B

//MPU6050 mpu(Wire);
float alpha=0.0, theta=0.0, rpm=0.0;
volatile long pos=0.0;
double prevMillis=0, currentMillis=0, oldPos=0, newPos=0,mil_now=0, mil_then=0;
double M=0.0,rads=0,prevRads=0.0, angAcc=0.0, trq=0.0;
double dt=0, ticks_1=0, ticks_60=0, del_pos=0, dt1=0, dt60s=0, tor=0,dtt=0,dta=0,theta_then, alpha_then;
double theta_dot, alpha_dot, mil_now_5, mil_then_5;
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

//void lqr_controller( y, int y_setpoint){
//  K = [-311.4268,-85.9813,-1.0000,-1.7637];
//  int mul=0
//  for (int i=0; i<length(K); i++){
//    mul=mul-(K[i]*(y[i]-y_setpoint[i]))
//  }
//  return mul
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
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
//  lqr_controller();
}

void loop() {
  // put your main code here, to run repeatedly
  //Serial.println(pos);
  //M=(100*0.0975*0.0975)/2;
  // Alpha angle
  alpha=(pos*360)/100;
  
  // Alpha_dot
  mil_now=millis();
  dtt=mil_now-mil_then;
  if (dtt>0){
    alpha_dot=(alpha-alpha_then)/dtt;
  }

  // Theta
  mpu.update();
  theta=mpu.getAngleX();

  // Theta_Dot
  if (dtt>0){
    theta_dot=(theta-theta_then)/dtt;
  }
//  Serial.print("Theta: ");
//  Serial.print(theta);
//  mpu.update();
//  alpha=mpu.getAngleX();
//  Serial.print("\tAlpha: ");
//  Serial.print(alpha);


  // New approach.
  // Wait for ten ticks, then find dt, 
  // That dt is for ten ticks, find out ticks for 1sec.
  // The mult it with 60 to get for a minute.


//  0.5dt=10 ticks;
//  1dt=10/0.5 ticks;
//  60dt=10*60/0.5;
//  1 deg/sec=0.017453 rad/sec.
  currentMillis=millis();
  new_alpha=(pos*360*0.017453)/100;
  if (currentMillis-prevMillis>=10){
    dt=currentMillis-prevMillis;
    del_alpha=new_alpha-old_alpha;
    dt1=del_pos/dt;
    dt60s=(dt1*1000*60)/dt;
    oldPos=newPos;
    prevMillis=currentMillis;
  }
  
  // Angular Acceleration finding: 
  // acc is del v .
  // convert rpm to velocity.
//  rads=0.10471975512*dt60s;





  if (mil_now_5-mil_then_5>=5000){
    analogWrite(pwm, 255-temp);
    temp+=10;
    mil_then_5=mil_now_5;
  }
//  Serial.print("TEMP: ");
//  Serial.print(temp);
  
//  newPos=pos;
//  currentMillis=millis();
//  if (newPos-oldPos >= 20){
//    dt=currentMillis-prevMillis;
//    ticks_1=20/dt;
//    ticks_60=(20*60)/dt;
//
//    rpm=ticks_60/100;
//    oldPos=newPos;
//    prevMillis=currentMillis;
//  }
//  Serial.print("New Pos: ");
//  Serial.print(newPos);
//  Serial.print("Old Pos: ");
//  Serial.print(oldPos);
//  Serial.print("PrevM: ");
//  Serial.print(prevMillis);
//  Serial.print("Curr: ");
//  Serial.print(currentMillis);
//  Serial.print("dt: ");
//  Serial.print(dt);
//  Serial.print("del_pos: ");
//  Serial.print(del_pos);
//  Serial.print("dt1: ");
//  Serial.print(dt1);
//  Serial.print("\tRPM: ");
//  Serial.println(dt60s);
  
/*
  currentMillis=millis();
  newPos=pos;
  if (currentMillis-prevMillis >= 50){
    rpm=float((newPos-oldPos)*20*60/(currentMillis-prevMillis));
  }
  rads=9.549296585513721*rpm;
  // torque code.
  // Formula: T=MOI*angular acc.
  angAcc=(rads-prevRads)/(currentMillis-prevMillis);
  //rpm=float((newPos-oldPos)*1000/(currentMillis-prevMillis));
  oldPos=newPos;
  prevMillis=currentMillis; 
  prevRads=rads;
  delay(50);
  Serial.print("\tRPM: ");
  Serial.print(rpm);
  Serial.print("Ang ACC: ");
  Serial.print(angAcc);
  trq=M*angAcc;
  Serial.print("\tTorque: ");
  Serial.println(trq);*/
//  y=[theta,theta_dot,alpha,alpha_dot];
//  y_setpoint=[0,0,0,0];
//  tor = lqr_controller(y, y_setpoint);

  Serial.print("Theta: ");
  Serial.print(theta);
  Serial.print("\tTheta_then: ");
  Serial.println(theta_dot);
//  Serial.print("\tAlpha: ");
//  Serial.print(alpha);
//  Serial.print("\tAlpha_dot: ");
//  Serial.println(alpha_dot);
  alpha_then=alpha; 
  mil_then=mil_now;
//  alpha_then=alpha;
}
