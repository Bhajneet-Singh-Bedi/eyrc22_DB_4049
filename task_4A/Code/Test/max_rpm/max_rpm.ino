#define ENCA 2 //21-PD0 //19-RX1
#define ENCB 3 //20-PD1 //18-TX1
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B
#define pi = 3.141;


float alpha=0.0, theta=0.0;
volatile long pos=0.0;
const int siz=4;
int y[siz], y_setpoint[siz];
double prevMillis=0, currentMillis=0, mil_now=0, mil_then=0, prev_rpms=0, velc;
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
    prev_rpm=rpm;
  }
  return rpm;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//  Serial.print("HII");
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(pwm, OUTPUT);
  digitalWrite(brake, HIGH); // Low means braking.
  digitalWrite(cw, HIGH); // gives positive value // Low will give negative
  analogWrite(pwm, 255);
  M = 0.134*0.097*0.097/2;
//  Serial.print("hi");
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:

//  Serial.print("Hello World");
//  Serial.print("Hello");
  Serial.println(pos);
  alpha=(pos*360*0.0174533)/100;

//  mil_now=millis();
//  dtt=mil_now-mil_then;
//  if (dtt>0){
//    alpha_dot=(alpha-alpha_then)/dtt;
//  }


  // 2727.28 -> with all screws.
  // 2828 -> without screws.
  mil_now_5=millis();
  if (mil_now_5-mil_then_5>=2000){
    analogWrite(pwm, 255-temp);
    temp+=5;
    mil_then_5=mil_now_5;
  }
//  Serial.print("Temp: ");
//  Serial.print(temp);

  rpms=rounds();
//  Serial.print("\tRounds: ");
//  Serial.println(rpms);


  alpha_then=alpha; 
  theta_then=theta;
  mil_then=mil_now;
//  mil_then_5=mil_now_5;
  rpms_then=rpms;
//  alpha_then=alpha;
}
