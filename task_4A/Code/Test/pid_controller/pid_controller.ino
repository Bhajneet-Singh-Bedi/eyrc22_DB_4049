// PID  controller code.
#define ENCA 2 //21-PD0 //19-RX1  //17-RX2
#define ENCB 3 //20-PD1 //18-TX1  //16-TX2
#define brake 41 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B
#define pi = 3.141;


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
int prev_dir = 1;

int setMotor(int dir, float pwr){
  analogWrite(pwm, pwr);

  if (dir == 1){
    if (prev_dir == -1){
      digitalWrite(brake, LOW);
      delay(5);
    }
    else{
      digitalWrite(brake, HIGH);
    }
    digitalWrite(cw, HIGH);
  }
  else if (dir == -1){
    if (prev_dir == 1){
      digitalWrite(brake, LOW);
      delay(5);
    }
    else{
      digitalWrite(brake, HIGH);
    }
    digitalWrite(cw, LOW);
  }  
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
//  digitalWrite(ENCA, HIGH);
//  digitalWrite(brake, LOW); // Low means braking.
//  digitalWrite(cw, HIGH); // gives positive value // Low will give negative
  analogWrite(pwm, 255); // 255 means stop // 0 means go.
//  mpu.calcOffsets(true, true); 
  M = 0.084*0.097*0.097/2; // Moment of Inertia.
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:

  int target_pos = 1200;

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos0 = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos0 = pos;
  interrupts(); // turn interrupts back on
  
  // error
  int e = target_pos-pos0;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eIntegral = eIntegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eIntegral;
  Serial.println(u);
  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr);


  // store previous error
  eprev = e;

//  Serial.print(target_pos);
//  Serial.print(" ");
//  Serial.print(pos0);
//  Serial.println();
}
