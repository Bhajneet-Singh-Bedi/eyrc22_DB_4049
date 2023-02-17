#define ENCA 2 //21-PD0 //19-RX1  //17-RX2
#define ENCB 3 //20-PD1 //18-TX1  //16-TX2
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 


int thet, thet_dot, pos, alp, alp_dot, tm=1, curr=0, prev=0, prev_alp, sz=4;
float K[4]={}, y[4]={}, y_setpoint[4]={}, trq;
float rp, nw, prev_nw, dtt;
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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
//  M = 0.084*0.097*0.097/2; // Moment of Inertia.
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  curr = millis();
  // put your main code here, to run repeatedly:
  // I have the ticks.
  nw = pos/100;
  dtt = curr-prev;
  prev = curr;
//  Serial.println(dtt);

  // Finding out rpms

  rp = (nw-prev_nw)*1000/dtt; // This will probably give me rpm
  Serial.println(rp);
  prev_nw = nw;
  
}
