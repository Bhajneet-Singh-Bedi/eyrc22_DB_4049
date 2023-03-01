
#define ENCA 2 //21-PD0 //19-RX1
#define ENCB 3 //20-PD1 //18-TX1
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 2B
//#define pi = 3.141;

double mill=0, mill_then=0;
volatile long pos=0.0;

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
  digitalWrite(brake, HIGH); // Low means braking.
  digitalWrite(cw, LOW); // HIGH -> gives positive value(Clock-Wise) // Low will give negative(Anti-Clock-Wise)
  analogWrite(pwm, 245);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Pos: ");
  Serial.println(pos);
}
