#include <util/atomic.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PIDController.h>

PIDController pid;
MPU6050 mpu;

#define ENCA 2 //21-PD0 //19-RX1  //17-RX2
#define ENCB 3 //20-PD1 //18-TX1  //16-TX2
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 

//int sampleRateDiv=0;
//// globals
//float prev = 0, K[4]={}, y_setpoint[4]={}, M, y[4]={};
//float thet, thet_dot, prevAlp=0, vGive, prev_u=0;
//int alp, alp_dot;
//long prevT=0;
//int posPrev=0,  sz=4, trq;
//volatile int pos_i=0; 
//volatile float velocity_i=0;
//volatile long prevT_i=0;
int prev_pwr=0;
int p, i, d;
String input;

/*
void readEncoder(){
  int b=digitalRead(ENCB);
  int increment=0;
  if (b>0){
    increment=1;
//    pos_i++;
  }
  else{
    increment=-1;
//    pos_i--;
  }
  pos_i=pos_i+increment;
//  Serial.println(pos_i);

  long currT=micros();
  float deltaT=((float) (currT-prevT_i))/1.0e6;
  velocity_i=increment/deltaT;
  prevT_i=currT;
}
*/

void setMotor(int pwr){
  if (pwr>0){
    // This means that motor should rotate clockwise. signal should be LOW.
    if (prev_pwr<=0){
      // first apply brake then reverse cw and ccw.
      digitalWrite(brake, LOW);
//      Serial.print("\tBrake");
    }
    digitalWrite(cw, LOW); // This means clockwise.
//    Serial.println("Clock");
    digitalWrite(brake, HIGH);
  }
  else if (pwr <= 0){
    if (prev_pwr>=0){
      // first apply brake then reverse cw and ccw.
      digitalWrite(brake, LOW); // Anticlockwise
//      Serial.print("\tBrake");
    }
    digitalWrite(cw, HIGH);
//    Serial.println("Anti");
    digitalWrite(brake, HIGH);
  }
//  Serial.print("\t");
//  Serial.print(pwr);
  if(fabs(pwr)>=255){
  analogWrite(pwm, 0);
  }
  else{
    analogWrite(pwm, 255-fabs(pwr));
  }
  prev_pwr = pwr;
}

void printValues(){
  Serial.print("P: ");
  Serial.print(p);
  Serial.print("\tI: ");
  Serial.print(i);
  Serial.print("\tD: ");
  Serial.println(d);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
//  mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  mpu.setDMPEnabled(true);
//  Serial.println(F("MPU6050 initialized successfully!"));
  delay(1000); // Wait for MPU6050 to stabilize
//  setSampleRate(sampleRateDiv); // Set sampling rate
  pid.begin();          // initialize the PID instance
  
  pid.setpoint(0);    // The "goal" the PID controller tries to "reach"
  pid.tune(5000, 1, 0);    // Tune the PID, arguments: kP, kI, kD
//  pid.limit(0, 255);    // Limit the PID output between 0 and 255, /this is important to get rid of integral windup!

  
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(brake, OUTPUT);
  pinMode(cw, OUTPUT);
  pinMode(pwm, OUTPUT);
  digitalWrite(ENCA, HIGH);
  digitalWrite(brake, LOW); // Low means braking.
  digitalWrite(cw, HIGH); // gives positive value // Low will give negative
  analogWrite(pwm, 255); // 255 means stop // 0 means go.
}


//void setSampleRate(int div) {
//  Wire.beginTransmission(0x68); // Start communication with MPU6050
//  Wire.write(0x19); // Set sample rate divider register
//  Wire.write(div); // Set sample rate to desired value
//  Wire.endTransmission(); // End transmission
//  delay(10); // Wait for settings to take effect
//  
//  Wire.beginTransmission(0x68); // Start communication with MPU6050
//  Wire.write(0x19); // Set pointer to sample rate divider register
//  Wire.endTransmission(); // End transmission
//  Wire.requestFrom(0x68, 1); // Request 1 byte of data from MPU6050
//  byte regVal = Wire.read(); // Read the data from the register
//  Serial.print("Sample rate set to: ");
//  Serial.print(1000 / (1 + regVal));
//  Serial.println(" Hz"); // Print actual sampling rate
//}


void loop() {
  // put your main code here, to run repeatedly:
  // read raw gyro data from MPU6050
//  int16_t gyroX = mpu.getRotationX();
  float gyroX = mpu.getRotationX();
//  int16_t gyroY = mpu.getRotationY();
//  int16_t gyroZ = mpu.getRotationZ();
//  float roll = mpu.getAngleX();
  

  // calculate the orientation data in degrees
  float roll = atan2(mpu.getAccelerationY(), mpu.getAccelerationZ()) * 180 / M_PI;
//  float pitch = atan2(-mpu.getAccelerationX(), sqrt(mpu.getAccelerationY() * mpu.getAccelerationY() + mpu.getAccelerationZ() * mpu.getAccelerationZ())) * 180 / M_PI;
//  float yaw = mpu.getRotationZ() / 131.0;

  // print the orientation data to the serial monitor
//  Serial.print("Roll: ");
//  Serial.print(roll);
//  Serial.print(" degrees, ");

//  Serial.print("Pitch: ");
//  Serial.print(pitch);
//  Serial.print(" degrees, ");
//

//  Serial.print("Yaw: ");
//  Serial.print(yaw);
//  Serial.println(" degrees");

  // calculate the change in orientation data in degrees per second
//  float gyroXrate = gyroX / 131.0;
//  float gyroYrate = gyroY / 131.0;
//  float gyroZrate = gyroZ / 131.0;

  // print the change in orientation data to the serial monitor
//  Serial.print("\tGyro X: ");
//  Serial.print(gyroXrate);
//  Serial.println(" degrees per second, ");

//  Serial.print("Gyro Y: ");
//  Serial.print(gyroYrate);
//  Serial.print(" degrees per second, ");
//
//  Serial.print("Gyro Z: ");
//  Serial.print(gyroZrate);
//  Serial.println(" degrees per second");

//  delay(10);

//  int pos=0;
//  float velocity2=0;
//  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
//    pos = pos_i;
//    velocity2=velocity_i;
//  }
  //112,105,100
  
//  if (Serial.available() == 0) return 0;
//  delay(2);
//  char param = Serial.read();
//  if (Serial.available() == 0) return 0;
//  char cmd = Serial.read();
//  Serial.flush();
//  switch(param){
//    case 'p':
//      if (cmd == '+') p+=5;
//      if (cmd == '-') p-=1;
//      printValues();
//      break;
//     case 'i':
//      if (cmd == '+') i+=5;
//      if (cmd == '-') i-=1;
//      printValues();
//      break;
//     case 'd':
//      if (cmd == '+') d+=5;
//      if (cmd == '-') d-=1;
//      printValues();
//      break;
//  }
  
//  pid.tune(p, i, d);    // Tune the PID, arguments: kP, kI, kD
  Serial.print("\troll");
  Serial.print(roll);  
  int output = pid.compute(roll);
//  Serial.println(output);

  setMotor(output);
  

}
