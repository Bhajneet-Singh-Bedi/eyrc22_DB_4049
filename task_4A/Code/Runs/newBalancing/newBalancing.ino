#include <util/atomic.h>
//#include <MPU6050_light.h>
//
//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

#define ENCA 2 //21-PD0 //19-RX1  //17-RX2
#define ENCB 3 //20-PD1 //18-TX1  //16-TX2
#define brake 48 //PL1
#define cw 11 // 1A
#define pwm 9 // 

// globals
float prev = 0, K[4]={}, y_setpoint[4]={}, M, y[4]={};
float thet, thet_dot, prevAlp=0, vGive, prev_u=0;
int alp, alp_dot;
long prevT=0;
int posPrev=0,  sz=4, trq;
volatile int pos_i=0; 
volatile float velocity_i=0;
volatile long prevT_i=0;


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




int lqrController(float yy[], float yy_setpoint[]){
  float mul=0;
  //  K = [-311.4268,-85.9813,-1.0000,-1.7637];
//  K[0]=-41.4663;K[1]=-6.4185;K[2]=-1.0000;K[3]=-1.3120;
//  K[0]=-49.32124;K[1]=-7.25900;K[2]=-1.00000;K[3]=-1.29629;
//  K[0]=-49.91982;K[1]=-7.31929;K[2]=-1.00000;K[3]=-1.29515;
//  K[0]=-69.90391;K[1]=-10.38223;K[2]=-1.00000;K[3]=-1.29991;
//  K[0]=-69.97770;K[1]=-10.38361;K[2]=-1.00000;K[3]=-1.30019;
//  K[0]=-69.89653;K[1]=-10.38210;K[2]=-1.00000;K[3]=-1.2998;4
//  K[0]=-77.60897;K[1]=-10.52474;K[2]=-1.00000;K[3]=-1.32904;
//  K[0] = -22.78308;K[1]=-3.37165;K[2]=-0.31623;K[3]=-0.41236;
//  K[0] = -49.71888;K[1]=-7.37913;K[2]=-0.70711;K[3]=-0.91972;
//    K[0]=-69.90391;  K[1]=-10.38223;   K[2]=-1.00000;   K[3]=-1.29991;
//    K[0]=-40.77681;   K[1]=-6.04874;   K[2]=-0.57735;   K[3]=-0.75130;
//    K[0]=-193.01827;   K[1]=-25.14391;    K[2]=-1.00000;    K[3]=-1.26103;
//    K[0]=-136.66472;   K[1]=-17.80188;    K[2]=-0.70711;    K[3]=-0.89180;
//    K[0]=-281.02199;   K[1]=-35.01541;    K[2]=-0.70711;    K[3]=-0.88350;
//    K[0]=-217.99154;   K[1]=-27.49369;    K[2]=-0.70711;    K[3]=-0.88573;
//    K[0]=-147.40983;   K[1]=-18.80253;    K[2]=-0.57735;    K[3]=-0.72496;
//    K[0]=-147.40983;   K[1]=-18.80253;    K[2]=-0.70711;    K[3]=-0.72496;
//    K[0]=-26.01250;   K[1]=-3.31595;   K[2]=-0.10000;   K[3]=-0.12580;
//    K[0]=-28.07205;   K[1]=-3.65106;   K[2]=-0.14142;   K[3]=-0.17865;
//    K[0]=-28.47513;   K[1]=-3.65753;   K[2]=-0.14142;   K[3]=-0.17913;
  K[0]=-272.09980;   K[1]=-36.50311;    K[2]=-1.00000;    K[3]=-1.26900;
  for (int i=0; i<sz; i++){
    mul=mul-(K[i]*(yy[i]-yy_setpoint[i]));
  }
  return mul;
}

void setMotor(int dir, int pwr, float prev_u){
  
  if (dir == 1){
    if (prev_u == -1){
      digitalWrite(brake, LOW);
    }
    digitalWrite(cw, LOW);
  }
  else{
    if (prev_u == 1){
      digitalWrite(brake, LOW);
    }
    digitalWrite(cw, HIGH);
  }
//  Serial.println(255-pwr);
  digitalWrite(brake, HIGH);
  analogWrite(pwm, 255-pwr);
}

void setTorque(float dtt, int trq, float vNow){
  // T = M*angular acc.
//  Serial.print(trq);
//  Serial.print("\t");
  vGive = ((trq*dtt)/M) + vNow;
//  Serial.println(vGive);

//  Serial.println(vGive);
  float kp=3;
  float e=vGive-vNow;
  float u=kp*e;

//  float u = map(fabs(vNow), 0, 3200, 0, 255);
//  Serial.println(u);

  // Direction.
  int dir=1;
  if (u<0){
    dir=-1;
  }

  int pwr = (int) fabs(u);

  if (pwr>255){
    pwr = 255;
  }
  setMotor(dir, pwr, prev_u);
  prev_u=u;
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//  Wire.begin();
//  mpu.begin();
//  pinMode(ENCA, INPUT);
//  pinMode(ENCB, INPUT);
//  pinMode(brake, OUTPUT);
//  pinMode(cw, OUTPUT);
//  pinMode(pwm, OUTPUT);
//  digitalWrite(ENCA, HIGH);
//  digitalWrite(brake, LOW); // Low means braking.
//  digitalWrite(cw, HIGH); // gives positive value // Low will give negative
//  analogWrite(pwm, 255); // 255 means stop // 0 means go.

  Wire.begin();
  mpu.initialize();
//  mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  mpu.setDMPEnabled(true);
  Serial.println(F("MPU6050 initialized successfully!"));

//  mpu.calcOffsets(true, true); 
//  M = 0.134*0.097*0.097/2; // Moment of Inertia.
//  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {

  // read raw gyro data from MPU6050
  int16_t gyroX = mpu.getRotationX();
  int16_t gyroY = mpu.getRotationY();
  int16_t gyroZ = mpu.getRotationZ();

  // calculate the orientation data in degrees
  float roll = atan2(mpu.getAccelerationY(), mpu.getAccelerationZ()) * 180 / M_PI;
  float pitch = atan2(-mpu.getAccelerationX(), sqrt(mpu.getAccelerationY() * mpu.getAccelerationY() + mpu.getAccelerationZ() * mpu.getAccelerationZ())) * 180 / M_PI;
  float yaw = mpu.getRotationZ() / 131.0;

  // print the orientation data to the serial monitor
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" degrees, ");

//  Serial.print("Pitch: ");
//  Serial.print(pitch);
//  Serial.print(" degrees, ");
//
//  Serial.print("Yaw: ");
//  Serial.print(yaw);
//  Serial.println(" degrees");

  // calculate the change in orientation data in degrees per second
  float gyroXrate = gyroX / 131.0;
  float gyroYrate = gyroY / 131.0;
  float gyroZrate = gyroZ / 131.0;

  // print the change in orientation data to the serial monitor
  Serial.print("\tGyro X: ");
  Serial.print(gyroXrate);
  Serial.println(" degrees per second, ");

//  Serial.print("Gyro Y: ");
//  Serial.print(gyroYrate);
//  Serial.print(" degrees per second, ");
//
//  Serial.print("Gyro Z: ");
//  Serial.print(gyroZrate);
//  Serial.println(" degrees per second");

  delay(10);


//  mpu.update();
////  Serial.println(mpu.getAngleY());
////  *0.0174533
//  thet = mpu.getAngleX(); // radians
//  // Theta_dot.
//  // radians
//  thet_dot = mpu.getGyroX(); // radians
//  Serial.print(thet);
//  Serial.print("\t");
//  Serial.println(thet_dot);


  int pos=0;
  float velocity2=0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2=velocity_i;
  }


  
  float curr = micros();
  // radians - 0.10471975512
  alp = (pos_i*360)/100; // radians
  float dtt = curr - prev;
  prev = curr;

  // We have to calculate for 
  alp_dot = (alp-prevAlp)/dtt;
  prevAlp=alp;
  
  
//  int pwr = 100/3.0*micros()/1.0e6;
//  int dir = 1;
//  setMotor(dir, pwr);
//  Serial.print(thet);
//  Serial.print("\t");
//  Serial.print(thet_dot);
//  Serial.println("\t");
  
  y[0]=thet; y[1]=thet_dot; y[2]=alp;  y[3]=alp_dot;
  y_setpoint[0]=0;y_setpoint[1]=0;y_setpoint[2]=0;y_setpoint[3]=0;
  trq = lqrController(y, y_setpoint);
//  Serial.println(trq);
  

//   Computing velocity with method 1
//  long currT = micros();
//  float deltaT = ((float) (currT-prevT))/1.0e6;
//  float velocity1 = (pos-posPrev)/deltaT;
//  posPrev=pos;
//  prevT = currT;  


  float v2 = velocity2/100.0*60.0;
//  Serial.print(velocity1);
//  Serial.print(" ");
//  Serial.print(v2);
//  Serial.println();
  setTorque(dtt, trq, v2);


}
