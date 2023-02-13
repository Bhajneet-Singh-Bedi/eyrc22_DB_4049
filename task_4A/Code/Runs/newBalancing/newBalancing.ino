#include <MPU6050_light.h>


MPU6050 mpu(Wire);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets(true, true);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
