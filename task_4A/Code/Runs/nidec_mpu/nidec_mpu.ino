#include <MPU6050_light.h>
#include<Wire.h>

MPU6050 mpu(Wire);

int x,y,z;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu.update();
  Serial.print(mpu.getAngleX());
  Serial.print('\t');
  Serial.print(mpu.getAngleY());
  Serial.print('\t');
  Serial.print(mpu.getAngleZ());
  Serial.println();
//  Serial.println('h');
}
