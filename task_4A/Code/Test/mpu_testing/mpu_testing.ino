#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
float x=0.0;

void timer1_init()
{
    cli(); //Clears the global interrupts
    TIMSK1 = 0x01; //timer5 overflow interrupt enable
    TCCR1B = 0x00; //stop
    TCNT1H = 0xA2; //Counter higher 8 bit value
    TCNT1L = 0x3F; //Counter lower 8 bit value
    TCCR1A = 0x00;
    TCCR1C = 0x00;
    TCCR1B = 0x02; //start Timer, prescaler 8
    sei();   //Enables the global interrupts
}

ISR (TIMER1_OVF_vect)
{
    sei();  
    TCNT1H = 0xA2; //Counter higher 8 bit value
    TCNT1L = 0x3F; //Counter lower 8 bit value
    mpu.update();
    cli();
    Serial.print("X: ");
    Serial.print(mpu.getAngleX());
//    Serial.print("Y: ");
//    Serial.print(mpu.getAngleY());
//    Serial.print("Z: ");
//    Serial.println(mpu.getAngleZ()),;
}

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  Wire.begin();
  byte status=mpu.begin();
  Serial.print(F("MPU6050 status: "));
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println("MPU begin done!\n");
  //Serial.println(mpu.getAngleX());
  timer1_init(); 
  Serial.println("Timer initialized\n");
}

void loop() {
  
  // put your main code here, to run repeatedly:
}
