// These two has to be interrupt pins.
#include<MPU6050_light.h>

MPU6050 mpu(Wire);

#define ENCA 19 //21-PD0 //19-RX1
#define ENCB 18 //20-PD1 //18-TX1

#define MPU_INT 2
float pos=0.0,alpha=0.0, theta=0.0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(MPU_INT, INPUT);
  //pinMode(brake, OUTPUT);
  //digitalWrite(brake, LOW);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
//  attachInterrupt(digitalPinToInterrupt(MPU_INT), readMPU, RISING);
//  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
//  timer1_init();
  
}

void loop() {
  // put your main code here, to run repeatedly
  //Serial.println(pos);
  theta=(pos*360)/100;
  Serial.print("Theta: ");
  Serial.print(theta);
  mpu.update();
  alpha=mpu.getAngleX();
  Serial.print("\tAlpha: ");
  Serial.println(alpha);
}

//void timer1_init()
//{
//    cli(); //Clears the global interrupts
//    TIMSK1 = 0x01; //timer5 overflow interrupt enable
//    TCCR1B = 0x00; //stop
//    TCNT1H = 0xA2; //Counter higher 8 bit value
//    TCNT1L = 0x3F; //Counter lower 8 bit value
//    TCCR1A = 0x00;
//    TCCR1C = 0x00;
//    TCCR1B = 0x02; //start Timer, prescaler 8
//    sei();   //Enables the global interrupts
//}
//
//ISR (TIMER1_OVF_vect)
//{
//    sei();  
//    TCNT1H = 0xA2; //Counter higher 8 bit value
//    TCNT1L = 0x3F; //Counter lower 8 bit value
//    mpu.update();
//    cli();
//    alpha=mpu.getAngleX();
//    Serial.println(alpha);
//}

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

//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//void readMPU() {
//   mpuInterrupt = true;
//  mpu.update();
//  alpha=mpu.getAngleX();
//  Serial.println(alpha);
//}
