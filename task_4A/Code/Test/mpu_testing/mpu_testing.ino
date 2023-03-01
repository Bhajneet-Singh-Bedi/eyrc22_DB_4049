#include <Wire.h>

const int MPU_addr = 0x68; // I2C address of MPU6050
const int sampleRateDiv = 99; // Set sample rate divider to 99 for 60 Hz sampling rate

void setup() {
  Wire.begin(); // Initialize I2C communication
  Serial.begin(9600); // Initialize serial communication
  delay(1000); // Wait for MPU6050 to stabilize
  setSampleRate(sampleRateDiv); // Set sampling rate
}

void loop() {
  // Your code here
}

void setSampleRate(int div) {
  Wire.beginTransmission(MPU_addr); // Start communication with MPU6050
  Wire.write(0x19); // Set sample rate divider register
  Wire.write(div); // Set sample rate to desired value
  Wire.endTransmission(); // End transmission
  delay(10); // Wait for settings to take effect
  
  Wire.beginTransmission(MPU_addr); // Start communication with MPU6050
  Wire.write(0x19); // Set pointer to sample rate divider register
  Wire.endTransmission(); // End transmission
  Wire.requestFrom(MPU_addr, 1); // Request 1 byte of data from MPU6050
  byte regVal = Wire.read(); // Read the data from the register
  Serial.print("Sample rate set to: ");
  Serial.print(1000 / (1 + regVal));
  Serial.println(" Hz"); // Print actual sampling rate
}
