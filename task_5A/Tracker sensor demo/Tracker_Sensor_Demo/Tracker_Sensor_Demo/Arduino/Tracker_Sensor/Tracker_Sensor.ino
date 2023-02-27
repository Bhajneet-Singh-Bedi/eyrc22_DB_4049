/* WaveShare ARPICAR Run Forward/Backward/Left/Right Test
   
   ARPICAR run forward,backward,left right and so on..
   
   Created 25th June 2016
           by Xinwu Lin
           
   CN: http://www.waveshare.net/
   EN: http://www.waveshare.com/
*/

void setup()
{
  Serial.begin(115200);
  Serial.println("TRSensor example");
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A11, INPUT);

}


void loop()
{

  Serial.print(analogRead(A8));
  Serial.print("  ");
  Serial.print(analogRead(A9));
  Serial.print("  ");
  Serial.print(analogRead(A10));
  Serial.print("  ");
  Serial.print(analogRead(A11));
  Serial.print("  ");
  Serial.print(analogRead(A12));
  Serial.println();
  delay(200);

}
