float leftSpeed = 0.0;
float rightSpeed = 0.0;

void setup() {
  Serial.begin(115200);  
}

void loop() {
  if (Serial.available()) {
    
    String data = Serial.readStringUntil('\n');

    
    int commaIndex = data.indexOf(',');

    
    if (commaIndex > 0) {
      String leftSpeedStr = data.substring(0, commaIndex);
      String rightSpeedStr = data.substring(commaIndex + 1);

     
      leftSpeed = leftSpeedStr.toFloat();
      rightSpeed = rightSpeedStr.toFloat();

      
      Serial.print("Left Wheel Speed: ");
      Serial.print(leftSpeed);
      Serial.print(", Right Wheel Speed: ");
      Serial.println(rightSpeed);
    }
  }
}
