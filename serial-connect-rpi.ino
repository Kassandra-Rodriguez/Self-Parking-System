#include <Wire.h>
#include <Servo.h>

// Servo pin
const int servoPin = 7;
Servo myServo;

void setup() {
  Wire.begin(8);  // I2C Slave Address
  Wire.onReceive(receiveEvent);
  myServo.attach(servoPin);
  myServo.write(90);  // Initialize servo to 90 degrees (center position)
  Serial.begin(9600);
  Serial.println("Arduino I2C Slave Ready");
}

void loop() {
  delay(100);
}

// Function to receive data from Raspberry Pi
void receiveEvent(int howMany) {
  if (Wire.available()) {
    int command = Wire.read();  // Read the command
    Serial.print("Received Command: ");
    Serial.println(command);

    if (command >= 0 && command <= 180) {
      myServo.write(command);  // Move the servo to the desired position
      Serial.print("Servo Moved to: ");
      Serial.println(command);
    }
  }
}
