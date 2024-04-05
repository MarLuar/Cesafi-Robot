// L298P Motor Driver Pins
const int motor1Pin1 = 11; // Connect to L298P IN1
const int motor1Pin2 = 13; // Connect to L298P IN2
const int motor2Pin1 = 3;   // Connect to L298P IN3
const int motor2Pin2 = 12;  // Connect to L298P IN4

// Infrared sensor pins
const int leftSensorPin = 2;    // Left sensor pin
const int middleSensorPin = 4;  // Middle sensor pin
const int rightSensorPin = 5;   // Right sensor pin

// Motor speeds
const int motorSpeed = 100;

const unsigned long pingInterval = 20; // Read sensors every 100 milliseconds

unsigned long lastPingTime = 0; // Variable to store the last time sensors were read

void setup() {
  // Set the motor pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Set sensor pins as inputs
  pinMode(leftSensorPin, INPUT);
  pinMode(middleSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  // Initialize Serial communication
  Serial.begin(9600);
}

// Function to move forward
void moveForward() {
  analogWrite(motor1Pin1, motorSpeed);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(motor2Pin1, motorSpeed);
  digitalWrite(motor2Pin2, LOW);
}

// Function to move backward
void moveBackward() {
  digitalWrite(motor1Pin1, LOW);
  analogWrite(motor1Pin2, motorSpeed);
  digitalWrite(motor2Pin1, LOW);
  analogWrite(motor2Pin2, motorSpeed);
}

// Function to turn left
void turnLeft() {
  analogWrite(motor1Pin1, 255);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  analogWrite(motor2Pin2, 255);
}

// Function to turn right
void turnRight() {
  digitalWrite(motor1Pin1, LOW);
  analogWrite(motor1Pin2, 255);
  analogWrite(motor2Pin1, 255);
  digitalWrite(motor2Pin2, LOW);
}

// Function to stop
void stopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

void loop() {
  // Read sensor values
   if (millis() - lastPingTime >= pingInterval) {

      lastPingTime = millis();
  int leftSensorValue = digitalRead(leftSensorPin);
  int middleSensorValue = digitalRead(middleSensorPin);
  int rightSensorValue = digitalRead(rightSensorPin);

  // Output sensor values to serial monitor
  Serial.print("Left Sensor: ");
  Serial.print(leftSensorValue);
  Serial.print(" - Middle Sensor: ");
  Serial.print(middleSensorValue);
  Serial.print(" - Right Sensor: ");
  Serial.println(rightSensorValue);

  // Line follower algorithm
  if (leftSensorValue == HIGH && middleSensorValue == HIGH && rightSensorValue == LOW) {
    // No line detected, stop
    moveForward();
  } else if (leftSensorValue == LOW && middleSensorValue == LOW && rightSensorValue == LOW) {
    // Right sensor detects line, turn right
    
    turnLeft();
    delay(100);
  }
  else if (leftSensorValue == LOW && middleSensorValue == HIGH && rightSensorValue == LOW) {
    // Right sensor detects line, turn right
    
    turnLeft();
    delay(100);
  } else if (leftSensorValue == LOW && middleSensorValue == LOW && rightSensorValue == HIGH) {
    // Middle sensor detects line, move forward
    moveForward();
  } else if (leftSensorValue == HIGH && middleSensorValue == LOW && rightSensorValue == HIGH) {
    // Left sensor detects line, turn left
    turnRight();
    delay(100);
  

  }
   else if (leftSensorValue == HIGH && middleSensorValue == HIGH && rightSensorValue == HIGH) {
    // Left sensor detects line, turn left
    turnRight();
    delay(1000);

  } else if (leftSensorValue == HIGH && middleSensorValue == HIGH && rightSensorValue == LOW) {
    // All sensors detect line, stop
    stopMotors();
  }
  }
}
