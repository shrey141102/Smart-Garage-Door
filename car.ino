#include <Servo.h>

// Motor control pins
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

// Ultrasonic sensor pins
#define TRIG_PIN A5
#define ECHO_PIN A4

// Servo pin
#define SERVO_PIN 3

// Constants
const int SAFE_DISTANCE = 15;    // Safe distance in centimeters
const int MOTOR_SPEED = 200;     // Motor speed (0-255)
const int STOP_DISTANCE = 10;    // Distance to stop at (cm)

// Create servo object
Servo sensorServo;
int currentAngle = 90;  // Initial servo position (looking forward)

void setup() {
  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initialize servo
  sensorServo.attach(SERVO_PIN);
  sensorServo.write(currentAngle);  // Set initial position
  
  // Start serial communication for debugging
  Serial.begin(9600);
  
  // Initially stop the car
  stopCar();
}

void loop() {
  // Measure distance using ultrasonic sensor
  int distance = measureDistance();
  
  // Print distance for debugging
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Control car based on distance
  if (distance <= STOP_DISTANCE) {
    // Too close to obstacle/gate - stop
    stopCar();
    Serial.println("Stopping - Gate closed or obstacle detected");
    
    // Look left and right for possible clearance
    scanSurroundings();
  }
  else if (distance > SAFE_DISTANCE) {
    // Safe to move forward
    moveForward();
    Serial.println("Moving forward - Path clear");
    
    // Return servo to center position if it's not already there
    if (currentAngle != 90) {
      currentAngle = 90;
      sensorServo.write(currentAngle);
      delay(200);  // Allow servo to reach position
    }
  }
  
  // Small delay before next measurement
  delay(100);
}

// Function to scan surroundings when stopped
void scanSurroundings() {
  // Look left
  currentAngle = 180;
  sensorServo.write(currentAngle);
  delay(500);  // Allow servo to reach position
  int leftDistance = measureDistance();
  
  // Look right
  currentAngle = 0;
  sensorServo.write(currentAngle);
  delay(500);  // Allow servo to reach position
  int rightDistance = measureDistance();
  
  // Print readings
  Serial.print("Left distance: ");
  Serial.print(leftDistance);
  Serial.print(" cm, Right distance: ");
  Serial.print(rightDistance);
  Serial.println(" cm");
  
  // Return to center
  currentAngle = 90;
  sensorServo.write(currentAngle);
  delay(200);
}

// Function to measure distance using ultrasonic sensor
int measureDistance() {
  // Clear trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send 10μs pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure the response
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate distance in centimeters
  int distance = duration * 0.034 / 2;
  
  return distance;
}

// Function to move car forward
void moveForward() {
  // Set motor directions
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Set motor speeds
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

// Function to stop car
void stopCar() {
  // Stop both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  // Set motor speeds to 0
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}


