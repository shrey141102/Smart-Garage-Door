#include <Servo.h>

// Pin definitions
const int trigPin = 9;
const int echoPin = 10;
const int greenLedPin = 13;
const int redLedPin = 12;
const int servoPin = 11;

// Create servo object
Servo gateServo;

// Define variables for distance measurement
long duration;
int distance;

// Random verification threshold for 50/50 chance
int verificationThreshold = 50; // 50/50 chance for vehicle verification

// Setup function
void setup() {
  // Start serial monitor for debugging
  Serial.begin(9600);

  // Initialize pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  // Attach the servo to the servo pin
  gateServo.attach(servoPin);
  gateServo.write(90); // Start with the gate in neutral position (closed)

  // Make sure LEDs are off at the start
  digitalWrite(greenLedPin, LOW);
  digitalWrite(redLedPin, LOW);

  delay(1000); // Wait for systems to stabilize
}

void loop() {
  // Measure the distance using the ultrasonic sensor
  distance = getDistance();

  // Debug output to serial monitor
  Serial.print("Distance: ");
  Serial.println(distance);

  // If a vehicle is detected (distance less than 30 cm)
  if (distance < 30) {
    Serial.println("Vehicle detected!");
    digitalWrite(greenLedPin, HIGH); // Turn on green LED

    // Simulate random vehicle verification
    int verification = random(0, 100); // Random number between 0 and 100
    Serial.print("Verification value: ");
    Serial.println(verification);

    // Check if the verification passes (50/50 chance)
    if (verification > verificationThreshold) {
      Serial.println("Verification passed. Opening the gate...");
      openGate();  // Open the gate if verified
    } else {
      Serial.println("Verification failed. Turning on red LED...");
      blinkRedLed();  // Blink the red LED if verification fails
    }

  } else {
    // If no vehicle detected, keep the LEDs off and gate closed
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, LOW);
    gateServo.write(90); // Stop the servo (neutral position)
  }

  delay(500); // Small delay before next sensor reading
}

// Function to get distance from the ultrasonic sensor
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2; // Calculate distance in cm

  return distance;
}

// Function to open the gate by rotating the servo (open)
void openGate() {
  gateServo.write(180); // Rotate servo to 180 degrees (open gate)
  delay(3000); // Keep gate open for 3 seconds
  gateServo.write(90); // Stop the servo (neutral position)
}

// Function to blink red LED when verification fails
void blinkRedLed() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(redLedPin, HIGH); // Turn on red LED
    delay(500); // Wait for 500 ms
    digitalWrite(redLedPin, LOW);  // Turn off red LED
    delay(500); // Wait for 500 ms
  }
}



