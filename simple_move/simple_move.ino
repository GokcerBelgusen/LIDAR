// Define motor control pins
#define IN1 D0
#define IN2 D1
#define IN3 D2
#define IN4 D3

void setup() {
  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  moveForward();
  delay(2000);
  stopMotors();
  delay(1000);

  moveBackward();
  delay(2000);
  stopMotors();
  delay(1000);

  turnLeft();
  delay(1000);
  stopMotors();
  delay(1000);

  turnRight();
  delay(1000);
  stopMotors();
  delay(1000);

}

// Function to move forward
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Function to move backward
void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Function to turn left
void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Function to turn right
void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  // Example usage: uncomment one command at a time to test
  /*moveForward();
    delay(2000);
    stopMotors();
    delay(1000);

    moveBackward();
    delay(2000);
    stopMotors();
    delay(1000);

    turnLeft();
    delay(1000);
    stopMotors();
    delay(1000);

    turnRight();
    delay(1000);
    stopMotors();
    delay(1000);*/
}
