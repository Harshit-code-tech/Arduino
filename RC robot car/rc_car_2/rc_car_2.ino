#include <Servo.h>
#include <AFMotor.h>

#define Echo A2               // Echo pin for ultrasonic sensor
#define Trig A3               // Trigger pin for ultrasonic sensor
#define motor 10              // Servo pin for steering
#define Speed 170             // Motor speed setting
#define spoint 103            // Servo position for straight
#define movSpeed 200          // Unused speed variable

char value;                   // Variable to store command from Bluetooth/voice
int distance;                // Variable to store distance from ultrasonic sensor
int Left;                    // Variable to store left distance
int Right;                   // Variable to store right distance
int L = 0;                   // Left distance check
int R = 0;                   // Right distance check
int L1 = 0;                  // Unused
int R1 = 0;                  // Unused

Servo servo;                 // Create Servo object
AF_DCMotor M1(1);           // Motor 1
AF_DCMotor M2(2);           // Motor 2
AF_DCMotor M3(3);           // Motor 3
AF_DCMotor M4(4);           // Motor 4

unsigned long previousMillis = 0; // Store previous time for obstacle checking
const long interval = 100;        // Interval to check for obstacles (in milliseconds)
bool obstacleAvoidanceActive = false; // Flag for obstacle avoidance status

void setup() {
  Serial.begin(9600);           // Initialize serial communication
  pinMode(Trig, OUTPUT);        // Set trigger pin as output
  pinMode(Echo, INPUT);         // Set echo pin as input
  servo.attach(motor);          // Attach servo to defined pin
  M1.setSpeed(Speed);           // Set speed for all motors
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
}

void loop() {
  // Check for incoming commands from Bluetooth or voice
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
    Bluetoothcontrol();          // Handle Bluetooth commands
    voicecontrol();              // Handle voice commands
  }

  // Check for obstacles at defined intervals
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    checkObstacle();            // Check for obstacles
  }
}

// Adjusted movement mapping based on circuit errors
// Forward --> Right
// Backward --> Left
// Right --> Forward
// Left --> Backward
void Bluetoothcontrol() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
  }
  if (value == 'R') {
    forward();                 // Forward
  } else if (value == 'L') {
    backward();                // Backward
  } else if (value == 'B') {
    left();                   // Turn left
  } else if (value == 'F') {
    right();                  // Turn right
  } else if (value == 'S') {
    Stop();                   // Stop
  } else if (value == 'G') {
    forwardLeft();           // Forward left
  } else if (value == 'I') {
    forwardRight();          // Forward right
  } else if (value == 'H') {
    backwardLeft();          // Backward left
  } else if (value == 'J') {
    backwardRight();         // Backward right
  }
}

void voicecontrol() {
  if (value == '^') {
    right();
    delay(500); // Turn right for 500 ms
    Stop();
  } else if (value == '-') {
    left();
    delay(500); // Turn left for 500 ms
    Stop();
  } else if (value == '<') {
    L = leftsee();             // Check left distance
    servo.write(spoint);
    if (L >= 10) {
      backward();
      delay(500);
      Stop();
    } else {
      Stop();
    }
  } else if (value == '>') {
    R = rightsee();            // Check right distance
    servo.write(spoint);
    if (R >= 10) {
      forward();
      delay(500);
      Stop();
    } else {
      Stop();
    }
  } else if (value == '*') {
    Stop();                   // Stop
  }
}

void checkObstacle() {
  distance = ultrasonic();     // Get distance from ultrasonic sensor
  if (distance <= 15) {        // Check if an obstacle is detected
    obstacleAvoidanceActive = true;
    Stop();
    left();                     // Turn left
    delay(500);
    Stop();
    
    L = leftsee();             // Measure left distance
    servo.write(spoint);
    delay(800);
    R = rightsee();            // Measure right distance
    servo.write(spoint);
    
    // Choose direction based on distance
    if (L < R) {
      forward();
      delay(500);
      Stop();
      delay(200);
    } else {
      backward();
      delay(500);
      Stop();
      delay(200);
    }
    
    obstacleAvoidanceActive = false; // Reset flag
  }
}

int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  long cm = t / 29 / 2; // Convert time to distance in cm
  return cm;
}

void forward() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

void backward() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

void right() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}

void left() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}

void Stop() {
  M1.run(RELEASE);             // Stop all motors
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}

int rightsee() {
  servo.write(20);            // Turn servo to the right
  delay(800);
  int Right = ultrasonic();   // Measure distance
  return Right;
}

int leftsee() {
  servo.write(180);           // Turn servo to the left
  delay(800);
  int Left = ultrasonic();     // Measure distance
  return Left;
}

void forwardLeft() {
  M1.run(BACKWARD);           // Bottom right motor
  M2.run(BACKWARD);           // Bottom left motor
  M3.run(BACKWARD);           // Top right motor
  M4.run(FORWARD);            // Top left motor
}

void forwardRight() {
  M1.run(BACKWARD);           // Bottom right motor
  M2.run(FORWARD);            // Bottom left motor
  M3.run(FORWARD);            // Top right motor
  M4.run(FORWARD);            // Top left motor
}

void backwardLeft() {
  M1.run(FORWARD);            // Bottom right motor
  M2.run(FORWARD);            // Bottom left motor
  M3.run(FORWARD);            // Top right motor
  M4.run(BACKWARD);           // Top left motor
}

void backwardRight() {
  M1.run(FORWARD);            // Bottom right motor
  M2.run(BACKWARD);           // Bottom left motor
  M3.run(BACKWARD);           // Top right motor
  M4.run(BACKWARD);           // Top left motor
}
