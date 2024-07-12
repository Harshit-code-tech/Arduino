#include <Servo.h>
#include <AFMotor.h>

#define Echo A2
#define Trig A3
#define motor 10
#define Speed 170
#define spoint 103
#define movSpeed 200

char value;
int distance;
int Left;
int Right;
int L = 0;
int R = 0;
int L1 = 0;
int R1 = 0;

Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

int LED1 = 9;  // Define additional LED pin 1
int LED2 = 10; // Define additional LED pin 2

void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(LED1, OUTPUT); // Initialize LED pin 1
  pinMode(LED2, OUTPUT); // Initialize LED pin 2
  servo.attach(motor);
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
}

void loop() {
  //Obstacle();
  Bluetoothcontrol();
  voicecontrol();
}

void Bluetoothcontrol() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
  }
  if (value == 'R') {
    forward();
    digitalWrite(LED1, HIGH); // Turn on LED1 when moving forward
  } else if (value == 'L') {
    backward();
    digitalWrite(LED2, HIGH); // Turn on LED2 when moving backward
  } else if (value == 'B') {
    left();
    digitalWrite(LED2, HIGH); // Turn on LED2 when moving left
  } else if (value == 'F') {
    right();
    digitalWrite(LED1, HIGH); // Turn on LED1 when moving right
  } else if (value == 'S') {
    Stop();
    digitalWrite(LED1, LOW); // Turn off LED1 when stopping
    digitalWrite(LED2, LOW); // Turn off LED2 when stopping
  } else if (value == 'G') {
    forwardLeft();
    digitalWrite(LED1, HIGH); // Turn on LED1 when moving forward left
  } else if (value == 'I') {
    forwardRight();
    digitalWrite(LED1, HIGH); // Turn on LED1 when moving forward right
  } else if (value == 'H') {
    backwardLeft();
    digitalWrite(LED2, HIGH); // Turn on LED2 when moving backward left
  } else if (value == 'J') {
    backwardRight();
    digitalWrite(LED2, HIGH); // Turn on LED2 when moving backward right
  }
}

void Obstacle() {
  distance = ultrasonic();
  if (distance <= 10) { 
    Stop();
    left();
    delay(100);
    Stop();
    L = leftsee();
    servo.write(spoint);
    delay(800);
    R = rightsee();
    servo.write(spoint);
    if (L < R) {
      forward();
      delay(500);
      Stop();
      delay(200);
    } else if (L > R) {
      backward();
      delay(500);
      Stop();
      delay(200);
    }
  } else {
    right();
  }
}

void voicecontrol() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
    if (value == '^') {
      right();
      digitalWrite(LED1, HIGH); // Turn on LED1 when moving right
    } else if (value == '-') {
      left();
      digitalWrite(LED2, HIGH); // Turn on LED2 when moving left
    } else if (value == '<') {
      L = leftsee();
      servo.write(spoint);
      if (L >= 10 ) {
        backward();
        delay(500);
        Stop();
      } else if (L < 10) {
        Stop();
      }
    } else if (value == '>') {
      R = rightsee();
      servo.write(spoint);
      if (R >= 10 ) {
        forward();
        delay(500);
        Stop();
      } else if (R < 10) {
        Stop();
      }
    } else if (value == '*') {
      Stop();
      digitalWrite(LED1, LOW); // Turn off LED1 when stopping
      digitalWrite(LED2, LOW); // Turn off LED2 when stopping
    }
  }
}

// Ultrasonic sensor distance reading function
int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  long cm = t / 29 / 2; //time convert distance
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
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
}

int rightsee() {
  servo.write(20);
  delay(800);
  Left = ultrasonic();
  return Left;
}

int leftsee() {
  servo.write(180);
  delay(800);
  Right = ultrasonic();
  return Right;
}

void forwardLeft() {
  M1.run(BACKWARD);  // Bottom right
  M1.setSpeed(Speed);
  M2.run(BACKWARD);  // Bottom left
  M2.setSpeed(Speed);
  M3.run(BACKWARD);  // Top right
  M3.setSpeed(Speed);
  M4.run(FORWARD);   // Top left
  M4.setSpeed(movSpeed);
}

void forwardRight() {
  M1.run(BACKWARD);  // Bottom right
  M1.setSpeed(movSpeed);
  M2.run(FORWARD);   // Bottom left
  M2.setSpeed(Speed);
  M3.run(FORWARD);   // Top right
  M3.setSpeed(Speed);
  M4.run(FORWARD);   // Top left
  M4.setSpeed(Speed);
}

void backwardLeft() {
  M1.run(FORWARD);  // Bottom right
  M1.setSpeed(Speed);
  M2.run(FORWARD);  // Bottom left
  M2.setSpeed(Speed);
  M3.run(FORWARD);  // Top right
  M3.setSpeed(Speed);
  M4.run(BACKWARD); // Top left
  M4.setSpeed(movSpeed);
}

void backwardRight() {
  M1.run(FORWARD);  // Bottom right
  M1.setSpeed(movSpeed);
  M2.run(BACKWARD); // Bottom left
  M2.setSpeed(Speed);
  M3.run(BACKWARD); // Top right
  M3.setSpeed(Speed);
  M4.run(BACKWARD); // Top left
  M4.setSpeed(Speed);
}
