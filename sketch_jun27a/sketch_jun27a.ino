#include <Servo.h>
#include <AFMotor.h>

#define Echo A3
#define Trig A4
#define motor 10
#define Speed 170
#define spoint 103
#define SafeDistance 10 // Safe distance in cm

char value;
int distance;
int Left;
int Right;
int L = 0;
int R = 0;

Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  servo.attach(motor);
  servo.write(spoint); // Center the servo
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
  Serial.println("Setup complete.");
}

void loop() {
  distance = ultrasonic();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= SafeDistance) {
    Serial.println("Obstacle detected! Stopping and scanning...");
    Stop();
    scan();
  } else {
    forward();
    Serial.println("Moving forward.");
  }

  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.print("Received command: ");
    Serial.println(value);
    Bluetoothcontrol();
    voicecontrol();
  }
}

void Bluetoothcontrol() {
  switch (value) {
    case 'R': 
      Serial.println("Moving forward.");
      forward(); 
      break;
    case 'L': 
      Serial.println("Moving backward.");
      backward(); 
      break;
    case 'B': 
      Serial.println("Turning left.");
      left(); 
      break;
    case 'F': 
      Serial.println("Turning right.");
      right(); 
      break;
    case 'S': 
      Serial.println("Stopping.");
      Stop(); 
      break;
  }
  value = 0; // Reset value to avoid repeated command
}

void voicecontrol() {
  switch (value) {
    case '^': 
      Serial.println("Turning right.");
      right(); 
      break;
    case '-': 
      Serial.println("Turning left.");
      left(); 
      break;
    case '<':
      L = leftsee();
      Serial.print("Left distance: ");
      Serial.print(L);
      Serial.println(" cm");
      servo.write(spoint);
      if (L >= SafeDistance) {
        Serial.println("Moving backward.");
        backward();
        delay(500);
        Stop();
      } else {
        Serial.println("Stopping.");
        Stop();
      }
      break;
    case '>':
      R = rightsee();
      Serial.print("Right distance: ");
      Serial.print(R);
      Serial.println(" cm");
      servo.write(spoint);
      if (R >= SafeDistance) {
        Serial.println("Moving forward.");
        forward();
        delay(500);
        Stop();
      } else {
        Serial.println("Stopping.");
        Stop();
      }
      break;
    case '*': 
      Serial.println("Stopping.");
      Stop(); 
      break;
  }
  value = 0; // Reset value to avoid repeated command
}

int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  long cm = t / 29 / 2; // Time converted to distance
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
  int distance = ultrasonic();
  servo.write(spoint);
  return distance;
}

int leftsee() {
  servo.write(180);
  delay(800);
  int distance = ultrasonic();
  servo.write(spoint);
  return distance;
}

void scan() {
  left();
  delay(200);
  Stop();
  L = leftsee();
  Serial.print("Left distance: ");
  Serial.print(L);
  Serial.println(" cm");

  right();
  delay(200);
  Stop();
  R = rightsee();
  Serial.print("Right distance: ");
  Serial.print(R);
  Serial.println(" cm");

  servo.write(spoint); // Center the servo

  if (L > SafeDistance && R > SafeDistance) {
    forward();
    delay(500);
    Stop();
  } else if (L > R) {
    left();
    delay(500);
    Stop();
  } else if (R > L) {
    right();
    delay(500);
    Stop();
  } else {
    backward();
    delay(500);
    Stop();
  }
}
