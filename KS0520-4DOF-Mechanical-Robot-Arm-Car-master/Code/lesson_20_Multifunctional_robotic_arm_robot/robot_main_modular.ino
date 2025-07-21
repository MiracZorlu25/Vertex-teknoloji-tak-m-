// robot_main_modular.ino
// Modular main sketch for robot-side Deneyap Card
// Controls motors, servos, sensors, and receives commands via Bluetooth
// Includes safety features and clear structure for easy modification

#include <Servo.h>
// #include <BluetoothSerial.h> // Uncomment if using ESP32/Deneyap
// #include <Wire.h> // For I2C sensors or OLED

// ----- Pin Definitions -----
#define MOTOR_A_DIR 2
#define MOTOR_A_PWM 3
#define MOTOR_B_DIR 4
#define MOTOR_B_PWM 5
#define SERVO1_PIN A1
#define SERVO2_PIN A0
#define SERVO3_PIN 8
#define SERVO4_PIN 9
#define ECHO_PIN A3
#define TRIG_PIN A4
#define BATTERY_PIN A2 // Analog pin for battery voltage monitoring

// ----- Global Variables -----
Servo servo1, servo2, servo3, servo4;
int pos1 = 90, pos2 = 100, pos3 = 80, pos4 = 90;
int speeds = 100;
unsigned long lastCommandTime = 0;
const unsigned long commsTimeout = 2000; // ms

// ----- Function Prototypes -----
void setupMotors();
void setupServos();
void setupSensors();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
void setServoPositions(int, int, int, int);
int readBatteryVoltage();
void checkSafety();
void processCommand(char);
void obstacleAvoidance();

void setup() {
  Serial.begin(9600); // For Bluetooth module (e.g., HC-05)
  setupMotors();
  setupServos();
  setupSensors();
  stopMotors();
  setServoPositions(pos1, pos2, pos3, pos4);
  lastCommandTime = millis();
}

void loop() {
  // Check for incoming Bluetooth commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    processCommand(cmd);
    lastCommandTime = millis();
  }
  checkSafety();
  // Optionally, add periodic sensor checks or status updates here
}

// ----- Setup Functions -----
void setupMotors() {
  pinMode(MOTOR_A_DIR, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_DIR, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
}
void setupServos() {
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
}
void setupSensors() {
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
}

// ----- Motor Control -----
void moveForward() {
  digitalWrite(MOTOR_A_DIR, LOW);
  analogWrite(MOTOR_A_PWM, speeds);
  digitalWrite(MOTOR_B_DIR, HIGH);
  analogWrite(MOTOR_B_PWM, speeds);
}
void moveBackward() {
  digitalWrite(MOTOR_A_DIR, HIGH);
  analogWrite(MOTOR_A_PWM, speeds);
  digitalWrite(MOTOR_B_DIR, LOW);
  analogWrite(MOTOR_B_PWM, speeds);
}
void turnLeft() {
  digitalWrite(MOTOR_A_DIR, HIGH);
  analogWrite(MOTOR_A_PWM, speeds);
  digitalWrite(MOTOR_B_DIR, HIGH);
  analogWrite(MOTOR_B_PWM, speeds);
}
void turnRight() {
  digitalWrite(MOTOR_A_DIR, LOW);
  analogWrite(MOTOR_A_PWM, speeds);
  digitalWrite(MOTOR_B_DIR, LOW);
  analogWrite(MOTOR_B_PWM, speeds);
}
void stopMotors() {
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
}

// ----- Servo Control -----
void setServoPositions(int p1, int p2, int p3, int p4) {
  servo1.write(p1);
  servo2.write(p2);
  servo3.write(p3);
  servo4.write(p4);
}

// ----- Battery Monitoring -----
int readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  // Convert raw ADC value to voltage (adjust for voltage divider if used)
  float voltage = raw * (5.0 / 1023.0) * 3; // Example: 3x divider
  return (int)(voltage * 100); // Return as integer (e.g., 123 = 12.3V)
}

// ----- Safety Check -----
void checkSafety() {
  // Communication loss shutdown
  if (millis() - lastCommandTime > commsTimeout) {
    stopMotors();
    // Optionally, move arm to safe position
  }
  // Battery voltage warning
  int batt = readBatteryVoltage();
  if (batt < 1100) { // 11.0V threshold for 3S LiPo
    // Add warning: blink LED, send status, etc.
    stopMotors();
  }
}

// ----- Command Processing -----
void processCommand(char cmd) {
  switch (cmd) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopMotors(); break;
    case 'U': obstacleAvoidance(); break;
    // Add more cases for servo/arm control, speed, etc.
    default: stopMotors(); break;
  }
}

// ----- Obstacle Avoidance Example -----
void obstacleAvoidance() {
  // Simple ultrasonic avoidance logic
  long duration;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration / 58;
  if (distance < 30 && distance > 2) {
    moveBackward();
    delay(300);
    turnRight();
    delay(300);
  } else {
    moveForward();
  }
}

// Add more modular functions as needed for sensors, status reporting, etc. 