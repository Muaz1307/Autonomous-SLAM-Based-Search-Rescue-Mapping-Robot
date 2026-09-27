#include <ESP32Servo.h>
#include <Wire.h>

// ---- Motor pins ----
#define BL_IN1 21
#define BL_IN2 47
#define BL_EN   9
#define BR_IN1 38
#define BR_IN2 39
#define BR_EN  40
#define FR_IN1 41
#define FR_IN2 42
#define FR_EN   2
#define FL_IN1  1
#define FL_IN2  8
#define FL_EN  18

// ---- Encoder pins ----
#define ENC_FL 3
#define ENC_FR 15

// ---- Sonar pins ----
#define SONAR1_ECHO 4   // on servo1, obstacle detection
#define SONAR1_TRIG 5
#define SONAR2_ECHO 6   // fixed, mapping
#define SONAR2_TRIG 7

// ---- Servo pins ----
#define SERVO1_PIN 12   // sonar1 sweep mount
#define SERVO2_PIN 13   // arm base
#define SERVO3_PIN 14   // arm rotation

// ---- MPU6500 (raw register, since WHO_AM_I=0x70) ----
#define SDA_PIN 10
#define SCL_PIN 11
#define MPU_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define GYRO_ZOUT_H  0x47

// ---- CALIBRATE these from your bench testing ----
#define PULSES_PER_CM   0.98
#define TURN_90_PULSES  40

#define DRIVE_SPEED 120
#define TURN_SPEED  80
#define OBSTACLE_THRESHOLD_CM 50

// ---- Map grid ----
#define GRID_SIZE 6
#define CELL_SIZE_CM 10
int8_t grid[GRID_SIZE][GRID_SIZE]; // -1 unknown, 0 free, 1 occupied

// ---- Position tracking ----
float posX = 0, posY = 0;
float heading = 0;
volatile long pulseFL = 0, pulseFR = 0;
long lastPulseAvg = 0;
unsigned long lastHeadingUpdate = 0;

// ---- Servo objects ----
Servo servo1, servo2, servo3;
int sweepAngle = 0;
int sweepDir = 1;
unsigned long lastSweepStep = 0;

// ---- 45-sec arm rotation routine (non-blocking) ----
enum RoutineState { R_IDLE, R_S3_ROTATE };
RoutineState routineState = R_IDLE;
unsigned long routineTimer = 0;
unsigned long lastRoutineTrigger = 0;
int servo2Base = 90; // confirmed correct

// ---- Motor control ----
// NOTE: BL and FR are wired with inverted polarity at the driver terminals,
// so their direction is flipped in software to match BR/FL.
void setMotor(int inA, int inB, int enPin, int dir, int speed) {
  digitalWrite(inA, dir < 0 ? HIGH : LOW);
  digitalWrite(inB, dir > 0 ? HIGH : LOW);
  analogWrite(enPin, dir == 0 ? 0 : speed);
}
void allMotors(int dirBL, int dirBR, int dirFR, int dirFL, int speed) {
  setMotor(BL_IN1, BL_IN2, BL_EN, -dirBL, speed);  // inverted wiring
  setMotor(BR_IN1, BR_IN2, BR_EN, dirBR, speed);
  setMotor(FR_IN1, FR_IN2, FR_EN, -dirFR, speed);  // inverted wiring
  setMotor(FL_IN1, FL_IN2, FL_EN, dirFL, speed);
}
void stopAll() { allMotors(0, 0, 0, 0, 0); }
void driveForward() { allMotors(1, 1, 1, 1, DRIVE_SPEED); }

void isrFL() { pulseFL++; }
void isrFR() { pulseFR++; }

// ---- MPU6500 raw register access ----
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2);
  int16_t raw = Wire.read() << 8 | Wire.read();
  return raw / 131.0;
}

// ---- Sonar ----
float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.0343 / 2.0;
}

// ---- Position/heading update ----
void updateHeading() {
  unsigned long now = millis();
  float dt = (now - lastHeadingUpdate) / 1000.0;
  lastHeadingUpdate = now;
  float gz = readGyroZ();
  heading += gz * dt;
  if (heading >= 360) heading -= 360;
  if (heading < 0) heading += 360;
}

void updatePosition() {
  long avg = (pulseFL + pulseFR) / 2;
  long delta = avg - lastPulseAvg;
  lastPulseAvg = avg;
  float distCm = delta / PULSES_PER_CM;
  float rad = heading * PI / 180.0;
  posX += distCm * cos(rad);
  posY += distCm * sin(rad);
}

// ---- Bresenham ray tracing: marks path cells FREE, endpoint OCCUPIED ----
void traceRay(int x0, int y0, int x1, int y1, bool hitObstacle) {
  int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
  int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy, e2;

  while (true) {
    bool isEnd = (x0 == x1 && y0 == y1);
    if (x0 >= 0 && x0 < GRID_SIZE && y0 >= 0 && y0 < GRID_SIZE) {
      if (isEnd && hitObstacle) {
        grid[y0][x0] = 1;
      } else if (grid[y0][x0] != 1) {
        grid[y0][x0] = 0;
      }
    }
    if (isEnd) break;
    e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

// ---- Map update from sonar2, with ray tracing ----
void mapUpdateFromSonar2() {
  float dist = readDistanceCM(SONAR2_TRIG, SONAR2_ECHO);

  int rx = GRID_SIZE / 2 + (int)(posX / CELL_SIZE_CM);
  int ry = GRID_SIZE / 2 + (int)(posY / CELL_SIZE_CM);

  if (dist < 0) {
    // No echo - don't guess, leave the ray direction UNKNOWN
    if (rx >= 0 && rx < GRID_SIZE && ry >= 0 && ry < GRID_SIZE) grid[ry][rx] = 0;
    return;
  }

  float rad = heading * PI / 180.0;
  float ox = posX + dist * cos(rad);
  float oy = posY + dist * sin(rad);
  int gx = GRID_SIZE / 2 + (int)(ox / CELL_SIZE_CM);
  int gy = GRID_SIZE / 2 + (int)(oy / CELL_SIZE_CM);

  traceRay(rx, ry, gx, gy, true);
}

// ---- Sonar1 sweep + obstacle avoidance ----
void sonar1SweepStep() {
  unsigned long now = millis();
  if (now - lastSweepStep < 50) return;
  lastSweepStep = now;

  sweepAngle += sweepDir * 10;
  if (sweepAngle >= 180) { sweepAngle = 180; sweepDir = -1; }
  if (sweepAngle <= 0)   { sweepAngle = 0;   sweepDir = 1; }
  servo1.write(sweepAngle);

  float dist = readDistanceCM(SONAR1_TRIG, SONAR1_ECHO);
  if (dist > 0 && dist < OBSTACLE_THRESHOLD_CM) {
    Serial.print("Obstacle at "); Serial.print(dist); Serial.println("cm - avoiding");
    stopAll();
    delay(200);
    pulseFL = 0; pulseFR = 0;
    allMotors(1, -1, -1, 1, TURN_SPEED);
    while (((pulseFL + pulseFR) / 2) < (TURN_90_PULSES / 3)) delay(5);
    stopAll();
  }
}

// ---- 45-sec arm rotation routine: servo3 90 -> back to 0 (servo2 fixed) ----
void servoRoutineUpdate() {
  unsigned long now = millis();

  if (routineState == R_IDLE && now - lastRoutineTrigger >= 45000) {
    lastRoutineTrigger = now;
    servo3.write(90);
    routineTimer = now;
    routineState = R_S3_ROTATE;
    Serial.println("Routine: servo3 rotate 90");
  }
  else if (routineState == R_S3_ROTATE && now - routineTimer >= 500) {
    servo3.write(0);
    routineTimer = now;
    routineState = R_IDLE;
    Serial.println("Routine: servo3 back to 0, cycle complete");
  }
}

// ---- Print map to Serial Monitor ----
void printMapToSerial() {
  Serial.println("---- MAP ----");
  for (int y = 0; y < GRID_SIZE; y++) {
    String row = "";
    for (int x = 0; x < GRID_SIZE; x++) {
      if (grid[y][x] == -1) row += "?";
      else if (grid[y][x] == 0) row += ".";
      else row += "#";
    }
    Serial.println(row);
  }
  Serial.println("-------------");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  int motorPins[] = {BL_IN1, BL_IN2, BL_EN, BR_IN1, BR_IN2, BR_EN,
                      FR_IN1, FR_IN2, FR_EN, FL_IN1, FL_IN2, FL_EN};
  for (int i = 0; i < 12; i++) pinMode(motorPins[i], OUTPUT);

  pinMode(SONAR1_TRIG, OUTPUT); pinMode(SONAR1_ECHO, INPUT);
  pinMode(SONAR2_TRIG, OUTPUT); pinMode(SONAR2_ECHO, INPUT);

  pinMode(ENC_FL, INPUT); pinMode(ENC_FR, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_FL), isrFL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR), isrFR, RISING);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo2.write(servo2Base - 30); // fixed position, set once
  servo3.write(0);

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  writeReg(PWR_MGMT_1, 0x00);
  delay(100);

  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      grid[y][x] = -1;

  lastHeadingUpdate = millis();
  lastRoutineTrigger = millis();
  Serial.println("Setup complete. Starting exploration...");
  delay(1000);
}

void loop() {
  updateHeading();
  updatePosition();
  sonar1SweepStep();
  servoRoutineUpdate();

  static unsigned long lastMapUpdate = 0;
  if (millis() - lastMapUpdate > 150) {
    mapUpdateFromSonar2();
    lastMapUpdate = millis();
  }

  static unsigned long lastMapPrint = 0;
  if (millis() - lastMapPrint > 1000) {
    printMapToSerial();
    lastMapPrint = millis();
  }

  driveForward();
}
