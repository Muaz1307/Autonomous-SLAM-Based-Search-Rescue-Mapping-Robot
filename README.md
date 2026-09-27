# Autonomous SLAM-Based Search & Rescue Mapping Robot

**Course:** CSE461 – Introduction to Robotics, BRAC University
**Author:** Muaz Abdur Rahim (22301395)

An ESP32-S3–controlled 4WD rover that explores an unknown space, avoids obstacles, builds a simplified 2D occupancy grid map from ultrasonic sensor data, and periodically actuates a servo-driven marker arm. Designed as a low-cost (~BDT 7,700) educational platform for search & rescue concepts.

---

## 1. Hardware

| Component | Role |
|---|---|
| ESP32-S3 Dev Board | Main controller (sensor fusion, motor control, mapping) |
| 2× HC-SR04 Ultrasonic | Sonar 1 (servo-mounted, sweeping): obstacle avoidance. Sonar 2 (fixed, forward-facing): occupancy grid mapping |
| MPU6500 (6050-series IMU, raw register access) | Yaw/heading via gyro-Z integration |
| 2× HC-020 Wheel Encoders (FL, FR) | Pulse counting for distance traveled |
| 3× Servo motors (MG90/SG90) | Servo 1: sonar sweep mount. Servo 2: arm base (fixed position). Servo 3: arm rotation (periodic marker routine) |
| L298N Motor Driver | Drives 4 DC gear motors (BL, BR, FR, FL) via PWM |
| 4WD Rover Chassis | Mechanical platform |
| 2× 18650 Li-ion + BMS, TP4056 | Rechargeable power supply and charging protection |
| HLK-LD2410C mmWave sensor | Included in BOM/design for human-presence detection |

### Pin Map (from firmware)

| Function | Pin(s) |
|---|---|
| Back-Left motor (IN1/IN2/EN) | 21 / 47 / 9 |
| Back-Right motor (IN1/IN2/EN) | 38 / 39 / 40 |
| Front-Right motor (IN1/IN2/EN) | 41 / 42 / 2 |
| Front-Left motor (IN1/IN2/EN) | 1 / 8 / 18 |
| Encoder – Front Left | 3 |
| Encoder – Front Right | 15 |
| Sonar 1 (sweep) – Trig / Echo | 5 / 4 |
| Sonar 2 (fixed) – Trig / Echo | 7 / 6 |
| Servo 1 (sonar sweep) | 12 |
| Servo 2 (arm base) | 13 |
| Servo 3 (arm rotation) | 14 |
| I2C – SDA / SCL (IMU) | 10 / 11 |

> **Note:** The Back-Left and Front-Right motors are wired with inverted polarity at the driver terminals; this is compensated in software (`allMotors()`), not in hardware.

---

## 2. Software

- **IDE:** Arduino IDE with ESP32 Board Support Package
- **Language:** Embedded C/C++
- **Libraries:** `ESP32Servo`, `Wire` (I2C)
- **Debug/telemetry:** Serial Monitor at 115200 baud (prints the live occupancy grid every second)

### Key tunable constants (`main_robot_code.ino`)

```cpp
#define PULSES_PER_CM   0.98   // encoder calibration — re-measure per robot
#define TURN_90_PULSES  40     // encoder pulses for a 90° turn — re-measure per robot
#define DRIVE_SPEED     120
#define TURN_SPEED      80
#define OBSTACLE_THRESHOLD_CM 50
#define GRID_SIZE       6      // 6x6 occupancy grid
#define CELL_SIZE_CM    10     // each cell = 10cm x 10cm
```

Recalibrate `PULSES_PER_CM` and `TURN_90_PULSES` on the bench for your specific wheels/encoders before relying on the position estimate.

---

## 3. How It Works

1. **Setup:** Initializes motor pins, sonar pins, encoder interrupts, servos, and the IMU (wakes the MPU via `PWR_MGMT_1` register). Grid cells are all set to `-1` (unknown).
2. **Localization:** Every loop iteration, heading is updated by integrating gyro-Z (`updateHeading`), and position is updated by averaging front-left/front-right encoder pulse deltas and projecting along the current heading (`updatePosition`) — a simple dead-reckoning sensor-fusion approach.
3. **Obstacle avoidance:** Sonar 1 sweeps 0°–180° in 10° steps (`sonar1SweepStep`). If an obstacle is detected within `OBSTACLE_THRESHOLD_CM`, the robot stops, reverses direction on one side, turns roughly 30° (1/3 of a 90° turn), and resumes.
4. **Occupancy grid mapping:** Sonar 2 (fixed forward) measures distance every 150 ms. The robot's current cell and the detected obstacle's cell are computed from `posX/posY` and `heading`, then a Bresenham ray-trace (`traceRay`) marks every cell along that ray as free and the endpoint as occupied.
5. **Marker/arm routine:** Every 45 seconds, Servo 3 rotates to 90° for 0.5s then returns to 0° — a placeholder periodic actuation for marking a location (non-blocking, timer-based).
6. **Telemetry:** The 6×6 grid (`?` unknown, `.` free, `#` occupied) is printed to Serial once per second.
7. **Driving:** The robot drives forward continuously between obstacle-avoidance maneuvers (no explicit stop/return-home state machine in this firmware version — see below).

---

## 4. Getting Started

1. Install the **ESP32 board package** in Arduino IDE (Boards Manager → search "esp32").
2. Install the **ESP32Servo** library (Library Manager).
3. Select your ESP32-S3 board variant and correct COM port.
4. Wire the hardware per the pin map above; ensure a **common ground** between the ESP32, motor driver, and any separately-regulated servo supply.
5. Measure `PULSES_PER_CM` and `TURN_90_PULSES` for your build and update the `#define`s.
6. Upload `main_robot_code.ino`.
7. Open Serial Monitor at **115200 baud** to view live grid output and status messages.

---

## 5. Project Status & Known Gaps

The uploaded firmware implements the **core mapping/navigation loop** (sensor fusion, obstacle avoidance, occupancy grid, periodic servo actuation). A few features described in the project report are **not yet present in this code version**:

- **Wi-Fi telemetry / remote monitoring station:** not implemented — no `WiFi.h` usage or transmission code in the current sketch.
- **Frontier-based exploration:** current behavior is "drive forward, turn away from obstacles" rather than active frontier selection over the grid.
- **Return-to-home routine:** odometry (`posX`, `posY`, `heading`) is tracked, but no return-to-start state machine is implemented yet.

---

## 7. Cost Summary

| Component | Cost (BDT) |
|---|---|
| ESP32-S3 Dev Board | 890 |
| HC-SR04 Ultrasonic ×4 | 500 |
| MPU6050 IMU | 250 |
| HLK-LD2410C | 740 |
| Wheel Encoders ×2 | 390 |
| SG90 Servo ×2 | 480 |
| L298N Driver | 895 |
| 4WD/Tank Chassis | 1350 |
| 18650 Battery + BMS ×2 | 500 |
| TP4056 Charger | 350 |
| Misc. | 300 |
| **Total** | **~7,695** |

---

## 8. License / Attribution

Educational project for CSE461, BRAC University. Adapt freely for coursework and non-commercial robotics learning.
