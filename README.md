# Fast Line Follower Code

Arduino-based fast line follower robot using PID control and a 5-channel IR sensor array.

## Hardware

| Component | Description |
|-----------|-------------|
| Microcontroller | Arduino Uno (or compatible) |
| Motor driver | L298N dual H-bridge |
| Motors | 2 × DC gear-motors |
| Sensors | 5-channel IR sensor array (analog) |

## Wiring

### Motor driver

| L298N pin | Arduino pin |
|-----------|-------------|
| ENA | 5 (PWM) |
| IN1 | 8 |
| IN2 | 9 |
| ENB | 6 (PWM) |
| IN3 | 10 |
| IN4 | 11 |

### IR sensor array

| Sensor | Arduino pin |
|--------|-------------|
| S0 (left-most) | A0 |
| S1 | A1 |
| S2 (centre) | A2 |
| S3 | A3 |
| S4 (right-most) | A4 |

## Getting started

1. Open `fast_line_follower/fast_line_follower.ino` in the Arduino IDE.
2. Connect your hardware according to the wiring tables above (or change the pin definitions in the sketch).
3. Upload the sketch to your Arduino.
4. Place the robot on the track. During the first **3 seconds** the sensors will calibrate — slowly sweep the sensor array across the line and the background.
5. After calibration the robot will start following the line automatically.

## Tuning

The PID constants (`Kp`, `Ki`, `Kd`) and `BASE_SPEED` are defined near the top of the sketch. Adjust them to match your robot's weight, motor speed, and track layout:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `Kp` | 25.0 | Proportional gain — increases responsiveness to error |
| `Ki` | 0.0 | Integral gain — corrects accumulated steady-state error |
| `Kd` | 15.0 | Derivative gain — dampens oscillations |
| `BASE_SPEED` | 150 | Base motor PWM (0–255) |

## License

This project is provided as-is for educational purposes.