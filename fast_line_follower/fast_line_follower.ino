/*
 * Fast Line Follower Robot - PID Control
 *
 * Hardware:
 *   - 5-channel IR sensor array (analog pins A0–A4)
 *   - L298N (or compatible) dual H-bridge motor driver
 *   - Two DC gear-motors (left / right)
 *
 * Wiring (default pin mapping – change in the "Pin definitions" section):
 *   Motor A (Left):   ENA -> pin 5 (PWM), IN1 -> pin 8,  IN2 -> pin 9
 *   Motor B (Right):  ENB -> pin 6 (PWM), IN3 -> pin 10, IN4 -> pin 11
 *   Sensors:          S0 -> A0, S1 -> A1, S2 -> A2, S3 -> A3, S4 -> A4
 */

// ---------------------------------------------------------------------------
//  Pin definitions
// ---------------------------------------------------------------------------
// Motor A (left)
const int ENA = 5;
const int IN1 = 8;
const int IN2 = 9;

// Motor B (right)
const int ENB = 6;
const int IN3 = 10;
const int IN4 = 11;

// IR sensor array (left-most to right-most)
const int NUM_SENSORS  = 5;
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4};

// ---------------------------------------------------------------------------
//  Sensor calibration values (updated during calibration phase)
// ---------------------------------------------------------------------------
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];

// ---------------------------------------------------------------------------
//  PID parameters – tune these for your robot
// ---------------------------------------------------------------------------
float Kp = 25.0;
float Ki = 0.0;
float Kd = 15.0;

// ---------------------------------------------------------------------------
//  Speed settings
// ---------------------------------------------------------------------------
const int BASE_SPEED = 150;   // base PWM (0-255)
const int MAX_SPEED  = 255;

// ---------------------------------------------------------------------------
//  PID state
// ---------------------------------------------------------------------------
float lastError    = 0;
float integral     = 0;

// ---------------------------------------------------------------------------
//  setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialise calibration arrays
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  // ----- Calibration: sweep over black & white for ~3 seconds -----
  Serial.println("Calibrating sensors...");
  unsigned long calStart = millis();
  while (millis() - calStart < 3000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
  }
  Serial.println("Calibration complete.");
}

// ---------------------------------------------------------------------------
//  Read calibrated sensor values (0–1000) and compute weighted position
//  Returns a value in the range [0, 4000]:
//    0    = line is far left
//    2000 = line is centred
//    4000 = line is far right
// ---------------------------------------------------------------------------
int readLinePosition() {
  unsigned long weightedSum = 0;
  unsigned long sum         = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorPins[i]);

    // Map to 0-1000 using calibration data
    int range = sensorMax[i] - sensorMin[i];
    int calibrated;
    if (range == 0) {
      calibrated = 0;
    } else {
      calibrated = (long)(raw - sensorMin[i]) * 1000 / range;
    }
    if (calibrated < 0)    calibrated = 0;
    if (calibrated > 1000) calibrated = 1000;

    weightedSum += (unsigned long)calibrated * i * 1000;
    sum         += calibrated;
  }

  if (sum == 0) {
    // No line detected – use last known direction
    return (lastError < 0) ? 0 : 4000;
  }

  return weightedSum / sum;
}

// ---------------------------------------------------------------------------
//  Motor helpers
// ---------------------------------------------------------------------------
void setMotors(int leftSpeed, int rightSpeed) {
  // Left motor direction
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed;
  }

  // Right motor direction
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightSpeed = -rightSpeed;
  }

  // Clamp to [0, MAX_SPEED]
  leftSpeed  = constrain(leftSpeed,  0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ---------------------------------------------------------------------------
//  loop()
// ---------------------------------------------------------------------------
void loop() {
  int position = readLinePosition();

  // Error: 0 when centred (position == 2000)
  float error = position - 2000;

  // PID terms (with integral windup clamping)
  integral  += error;
  integral   = constrain(integral, -5000, 5000);
  float derivative = error - lastError;
  float correction = Kp * error / 1000.0
                   + Ki * integral / 1000.0
                   + Kd * derivative / 1000.0;

  lastError = error;

  // Apply correction to motors
  int leftSpeed  = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;

  setMotors(leftSpeed, rightSpeed);
}
