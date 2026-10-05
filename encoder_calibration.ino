/*
  Arduino Nano R4 - Bluetooth Encoder Calibration for Forward Motion
  ---------------------------------------------------------------
  Purpose:
    - Receive a target tick count from a Bluetooth terminal app
    - Spin both wheels forward
    - Count encoder ticks using interrupts
    - Stop exactly when target is reached
    - Keep left/right wheel ticks reasonably balanced with basic correction
    - Send status information back over Bluetooth
    - Support optional commands:
        S = stop immediately
        R = reset encoder counters
        ? = display current encoder counts

  Hardware:
    - Arduino Nano R4
    - HC-05 Bluetooth module connected to UART (D0 RX, D1 TX)
    - 2 DC motors with wheel encoders
    - Motor driver control pins:
        Motor 1:
          IN1 = D2
          IN2 = D3
        Motor 2:
          IN1 = D12
          IN2 = D13
    - Encoder pins:
        ENC_L_A = D6  (left encoder A - interrupt capable)
        ENC_L_B = D7  (left encoder B)
        ENC_R_A = D8  (right encoder A - interrupt capable)
        ENC_R_B = D9  (right encoder B)

  Bluetooth:
    - HC-05 TX -> Arduino D0 (RX)
    - HC-05 RX -> Arduino D1 (TX)
    - Using hardware UART Serial1 (not SoftwareSerial)
    - Use a voltage divider or proper logic-level adaptation if needed
      because HC-05 RX may not tolerate 5V directly depending on module variant.

  Notes:
    - This is calibration code only.
    - No maze-solving logic, no line-following, no turns, no PID.
    - Uses encoder feedback instead of delays.
    - No delay() in the movement loop.

  Serial:
    - USB Serial Monitor for debugging (Serial @ 115200)
    - Bluetooth serial for commands/status (Serial1 @ 9600)
*/

#include <Arduino.h>

// ------------------------------------------------------------
// Pin definitions
// ------------------------------------------------------------

// Motor driver pins
const uint8_t MOTOR1_IN1 = 2;
const uint8_t MOTOR1_IN2 = 3;
const uint8_t MOTOR2_IN1 = 12;
const uint8_t MOTOR2_IN2 = 13;

// Encoder pins - interrupt-capable, not D2/D3/D12/D13
const uint8_t ENC_L_A = 6;   // left encoder A (interrupt capable)
const uint8_t ENC_L_B = 7;   // left encoder B
const uint8_t ENC_R_A = 8;   // right encoder A (interrupt capable)
const uint8_t ENC_R_B = 9;   // right encoder B

// HC-05 Bluetooth uses hardware Serial1 on Nano R4
// D0 = RX (HC-05 TX connects here)
// D1 = TX (HC-05 RX connects here)
// No need to define pins; Serial1 is already mapped

// ------------------------------------------------------------
// Motor control helpers
// ------------------------------------------------------------

enum MotorDirection {
  MOTOR_STOP = 0,
  MOTOR_FORWARD = 1,
  MOTOR_REVERSE = 2
};

void setMotorState(uint8_t in1Pin, uint8_t in2Pin, MotorDirection dir) {
  switch (dir) {
    case MOTOR_STOP:
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
      break;

    case MOTOR_FORWARD:
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
      break;

    case MOTOR_REVERSE:
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
      break;
  }
}

// ------------------------------------------------------------
// Encoder variables
// ------------------------------------------------------------

// Use volatile because updated inside ISR
volatile long leftTicks = 0;
volatile long rightTicks = 0;

volatile long leftTicksRaw = 0;
volatile long rightTicksRaw = 0;

// Motor speed control for balancing
// These are PWM values.
const uint8_t MOTOR_BASE_SPEED = 180;  // 0..255
const uint8_t MOTOR_BALANCE_STEP = 10;  // small correction

uint8_t leftMotorSpeed = MOTOR_BASE_SPEED;
uint8_t rightMotorSpeed = MOTOR_BASE_SPEED;

// ------------------------------------------------------------
// Movement state
// ------------------------------------------------------------

bool moving = false;
bool targetActive = false;
long targetTicks = 0;

// Stop watchdog / timeout protection
unsigned long lastPulseTime = 0;
const unsigned long ENCODER_TIMEOUT_MS = 250;   // if no pulse in 250 ms -> stop
const unsigned long TARGET_CHECK_INTERVAL = 20;  // ms, periodic task check

// For calibration or debugging status
bool commandReceived = false;

// ------------------------------------------------------------
// Interrupt service routines for encoders
// ------------------------------------------------------------

// Left encoder pulse ISR
void leftEncoderISR() {
  leftTicksRaw++;
  leftTicks++;
  lastPulseTime = millis();
}

// Right encoder pulse ISR
void rightEncoderISR() {
  rightTicksRaw++;
  rightTicks++;
  lastPulseTime = millis();
}

// ------------------------------------------------------------
// Helper functions for movement and control
// ------------------------------------------------------------

void stopMotors() {
  setMotorState(MOTOR1_IN1, MOTOR1_IN2, MOTOR_STOP);
  setMotorState(MOTOR2_IN1, MOTOR2_IN2, MOTOR_STOP);
  moving = false;
  targetActive = false;
  leftMotorSpeed = MOTOR_BASE_SPEED;
  rightMotorSpeed = MOTOR_BASE_SPEED;
  Serial.println("Motors stopped.");
}

void startForward() {
  moving = true;
  targetActive = true;

  // If using driver with INx pins only and no ENABLE PWM pins
  // simply set digital states.
  setMotorState(MOTOR1_IN1, MOTOR1_IN2, MOTOR_FORWARD);
  setMotorState(MOTOR2_IN1, MOTOR2_IN2, MOTOR_FORWARD);

  // If your motor driver supports PWM on enable pins, use:
  // analogWrite(MOTOR1_EN, leftMotorSpeed);
  // analogWrite(MOTOR2_EN, rightMotorSpeed);
}

void resetEncoders() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  leftTicksRaw = 0;
  rightTicksRaw = 0;
  interrupts();
  Serial.println("Encoder counters reset.");
}

void applyBalanceCorrection() {
  // Basic closed-loop correction:
  // If left wheel is ahead, reduce left speed slightly or increase right speed.
  // If right wheel is ahead, reduce right speed slightly or increase left speed.
  // This is a very simple correction, not a full PID.

  // Use raw counts or current tick difference.
  long diff = leftTicks - rightTicks;

  if (diff > 0) {
    // left is ahead; reduce left slightly, or increase right
    leftMotorSpeed = max(120, MOTOR_BASE_SPEED - MOTOR_BALANCE_STEP);
    rightMotorSpeed = min(255, MOTOR_BASE_SPEED + MOTOR_BALANCE_STEP);
  } else if (diff < 0) {
    // right is ahead; reduce right slightly, or increase left
    leftMotorSpeed = min(255, MOTOR_BASE_SPEED + MOTOR_BALANCE_STEP);
    rightMotorSpeed = max(120, MOTOR_BASE_SPEED - MOTOR_BALANCE_STEP);
  } else {
    leftMotorSpeed = MOTOR_BASE_SPEED;
    rightMotorSpeed = MOTOR_BASE_SPEED;
  }

  // If your motor driver has PWM enable pins, drive them here.
  // Example:
  // analogWrite(MOTOR1_EN, leftMotorSpeed);
  // analogWrite(MOTOR2_EN, rightMotorSpeed);

  // For digital-only IN1/IN2 driver, we cannot change speed dynamically here.
  // If your driver uses PWM enable pins, uncomment the analogWrite lines.
}

// ------------------------------------------------------------
// Bluetooth command parsing
// ------------------------------------------------------------

void sendBluetoothStatus(const String &msg) {
  Serial1.print(msg);
  Serial1.flush();
}

void sendBluetoothStatusLine(const String &msg) {
  Serial1.println(msg);
  Serial1.flush();
}

bool isWhitespace(char c) {
  return (c == ' ' || c == '\n' || c == '\r' || c == '\t');
}

String readBluetoothCommand() {
  String cmd = "";
  while (Serial1.available()) {
    char c = Serial1.read();
    if (isWhitespace(c)) {
      // ignore spaces/newlines
      if (cmd.length() > 0) {
        // allow trailing whitespace after numeric value
      }
      continue;
    }
    cmd += c;
  }
  return cmd;
}

bool parseCommandAndExecute() {
  String cmd = readBluetoothCommand();
  if (cmd.length() == 0) return false;

  // Optional commands
  if (cmd == "S" || cmd == "s") {
    stopMotors();
    sendBluetoothStatusLine("STOPPED");
    sendBluetoothStatusLine("LEFT TICKS: " + String(leftTicks));
    sendBluetoothStatusLine("RIGHT TICKS: " + String(rightTicks));
    Serial.println("STOP command received.");
    Serial.print("Current left ticks: ");
    Serial.println(leftTicks);
    Serial.print("Current right ticks: ");
    Serial.println(rightTicks);
    return true;
  }

  if (cmd == "R" || cmd == "r") {
    resetEncoders();
    sendBluetoothStatusLine("ENCODERS RESET");
    return true;
  }

  if (cmd == "?") {
    sendBluetoothStatusLine("LEFT TICKS: " + String(leftTicks));
    sendBluetoothStatusLine("RIGHT TICKS: " + String(rightTicks));
    Serial.print("Query: LEFT TICKS = ");
    Serial.print(leftTicks);
    Serial.print(" | RIGHT TICKS = ");
    Serial.println(rightTicks);
    return true;
  }

  // Numeric target
  // Accept "650", "650\n", with spaces/newlines ignored
  // Reject invalid chars
  bool validNumber = true;
  for (int i = 0; i < cmd.length(); i++) {
    if (cmd[i] < '0' || cmd[i] > '9') {
      validNumber = false;
      break;
    }
  }

  if (!validNumber) {
    Serial.print("Invalid command received: ");
    Serial.println(cmd);
    sendBluetoothStatusLine("INVALID COMMAND");
    return true;
  }

  long newTarget = cmd.toInt();
  if (newTarget <= 0) {
    Serial.println("Target must be > 0");
    sendBluetoothStatusLine("TARGET MUST BE GREATER THAN 0");
    return true;
  }

  targetTicks = newTarget;

  // Reset counters before moving
  resetEncoders();

  Serial.print("Bluetooth command received: ");
  Serial.println(targetTicks);
  Serial.print("Target ticks: ");
  Serial.println(targetTicks);

  sendBluetoothStatusLine("TARGET: " + String(targetTicks));
  sendBluetoothStatusLine("STARTING FORWARD...");
  commandReceived = true;

  startForward();

  return true;
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------

void setup() {
  // USB Serial Monitor
  Serial.begin(115200);
  while (!Serial) {
    // wait for serial monitor (optional)
  }

  // Bluetooth serial on hardware UART (Serial1)
  // D0 = RX, D1 = TX
  Serial1.begin(9600);

  // Motor control pins
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  // Encoder pins
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  // Attach interrupts to encoder A pins
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, RISING);

  // Initial stop state
  stopMotors();

  Serial.println("========================================");
  Serial.println("System ready.");
  Serial.println("Arduino Nano R4 - Encoder Calibration");
  Serial.println("========================================");
  Serial.println("Use Bluetooth terminal to send target ticks like: 100, 235, 500, 650");
  Serial.println("Optional commands:");
  Serial.println("  S -> Stop immediately");
  Serial.println("  R -> Reset encoder counters");
  Serial.println("  ? -> Display current encoder counts");
  Serial.println("========================================");
  
  sendBluetoothStatusLine("READY");
}

void printDebugStatus() {
  Serial.print("Left ticks: ");
  Serial.print(leftTicks);
  Serial.print(" | Right ticks: ");
  Serial.println(rightTicks);
}

void handleTargetReached() {
  stopMotors();

  Serial.println("Target reached.");
  Serial.print("Left ticks: ");
  Serial.println(leftTicks);
  Serial.print("Right ticks: ");
  Serial.println(rightTicks);
  Serial.println("Motors stopped.");

  sendBluetoothStatusLine("TARGET: " + String(targetTicks));
  sendBluetoothStatusLine("LEFT TICKS: " + String(leftTicks));
  sendBluetoothStatusLine("RIGHT TICKS: " + String(rightTicks));
  sendBluetoothStatusLine("STATUS: COMPLETED");

  targetTicks = 0;
  commandReceived = false;
}

// ------------------------------------------------------------
// Main loop
// ------------------------------------------------------------

void loop() {
  // 1) Read incoming Bluetooth data
  parseCommandAndExecute();

  // 2) If a command is active, monitor encoder progress
  if (moving && targetActive) {
    // Simple encoder-based stop check
    long maxTicks = max(leftTicks, rightTicks);

    if (maxTicks >= targetTicks) {
      handleTargetReached();
    } else {
      // Apply simple balancing correction
      applyBalanceCorrection();

      // Safety timeout:
      // If no pulse has arrived for a while, stop to prevent indefinite movement.
      if ((millis() - lastPulseTime) > ENCODER_TIMEOUT_MS) {
        Serial.println("Encoder timeout: no pulses detected for too long.");
        Serial.println("Emergency stop.");
        stopMotors();
        sendBluetoothStatusLine("ERROR: ENCODER TIMEOUT");
        sendBluetoothStatusLine("LEFT TICKS: " + String(leftTicks));
        sendBluetoothStatusLine("RIGHT TICKS: " + String(rightTicks));
        targetTicks = 0;
      }

      // Debug print every ~100 ms
      static unsigned long lastDebugMillis = 0;
      if (millis() - lastDebugMillis > 100) {
        printDebugStatus();
        lastDebugMillis = millis();
      }
    }
  }

  // Small non-blocking delay is okay here if needed, but not necessary.
  // Avoid delay() in the main movement logic to keep Bluetooth responsive.
  delay(10);
}
