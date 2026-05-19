#include <Arduino.h>

// =============================================================
// ROS diffdrive_arduino-compatible ESP32 firmware
// Motor driver: FIT0441 PWM + DIR
// ESP32 Arduino Core 3.x compatible
// =============================================================

#define FIRMWARE_NAME "phoenix_fit0441_diffdrive"
#define FIRMWARE_VERSION "2026-05-16-v1"

// -----------------------
// Serial settings
// -----------------------
#define BAUD_RATE 57600 // must match ros2_control.xacro
#define CMD_TIMEOUT_MS 500

// -----------------------
// FIT0441 motor pins
// -----------------------

// Motor 1 / LEFT
#define LEFT_PWM_PIN 25
#define LEFT_DIR_PIN 26
#define LEFT_FG_PIN 34

// Motor 2 / RIGHT
#define RIGHT_PWM_PIN 14
#define RIGHT_DIR_PIN 27
#define RIGHT_FG_PIN 35

// -----------------------
// Encoder enable flags
// If a motor/FG wire is not connected yet, set that side to false.
// -----------------------
#define USE_LEFT_ENCODER true
#define USE_RIGHT_ENCODER true

// pulseIn variables removed — replaced with interrupt-based counting

// -----------------------
// Orientation correction
// -----------------------
#define INVERT_LEFT_MOTOR true
#define INVERT_RIGHT_MOTOR false
#define SWAP_LEFT_RIGHT false
#define INVERT_LEFT_ENCODER true
#define INVERT_RIGHT_ENCODER false
// Measured encoder CPR per wheel (from m 123 123 one-revolution test)
// Right (A) is the reference. Left (B) over-counts and is corrected to match.
#define LEFT_ENC_CPR 259
#define RIGHT_ENC_CPR 248

// -----------------------
// PWM settings for FIT0441
// FIT0441 behavior:
// PWM 0   = max speed
// PWM 255 = stop
// -----------------------
#define PWM_FREQ 25000
#define PWM_RES_BITS 8
#define MAX_PWM 255
#define MIN_PWM 0

// -----------------------
// Control loop
// diffdrive_arduino sends target encoder ticks per loop
// -----------------------
#define LOOP_RATE_HZ 30
#define LOOP_PERIOD_MS (1000 / LOOP_RATE_HZ)

// Conservative PID
#define KP 1.0f
#define KI 0.3f
#define KD 0.0f

#define MIN_MOVING_PWM 35

// =============================================================
// State
// =============================================================

volatile long enc_left = 0;
volatile long enc_right = 0;

volatile bool left_going_forward = true;
volatile bool right_going_forward = true;

long target_left = 0;
long target_right = 0;

long prev_enc_left = 0;
long prev_enc_right = 0;

float pid_int_left = 0.0f;
float pid_int_right = 0.0f;

float pid_prev_left = 0.0f;
float pid_prev_right = 0.0f;

unsigned long last_loop_ms = 0;
unsigned long last_cmd_ms = 0;

String serial_buf = "";
bool serial_last_was_cr = false;

// =============================================================
// FIT0441 motor control
// Internal command:
//  0   = stop
//  255 = max speed
//
// FIT0441 actual PWM:
//  255 = stop
//  0   = max speed
// =============================================================

int clampPWM(int value)
{
  if (value > MAX_PWM)
    return MAX_PWM;
  if (value < -MAX_PWM)
    return -MAX_PWM;
  return value;
}

int convertToFit0441PWM(int magnitude)
{
  magnitude = constrain(magnitude, 0, MAX_PWM);
  return 255 - magnitude;
}

void setFit0441Motor(
    int signedCommand,
    int pwmPin,
    int dirPin,
    bool invertMotor,
    volatile bool &isGoingForward)
{
  signedCommand = clampPWM(signedCommand);

  if (signedCommand == 0)
  {
    ledcWrite(pwmPin, 255); // FIT0441 stop
    return;
  }

  bool forward = signedCommand > 0;
  if (invertMotor)
  {
    forward = !forward;
  }

  isGoingForward = forward;

  digitalWrite(dirPin, forward ? HIGH : LOW);

  int magnitude = abs(signedCommand);

  if (magnitude > 0 && magnitude < MIN_MOVING_PWM)
  {
    magnitude = MIN_MOVING_PWM;
  }

  ledcWrite(pwmPin, convertToFit0441PWM(magnitude));
}

void stopMotors()
{
  target_left = 0;
  target_right = 0;

  pid_int_left = 0.0f;
  pid_int_right = 0.0f;
  pid_prev_left = 0.0f;
  pid_prev_right = 0.0f;

  setFit0441Motor(0, LEFT_PWM_PIN, LEFT_DIR_PIN, INVERT_LEFT_MOTOR, left_going_forward);
  setFit0441Motor(0, RIGHT_PWM_PIN, RIGHT_DIR_PIN, INVERT_RIGHT_MOTOR, right_going_forward);
}

// =============================================================
// Encoder ISRs — interrupt-based, zero blocking.
// Triggered on RISING edge of each FG pulse.
// Direction is inferred from the last commanded motor direction.
// GPIO 34 and 35 are input-only on ESP32; external 5k pull-ups required.
// =============================================================

void IRAM_ATTR leftFgISR()
{
  bool forward = INVERT_LEFT_ENCODER ? !left_going_forward : left_going_forward;
  enc_left += forward ? 1 : -1;
}

void IRAM_ATTR rightFgISR()
{
  bool forward = INVERT_RIGHT_ENCODER ? !right_going_forward : right_going_forward;
  enc_right += forward ? 1 : -1;
}

// =============================================================
// PID
// target = desired ticks per loop
// actual = measured ticks per loop
// =============================================================

float pidStep(float target, float actual, float &previousError, float &integral)
{
  if (target == 0)
  {
    previousError = 0;
    integral = 0;
    return 0;
  }

  float error = abs(target) - abs(actual);
  integral += error;

  if (integral > 300)
    integral = 300;
  if (integral < -300)
    integral = -300;

  float derivative = error - previousError;
  previousError = error;

  float output = KP * error + KI * integral + KD * derivative;

  output = constrain(output, 0, MAX_PWM);

  if (output > 0 && output < MIN_MOVING_PWM)
  {
    output = MIN_MOVING_PWM;
  }

  return output;
}

// =============================================================
// Serial protocol for diffdrive_arduino
//
// Pi -> ESP32:
//   e        -> return encoder counts: "<left> <right>"
//   m L R    -> set left/right target ticks per loop
//   bare CR  -> return "I ack"
// =============================================================

void processCommand(String cmd)
{
  cmd.trim();

  if (cmd == "e")
  {
    noInterrupts();
    long l = enc_left;
    long r = enc_right;
    interrupts();

    // Scale left encoder to RIGHT_ENC_CPR equivalent so ros2_control
    // sees consistent counts and odometry is correct for both wheels.
    long l_scaled = (long)roundf((float)l * (float)RIGHT_ENC_CPR / (float)LEFT_ENC_CPR);

    Serial.print(l_scaled);
    Serial.print(' ');
    Serial.print(r);
    Serial.print("\r\n");
    return;
  }

  if (cmd.startsWith("m "))
  {
    int split = cmd.indexOf(' ', 2);

    if (split > 0)
    {
      long l_cmd = cmd.substring(2, split).toInt();
      long r_cmd = cmd.substring(split + 1).toInt();

      if (SWAP_LEFT_RIGHT)
      {
        target_left = r_cmd;
        target_right = l_cmd;
      }
      else
      {
        target_left = l_cmd;
        target_right = r_cmd;
      }

      last_cmd_ms = millis();
    }

    Serial.print("OK\r\n");
    return;
  }

  if (cmd == "v")
  {
    Serial.print(FIRMWARE_NAME);
    Serial.print(" ");
    Serial.print(FIRMWARE_VERSION);
    Serial.print("\r\n");
  }

  // diffdrive_arduino ping
  if (cmd.length() == 0)
  {
    Serial.print("I ack\r\n");
    return;
  }
}

// =============================================================
// Setup
// =============================================================

void setup()
{
  Serial.begin(BAUD_RATE);

  // Direction pins
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  // Before attaching PWM, force PWM lines HIGH because FIT0441 HIGH/255 = stop.
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);

  digitalWrite(LEFT_PWM_PIN, HIGH);
  digitalWrite(RIGHT_PWM_PIN, HIGH);

  delay(300);

  // ESP32 Arduino Core 3.x PWM attach
  ledcAttach(LEFT_PWM_PIN, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(RIGHT_PWM_PIN, PWM_FREQ, PWM_RES_BITS);

  // Explicit stop after PWM attach
  ledcWrite(LEFT_PWM_PIN, 255);
  ledcWrite(RIGHT_PWM_PIN, 255);

  // Encoder pins — input only, no internal pull-up (external 5k to 3.3V required)
  pinMode(LEFT_FG_PIN, INPUT);
  pinMode(RIGHT_FG_PIN, INPUT);

  // Brief settling time so FG pins are stable before ISRs are armed
  delay(500);
  // Attach ISRs — counts pulses in background with zero blocking
  attachInterrupt(digitalPinToInterrupt(LEFT_FG_PIN), leftFgISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_FG_PIN), rightFgISR, RISING);

  stopMotors();

  last_loop_ms = millis();
  last_cmd_ms = millis();
}

// =============================================================
// Main loop
// =============================================================

void loop()
{
  // Read serial commands (no updateEncoders() — ISRs count pulses in background)
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\r')
    {
      processCommand(serial_buf);
      serial_buf = "";
      serial_last_was_cr = true;
    }
    else if (c == '\n')
    {
      // Skip if \r already processed this command (handles CRLF without double-firing).
      // Process if \r did NOT precede this (handles bare \n line endings).
      if (!serial_last_was_cr)
      {
        processCommand(serial_buf);
        serial_buf = "";
      }
      serial_last_was_cr = false;
    }
    else
    {
      serial_buf += c;
      serial_last_was_cr = false;
    }
  }

  unsigned long now = millis();

  // Safety timeout
  if (now - last_cmd_ms > CMD_TIMEOUT_MS)
  {
    stopMotors();
    last_cmd_ms = now;
  }

  // PID loop
  if (now - last_loop_ms >= LOOP_PERIOD_MS)
  {
    last_loop_ms = now;

    noInterrupts();
    long cur_left = enc_left;
    long cur_right = enc_right;
    interrupts();

    float delta_left = (float)(cur_left - prev_enc_left);
    float delta_right = (float)(cur_right - prev_enc_right);

    prev_enc_left = cur_left;
    prev_enc_right = cur_right;

    // Correct left delta to RIGHT_ENC_CPR scale before PID.
    // Without this, the PID sees left "going faster" and throttles it.
    float delta_left_corrected = delta_left * (float)RIGHT_ENC_CPR / (float)LEFT_ENC_CPR;

    float pwm_left = pidStep((float)target_left, delta_left_corrected, pid_prev_left, pid_int_left);
    float pwm_right = pidStep((float)target_right, delta_right, pid_prev_right, pid_int_right);

    int signed_left = target_left >= 0 ? (int)pwm_left : -(int)pwm_left;
    int signed_right = target_right >= 0 ? (int)pwm_right : -(int)pwm_right;

    if (target_left == 0)
      signed_left = 0;
    if (target_right == 0)
      signed_right = 0;

    setFit0441Motor(signed_left, LEFT_PWM_PIN, LEFT_DIR_PIN, INVERT_LEFT_MOTOR, left_going_forward);
    setFit0441Motor(signed_right, RIGHT_PWM_PIN, RIGHT_DIR_PIN, INVERT_RIGHT_MOTOR, right_going_forward);
  }
}