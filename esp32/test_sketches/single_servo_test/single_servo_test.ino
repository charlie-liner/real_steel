/*
 * single_servo_test.ino
 *
 * Minimal test: drives ONE servo via PCA9685 over I2C.
 *
 * Wiring:
 *   ESP32 GPIO 21 (SDA) -> PCA9685 SDA
 *   ESP32 GPIO 22 (SCL) -> PCA9685 SCL
 *   PCA9685 screw terminal -> External 5-6V PSU
 *   Servo on PCA9685 Channel 0
 *
 * Serial commands (115200 baud):
 *   A:<angle>    -> set servo angle (0-180 degrees)
 *   H            -> home (go to 90)
 *   Q            -> report current angle
 *   E:<0|1>      -> enable/disable
 *
 * On boot: sweeps to verify servo works, then listens for commands.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== CONFIG ====================

#define SERIAL_BAUD 115200
#define SERVO_CH    0       // PCA9685 channel

#define SERVO_MIN   102     // ~0.5ms pulse at 50Hz
#define SERVO_MAX   512     // ~2.5ms pulse at 50Hz

#define ANGLE_MIN   0.0
#define ANGLE_MAX   180.0

// ==================== GLOBALS ====================

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

float currentAngle = 90.0;
bool enabled = true;
String inputBuffer = "";

// ==================== HELPERS ====================

void setServoAngle(float angle) {
  if (!enabled) return;

  if (angle < ANGLE_MIN) angle = ANGLE_MIN;
  if (angle > ANGLE_MAX) angle = ANGLE_MAX;

  currentAngle = angle;

  int pulse = map((int)angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pca.setPWM(SERVO_CH, 0, pulse);
}

// ==================== STARTUP SWEEP ====================

void sweepTest() {
  Serial.println("Sweep test: 0 -> 180 -> 90");

  setServoAngle(0);
  delay(500);

  for (int a = 0; a <= 180; a += 5) {
    setServoAngle(a);
    delay(20);
  }
  delay(300);

  for (int a = 180; a >= 90; a -= 5) {
    setServoAngle(a);
    delay(20);
  }
  delay(200);

  Serial.println("Sweep done");
}

// ==================== COMMAND HANDLER ====================

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char type = cmd.charAt(0);

  switch (type) {
    case 'A': {
      if (cmd.length() < 3 || cmd.charAt(1) != ':') {
        Serial.println("ERR:1:Bad format");
        return;
      }
      float angle = cmd.substring(2).toFloat();
      if (angle < ANGLE_MIN || angle > ANGLE_MAX) {
        Serial.println("ERR:2:Angle out of range");
        return;
      }
      setServoAngle(angle);
      Serial.print("OK angle=");
      Serial.println(angle, 1);
      break;
    }

    case 'H':
      setServoAngle(90.0);
      Serial.println("OK");
      break;

    case 'Q':
      Serial.print("P:");
      Serial.println(currentAngle, 1);
      break;

    case 'E':
      if (cmd.length() >= 3) {
        enabled = (cmd.charAt(2) == '1');
        if (!enabled) {
          pca.setPWM(SERVO_CH, 0, 0);  // Stop PWM
        }
      }
      Serial.println("OK");
      break;

    default:
      Serial.println("ERR:1:Unknown command");
  }
}

// ==================== SETUP & LOOP ====================

void setup() {
  Serial.begin(SERIAL_BAUD);

  Wire.begin(21, 22);
  pca.begin();
  pca.setPWMFreq(50);
  delay(100);

  Serial.println("PCA9685 initialized");

  sweepTest();

  Serial.println("READY");
  Serial.println("Commands: A:<angle> | H | Q | E:<0|1>");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}
