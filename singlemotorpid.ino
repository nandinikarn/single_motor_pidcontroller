/*
   SINGLE MOTOR TILT BALANCE - PID CONTROL
   ------------------------------------------------------------------
   ESP32 + MPU6050 + Motor A (ESC A) ONLY

   Single independent PID controller, driven directly by tilt angle.
   Includes safety cutoff, manual/auto serial commands, and full
   calibration.

   LIBRARIES NEEDED:
     1. "MPU6050" by Electronic Cats
     2. "ESP32Servo" by Kevin Harrington

   WIRING:
     ESP32 3.3V   -> MPU6050 VCC
     ESP32 GND    -> MPU6050 GND, ESC A GND (COMMON GROUND)
     ESP32 GPIO21 -> MPU6050 SDA
     ESP32 GPIO22 -> MPU6050 SCL
     ESP32 GPIO25 -> ESC A signal wire   (Motor A)
     (leave ESC A's red +5V wire disconnected from the ESP32)

   SAFETY: remove propeller while bench-testing PID gains. Only fit
   the prop once direction + response are verified and gains are sane.
*/

#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

MPU6050 mpu;
Servo escA;

const int SDA_PIN  = 21;
const int SCL_PIN  = 22;
const int ESC_A_PIN = 25;

const int ESC_MIN_US = 1000;
const int ESC_MAX_US = 2000;

int baseThrottleUs = 1075;

const int MOUNT_SIGN = -1;

const float MAX_SAFE_ANGLE = 45.0;
const int   PID_OUTPUT_LIMIT_US = 400;

const float ALPHA = 0.98;
const float GYRO_SCALE = 131.0;
const int CALIBRATION_SAMPLES = 500;

float rollOffset = 0, pitchOffset = 0;
float gyroOffsetX = 0, gyroOffsetY = 0;
float rollAngle = 0, pitchAngle = 0;

#define USE_PITCH_AXIS 1

unsigned long lastTime = 0;

bool manualMode = true;
int manualThrottleA = ESC_MIN_US;
String serialBuffer = "";

struct PID {
  float Kp, Ki, Kd;
  float integral;
  float prevError;
  float integralLimit;
};

PID pidA = { 3.0, 0.00, 0.0, 0, 0, 100 };

float computePID(PID &pid, float error, float dt) {
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -pid.integralLimit, pid.integralLimit);
  float derivative = (dt > 0) ? (error - pid.prevError) / dt : 0;
  pid.prevError = error;
  return pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
}

void printHelp() {
  Serial.println("---- SERIAL COMMANDS ----");
  Serial.println("T<us>   set base throttle, e.g. T1300");
  Serial.println("A<us>   manual override Motor A pulse, e.g. A1500 (enters MANUAL mode)");
  Serial.println("AUTO    return to PID control");
  Serial.println("P<val>  set Kp, e.g. P15.0");
  Serial.println("I<val>  set Ki, e.g. I0.02");
  Serial.println("D<val>  set Kd, e.g. D1.0");
  Serial.println("STOP    idle motor immediately");
  Serial.println("HELP    show this list");
  Serial.println("-------------------------");
}

void processSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  String upper = line;
  upper.toUpperCase();

  if (upper == "AUTO") {
    manualMode = false;
    pidA.integral = 0;
    Serial.println("Switched to AUTOMATIC (PID) mode.");
    return;
  }
  if (upper == "STOP" || upper == "S") {
    manualMode = true;
    manualThrottleA = ESC_MIN_US;
    Serial.println("STOP - motor idled, manual mode ON.");
    return;
  }
  if (upper == "HELP" || upper == "?") {
    printHelp();
    return;
  }

  char cmd = upper.charAt(0);
  float val = upper.substring(1).toFloat();

  switch (cmd) {
    case 'T':
      baseThrottleUs = constrain((int)val, ESC_MIN_US, ESC_MAX_US);
      Serial.print("Base throttle set to "); Serial.print(baseThrottleUs); Serial.println(" us");
      break;
    case 'A':
      manualMode = true;
      manualThrottleA = constrain((int)val, ESC_MIN_US, ESC_MAX_US);
      Serial.print("Manual mode ON. Motor A set to "); Serial.print(manualThrottleA); Serial.println(" us");
      break;
    case 'P':
      pidA.Kp = val;
      Serial.print("Kp set to "); Serial.println(val);
      break;
    case 'I':
      pidA.Ki = val;
      Serial.print("Ki set to "); Serial.println(val);
      break;
    case 'D':
      pidA.Kd = val;
      Serial.print("Kd set to "); Serial.println(val);
      break;
    default:
      Serial.println("Unknown command. Type HELP for command list.");
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

void calibrate() {
  Serial.println("Calibrating - keep beam LEVEL and STILL...");
  long sumAx = 0, sumAy = 0, sumAz = 0;
  long sumGx = 0, sumGy = 0;
  int16_t ax, ay, az, gx, gy, gz;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumAx += ax; sumAy += ay; sumAz += az;
    sumGx += gx; sumGy += gy;
    delay(2);
  }

  float avgAx = sumAx / (float)CALIBRATION_SAMPLES;
  float avgAy = sumAy / (float)CALIBRATION_SAMPLES;
  float avgAz = sumAz / (float)CALIBRATION_SAMPLES;

  rollOffset  = atan2(avgAx, avgAz) * 180.0 / PI;
  pitchOffset = atan2(avgAy, avgAz) * 180.0 / PI;
  gyroOffsetX = sumGx / (float)CALIBRATION_SAMPLES;
  gyroOffsetY = sumGy / (float)CALIBRATION_SAMPLES;

  Serial.println("Calibration done.");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection FAILED. Check wiring.");
    while (1) delay(1000);
  }
  Serial.println("MPU6050 connected.");

  calibrate();

  escA.attach(ESC_A_PIN, ESC_MIN_US, ESC_MAX_US);

  Serial.println("Arming ESC A - keep clear of prop...");
  escA.writeMicroseconds(ESC_MIN_US);
  delay(3000);
  Serial.println("Armed. Motor idle - type AUTO to start PID balancing, or T/A to set throttle manually.");
  printHelp();

  lastTime = millis();
}

unsigned long lastPrint = 0;

void loop() {
  readSerialCommands();

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accRoll  = atan2((float)ax, (float)az) * 180.0 / PI;
  float accPitch = atan2((float)ay, (float)az) * 180.0 / PI;

  float gyroRollRate  = (gx - gyroOffsetX) / GYRO_SCALE;
  float gyroPitchRate = (gy - gyroOffsetY) / GYRO_SCALE;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  rollAngle  = ALPHA * (rollAngle  + gyroRollRate  * dt) + (1.0 - ALPHA) * accRoll;
  pitchAngle = ALPHA * (pitchAngle + gyroPitchRate * dt) + (1.0 - ALPHA) * accPitch;

#if USE_PITCH_AXIS
  float angle = MOUNT_SIGN * (pitchAngle - pitchOffset);
#else
  float angle = MOUNT_SIGN * (rollAngle - rollOffset);
#endif

  bool safe = fabs(angle) <= MAX_SAFE_ANGLE;

  int motorAUs;
  float outA = 0;

  if (!safe) {
    motorAUs = ESC_MIN_US;
    pidA.integral = 0;
  } else if (manualMode) {
    motorAUs = manualThrottleA;
  } else {
    float errorA = angle;
    outA = computePID(pidA, errorA, dt);
    outA = constrain(outA, -PID_OUTPUT_LIMIT_US, PID_OUTPUT_LIMIT_US);
    motorAUs = constrain(baseThrottleUs + (int)outA, ESC_MIN_US, ESC_MAX_US);
  }

  escA.writeMicroseconds(motorAUs);

  if (now - lastPrint >= 200) {
    lastPrint = now;

    String tiltSide;
    if (angle > 1.0) tiltSide = "SIDE DOWN";
    else if (angle < -1.0) tiltSide = "SIDE UP";
    else tiltSide = "LEVEL";

    Serial.print(!safe ? "[!! SAFETY CUTOFF !!] " : (manualMode ? "[MANUAL] " : "[AUTO] "));
    Serial.print("Angle: "); Serial.print(angle, 2); Serial.print(" deg (");
    Serial.print(tiltSide); Serial.print(")  |  Motor A: "); Serial.print(motorAUs);
    Serial.print(" us ("); Serial.print(motorAUs - baseThrottleUs >= 0 ? "+" : "");
    Serial.print(motorAUs - baseThrottleUs); Serial.print(")");

    if (!manualMode) {
      Serial.print("  |  PID[P:"); Serial.print(pidA.Kp * angle, 1);
      Serial.print(" I:"); Serial.print(pidA.Ki * pidA.integral, 1);
      Serial.print(" D:"); Serial.print(outA - (pidA.Kp * angle + pidA.Ki * pidA.integral), 1);
      Serial.print("]");
    }
    Serial.println();
  }

  delay(10);
}