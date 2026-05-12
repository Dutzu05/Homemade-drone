#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float rollTrim = 0.0f, pitchTrim = 0.0f;
float gyroX_offset = -0.07f;
float gyroY_offset = -0.02f;
float gyroZ_offset = 0.00f;
unsigned long lastTime;
unsigned long lastTelemetryTime;
const unsigned long TELEMETRY_INTERVAL_MS = 100;

const int POT_PIN = 0;

const int PWM_PINS[] = {3, 4, 6, 7};
const int NUM_PWM_PINS = 4;

const int PWM_CHANNELS[] = {0, 1, 2, 3};

const int MOTOR_REPORT_PINS[] = {4, 3, 7, 6};
const char* MOTOR_NAMES[] = {"Motor 1", "Motor 2", "Motor 3", "Motor 4"};

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;

const int ADC_RESOLUTION = 12;
const int ADC_MAX = 4095;

const float BATTERY_VOLTAGE = 3.6f;
const float MOTOR_RESISTANCE = 0.5f;
const float DEG_PER_RAD = 57.2958f;
const float STILL_GYRO_THRESHOLD = 0.03f;
const float STILL_ACCEL_TOLERANCE = 0.6f;

float estimateAverageVoltage(int dutyCycle) {
  return BATTERY_VOLTAGE * ((float)dutyCycle / (float)PWM_MAX);
}

float estimateCurrent(float voltage) {
  if (voltage < 0.1f) return 0.0f;
  return voltage / MOTOR_RESISTANCE;
}

void printMotorTelemetry(const char* motorName, int pin, int dutyCycle) {
  float vAvg = estimateAverageVoltage(dutyCycle);
  float iEst = estimateCurrent(vAvg);

  Serial.print(motorName);
  Serial.print(" | pin=");
  Serial.print(pin);
  Serial.print(" | pwm=");
  Serial.print(dutyCycle);
  Serial.print(" | v=");
  Serial.print(vAvg, 2);
  Serial.print("V | curr=");
  Serial.print(iEst, 2);
  Serial.print("A | rpm=N/A");
  Serial.println();
}

void updateImuTelemetry(float &accX, float &accY, float &accZ,
                        float &gyroX, float &gyroY, float &gyroZ) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0f;
  lastTime = currentTime;

  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

  gyroX = g.gyro.x - gyroX_offset;
  gyroY = g.gyro.y - gyroY_offset;
  gyroZ = g.gyro.z - gyroZ_offset;

  float accRoll = atan2(accY, accZ) * DEG_PER_RAD;
  float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * DEG_PER_RAD;
  float accelMagnitude = sqrt(accX * accX + accY * accY + accZ * accZ);
  bool isStill =
    fabs(gyroX) < STILL_GYRO_THRESHOLD &&
    fabs(gyroY) < STILL_GYRO_THRESHOLD &&
    fabs(gyroZ) < STILL_GYRO_THRESHOLD &&
    fabs(accelMagnitude - 9.81f) < STILL_ACCEL_TOLERANCE;

  float gyroWeight = isStill ? 0.90f : 0.96f;
  float accelWeight = 1.0f - gyroWeight;

  roll = gyroWeight * (roll + gyroX * DEG_PER_RAD * dt) + accelWeight * accRoll;
  pitch = gyroWeight * (pitch + gyroY * DEG_PER_RAD * dt) + accelWeight * accPitch;
  yaw = yaw + gyroZ * DEG_PER_RAD * dt;
}

void calibrateLevelTrim() {
  const int samples = 200;
  float accRollSum = 0.0f;
  float accPitchSum = 0.0f;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float accRoll = atan2(a.acceleration.y, a.acceleration.z) * 57.2958f;
    float accPitch = atan2(
      -a.acceleration.x,
      sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)
    ) * 57.2958f;

    accRollSum += accRoll;
    accPitchSum += accPitch;
    delay(5);
  }

  rollTrim = accRollSum / samples;
  pitchTrim = accPitchSum / samples;
  roll = 0.0f;
  pitch = 0.0f;
  yaw = 0.0f;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(8, 9);

  if (!mpu.begin()) {
    Serial.println("MPU6050 connection failed!");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("Keep drone still: calibrating level trim...");
    calibrateLevelTrim();
  }

  analogReadResolution(ADC_RESOLUTION);

  for (int i = 0; i < NUM_PWM_PINS; i++) {
    ledcSetup(PWM_CHANNELS[i], PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_PINS[i], PWM_CHANNELS[i]);
    ledcWrite(PWM_CHANNELS[i], 0);
  }

  lastTime = micros();
  lastTelemetryTime = millis();
}

void loop() {
  float accX = 0.0f;
  float accY = 0.0f;
  float accZ = 0.0f;
  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;
  updateImuTelemetry(accX, accY, accZ, gyroX, gyroY, gyroZ);

  int potValue = analogRead(POT_PIN);
  int dutyCycle = map(potValue, 0, ADC_MAX, 0, PWM_MAX);
  dutyCycle = constrain(dutyCycle, 0, PWM_MAX);

  for (int i = 0; i < NUM_PWM_PINS; i++) {
    ledcWrite(PWM_CHANNELS[i], dutyCycle);
  }

  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = millis();

    Serial.println("----- Telemetry Snapshot -----");

    Serial.print("POSITION | Ruliu: ");
    Serial.print(roll - rollTrim, 1);
    Serial.print(" deg | Tangaj: ");
    Serial.print(pitch - pitchTrim, 1);
    Serial.print(" deg | Girație (Yaw): ");
    Serial.print(yaw, 1);
    Serial.println(" deg");

    Serial.print("ACCEL    | X: ");
    Serial.print(accX, 2);
    Serial.print(" | Y: ");
    Serial.print(accY, 2);
    Serial.print(" | Z: ");
    Serial.print(accZ, 2);
    Serial.println(" m/s^2");

    Serial.print("GYRO     | X: ");
    Serial.print(gyroX, 3);
    Serial.print(" | Y: ");
    Serial.print(gyroY, 3);
    Serial.print(" | Z: ");
    Serial.print(gyroZ, 3);
    Serial.println(" rad/s");

    Serial.print("INPUT    | Pot: ");
    Serial.print(potValue);
    Serial.print(" | PWM_CMD: ");
    Serial.println(dutyCycle);

    for (int i = 0; i < NUM_PWM_PINS; i++) {
      printMotorTelemetry(MOTOR_NAMES[i], MOTOR_REPORT_PINS[i], dutyCycle);
    }

    Serial.println();
  }

  delay(10);
}
