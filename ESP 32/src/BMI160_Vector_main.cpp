#include <Arduino.h>
#include <Wire.h>
#include "BMI160.h"

BMI160 imu(Wire);

static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;
static constexpr uint8_t BMI_ADDR = 0x69;

static constexpr float SAMPLE_HZ = 100.0f;
static constexpr float TWO_KP = 2.0f * 1.2f;
static constexpr float TWO_KI = 2.0f * 0.05f;

static float q0 = 1, q1 = 0, q2 = 0, q3 = 0; // quaternion
static float ix = 0, iy = 0, iz = 0;         // integral error
static uint32_t lastUs = 0;

static inline float invSqrt(float x) { return 1.0f / sqrtf(x); }

static void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float norm = ax*ax + ay*ay + az*az;
  if (norm < 1e-12f) return;
  norm = invSqrt(norm);
  ax *= norm; ay *= norm; az *= norm;

  float vx = 2.0f*(q1*q3 - q0*q2);
  float vy = 2.0f*(q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  float ex = (ay*vz - az*vy);
  float ey = (az*vx - ax*vz);
  float ez = (ax*vy - ay*vx);

  ix += TWO_KI * ex * dt;
  iy += TWO_KI * ey * dt;
  iz += TWO_KI * ez * dt;

  gx += TWO_KP * ex + ix;
  gy += TWO_KP * ey + iy;
  gz += TWO_KP * ez + iz;

  float halfDt = 0.5f * dt;
  float qa=q0, qb=q1, qc=q2, qd=q3;

  q0 += (-qb*gx - qc*gy - qd*gz) * halfDt;
  q1 += ( qa*gx + qc*gz - qd*gy) * halfDt;
  q2 += ( qa*gy - qb*gz + qd*gx) * halfDt;
  q3 += ( qa*gz + qb*gy - qc*gx) * halfDt;

  float qn = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= qn; q1 *= qn; q2 *= qn; q3 *= qn;
}

static void quatToYPR(float &yawDeg, float &pitchDeg, float &rollDeg) {
  float yaw   = atan2f(2.0f*q1*q2 + 2.0f*q0*q3, q0*q0 + q1*q1 - q2*q2 - q3*q3);
  float gx = 2.0f*(q1*q3 - q0*q2);
  float gy = 2.0f*(q0*q1 + q2*q3);
  float gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  float pitch = atanf(gx / sqrtf(gy*gy + gz*gz));
  float roll  = atanf(gy / sqrtf(gx*gx + gz*gz));

  yawDeg   = yaw   * 57.295779513f;
  pitchDeg = pitch * 57.295779513f;
  rollDeg  = roll  * 57.295779513f;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  if (imu.I2cInit(BMI_ADDR) != BMI160_OK) {
    Serial.println("BMI160 init failed");
    while (1) delay(1000);
  }

  lastUs = micros();

  Serial.println("DMPSTYLE,qw,qx,qy,qz,yaw,pitch,roll,ax,ay,az,amag");
}

void loop() {
  uint32_t now = micros();
  const uint32_t periodUs = (uint32_t)(1000000.0f / SAMPLE_HZ);
  if (now - lastUs < periodUs) return;
  float dt = (now - lastUs) / 1e6f;
  lastUs = now;

  int16_t d[6]; // gx,gy,gz, ax,ay,az
  if (imu.getAccelGyroData(d) != BMI160_OK) return;

  float ax = imu.accelLSB_to_mps2(d[3]);
  float ay = imu.accelLSB_to_mps2(d[4]);
  float az = imu.accelLSB_to_mps2(d[5]);

  float gx = imu.gyroLSB_to_dps(d[0]) * 0.01745329252f;
  float gy = imu.gyroLSB_to_dps(d[1]) * 0.01745329252f;
  float gz = imu.gyroLSB_to_dps(d[2]) * 0.01745329252f;

  mahonyUpdate(gx, gy, gz, ax, ay, az, dt);

  float yaw, pitch, roll;
  quatToYPR(yaw, pitch, roll);

  float amag = sqrtf(ax*ax + ay*ay + az*az);

  Serial.print("DMPSTYLE,");
  Serial.print(q0, 6); Serial.print(",");
  Serial.print(q1, 6); Serial.print(",");
  Serial.print(q2, 6); Serial.print(",");
  Serial.print(q3, 6); Serial.print(",");
  Serial.print(yaw, 2); Serial.print(",");
  Serial.print(pitch, 2); Serial.print(",");
  Serial.print(roll, 2); Serial.print(",");
  Serial.print(ax, 4); Serial.print(",");
  Serial.print(ay, 4); Serial.print(",");
  Serial.print(az, 4); Serial.print(",");
  Serial.println(amag, 4);
}

