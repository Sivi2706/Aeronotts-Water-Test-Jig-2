#pragma once
#include <Arduino.h>

// ============================================================
//  Mahony AHRS filter
//  Ported from BMI160_Vector_main.cpp
//
//  Usage:
//    MahonyAHRS ahrs;
//    ahrs.begin(100.0f);          // sample rate Hz
//    ahrs.update(gx, gy, gz, ax, ay, az, dt);
//    float yaw   = ahrs.yawDeg();
//    float pitch = ahrs.pitchDeg();
//    float roll  = ahrs.rollDeg();
//    ahrs.getQuaternion(qw, qx, qy, qz);
// ============================================================

class MahonyAHRS {
public:
  MahonyAHRS() = default;

  // kp / ki defaults match the Python hand-test file
  void begin(float sampleHz, float kp = 2.4f, float ki = 0.1f);

  // All inputs: gyro in rad/s, accel in m/s²
  void update(float gx, float gy, float gz,
              float ax, float ay, float az,
              float dt_s);

  // Euler angles in degrees
  float yawDeg()   const;
  float pitchDeg() const;
  float rollDeg()  const;

  // Raw quaternion (w, x, y, z)
  void getQuaternion(float& qw, float& qx, float& qy, float& qz) const;

  // Accel magnitude used last update (m/s²)
  float accelMag() const { return _aMag; }

private:
  float _q0 = 1.0f, _q1 = 0.0f, _q2 = 0.0f, _q3 = 0.0f;
  float _ix = 0.0f, _iy = 0.0f, _iz = 0.0f;
  float _twoKp = 2.4f;
  float _twoKi = 0.1f;
  float _aMag  = 9.81f;

  static float invSqrt(float x);
};
