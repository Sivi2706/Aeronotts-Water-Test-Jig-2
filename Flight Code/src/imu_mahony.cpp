#include "imu_mahony.h"
#include <math.h>

float MahonyAHRS::invSqrt(float x) {
  return 1.0f / sqrtf(x);
}

void MahonyAHRS::begin(float sampleHz, float kp, float ki) {
  (void)sampleHz;
  _twoKp = 2.0f * kp;
  _twoKi = 2.0f * ki;
  _q0 = 1.0f; _q1 = 0.0f; _q2 = 0.0f; _q3 = 0.0f;
  _ix = 0.0f; _iy = 0.0f; _iz = 0.0f;
}

void MahonyAHRS::update(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt_s) {
  // Cache accel magnitude for FSM
  _aMag = sqrtf(ax*ax + ay*ay + az*az);

  float norm = _aMag;
  if (norm < 1e-12f) return;   // discard degenerate reads
  float inv = 1.0f / norm;
  ax *= inv; ay *= inv; az *= inv;

  // Estimated gravity direction from quaternion
  float vx = 2.0f*(_q1*_q3 - _q0*_q2);
  float vy = 2.0f*(_q0*_q1 + _q2*_q3);
  float vz = _q0*_q0 - _q1*_q1 - _q2*_q2 + _q3*_q3;

  // Cross-product error
  float ex = ay*vz - az*vy;
  float ey = az*vx - ax*vz;
  float ez = ax*vy - ay*vx;

  // Integral feedback
  _ix += _twoKi * ex * dt_s;
  _iy += _twoKi * ey * dt_s;
  _iz += _twoKi * ez * dt_s;

  // Proportional + integral correction on gyro
  gx += _twoKp * ex + _ix;
  gy += _twoKp * ey + _iy;
  gz += _twoKp * ez + _iz;

  // Integrate quaternion rate
  float hdt = 0.5f * dt_s;
  float qa = _q0, qb = _q1, qc = _q2, qd = _q3;

  _q0 += (-qb*gx - qc*gy - qd*gz) * hdt;
  _q1 += ( qa*gx + qc*gz - qd*gy) * hdt;
  _q2 += ( qa*gy - qb*gz + qd*gx) * hdt;
  _q3 += ( qa*gz + qb*gy - qc*gx) * hdt;

  // Normalise
  float qn = invSqrt(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);
  _q0 *= qn; _q1 *= qn; _q2 *= qn; _q3 *= qn;
}

void MahonyAHRS::getQuaternion(float& qw, float& qx, float& qy, float& qz) const {
  qw = _q0; qx = _q1; qy = _q2; qz = _q3;
}

float MahonyAHRS::yawDeg() const {
  float yaw = atan2f(2.0f*_q1*_q2 + 2.0f*_q0*_q3,
                     _q0*_q0 + _q1*_q1 - _q2*_q2 - _q3*_q3);
  return yaw * 57.295779513f;
}

float MahonyAHRS::pitchDeg() const {
  float gx = 2.0f*(_q1*_q3 - _q0*_q2);
  float gy = 2.0f*(_q0*_q1 + _q2*_q3);
  float gz = _q0*_q0 - _q1*_q1 - _q2*_q2 + _q3*_q3;
  float pitch = atanf(gx / sqrtf(gy*gy + gz*gz));
  return pitch * 57.295779513f;
}

float MahonyAHRS::rollDeg() const {
  float gx = 2.0f*(_q1*_q3 - _q0*_q2);
  float gy = 2.0f*(_q0*_q1 + _q2*_q3);
  float gz = _q0*_q0 - _q1*_q1 - _q2*_q2 + _q3*_q3;
  float roll = atanf(gy / sqrtf(gx*gx + gz*gz));
  return roll * 57.295779513f;
}
