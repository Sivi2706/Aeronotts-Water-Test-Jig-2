#pragma once
#include <Arduino.h>

// ============================================================
//  FLIGHT STATE MACHINE
//  Ported from Hand-Test_Treshold.py
//
//  Detection method: delta of raw accel magnitude from baseline.
//  No barometer required — works on IMU alone.
// ============================================================

// ---- Thresholds tuned for water rocket ----------------------
//
// Water rockets produce a sharp thrust spike (10-30+ m/s²) on launch
// then go ballistic — accel drops close to 0 g (freefall) at apogee.
// Baseline on the pad ≈ 9.81 m/s² (gravity).
//
// LAUNCH  : delta > 3.5 m/s² above baseline  (ignores wind/handling)
// APOGEE  : delta > 8.0 m/s² BELOW baseline  (near-freefall, ~1.8 m/s²)
//           held for 0.25 s to reject brief zero-g bumps
//
// Auto-reset is DISABLED — once APOGEE is latched it stays latched.
// Power-cycle or hardware reset to re-arm for the next flight.

static constexpr float FSM_DELTA_LAUNCH_THRESH = 3.5f;   // m/s², above baseline -> LAUNCH
static constexpr float FSM_DELTA_APOGEE_THRESH = 8.0f;   // m/s², below baseline (freefall) -> apogee candidate
static constexpr float FSM_APOGEE_DWELL_S      = 0.25f;  // s, must sustain near-freefall this long

// Reset constants kept for reference but auto-reset is disabled (see flight_fsm.cpp)
static constexpr float FSM_RESET_DELTA_THRESH  = 0.35f;  // unused — no auto-reset
static constexpr float FSM_RESET_DWELL_S       = 0.60f;  // unused — no auto-reset

// ---- Calibration ------------------------------------------
static constexpr float FSM_INIT_DURATION_S     = 3.0f;   // s, startup averaging window

// ---- States -----------------------------------------------
enum FlightState : uint8_t {
  STATE_INIT      = 0,   // collecting baseline average
  STATE_IDLE      = 1,   // on pad, waiting for launch
  STATE_IN_FLIGHT = 2,   // launched
  STATE_APOGEE    = 3,   // apogee detected — deploy payload / servo
};

static inline const char* flightStateName(FlightState s) {
  switch (s) {
    case STATE_INIT:      return "INIT";
    case STATE_IDLE:      return "IDLE";
    case STATE_IN_FLIGHT: return "IN_FLIGHT";
    case STATE_APOGEE:    return "APOGEE";
    default:              return "UNKNOWN";
  }
}

// ============================================================
//  FlightFSM class
// ============================================================
class FlightFSM {
public:
  FlightFSM() = default;

  // Call once before the main loop. Resets all state.
  void reset();

  // Call every sensor tick with:
  //   rawMag_ms2 — current raw |accel| in m/s²
  //   dt_s       — elapsed seconds since last call
  // Returns true if the state changed this tick.
  bool update(float rawMag_ms2, float dt_s);

  // ---- Accessors ----------------------------------------
  FlightState state()       const { return _state; }
  float       baseline()    const { return _baselineMag; }
  float       deltaMag()    const { return _deltaMag; }
  bool        isCalibrated()const { return _calibrated; }

  // Progress 0.0 – 1.0 during INIT phase
  float       initProgress()const { return _calibrated ? 1.0f : (_initElapsed / FSM_INIT_DURATION_S); }

private:
  FlightState _state        = STATE_INIT;
  bool        _calibrated   = false;

  // Calibration accumulators
  float       _initSum      = 0.0f;
  uint32_t    _initCount    = 0;
  float       _initElapsed  = 0.0f;

  float       _baselineMag  = 9.81f;
  float       _deltaMag     = 0.0f;

  float       _apogeeTimer  = 0.0f;
  float       _resetTimer   = 0.0f;
};