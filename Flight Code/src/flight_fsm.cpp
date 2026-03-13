#include "flight_fsm.h"

void FlightFSM::reset() {
  _state       = STATE_INIT;
  _calibrated  = false;
  _initSum     = 0.0f;
  _initCount   = 0;
  _initElapsed = 0.0f;
  _baselineMag = 9.81f;
  _deltaMag    = 0.0f;
  _apogeeTimer = 0.0f;
  _resetTimer  = 0.0f;
}

bool FlightFSM::update(float rawMag_ms2, float dt_s) {
  const FlightState prevState = _state;

  // ---- Phase 1: collect baseline --------------------------------
  if (!_calibrated) {
    _initSum     += rawMag_ms2;
    _initCount   += 1;
    _initElapsed += dt_s;

    if (_initElapsed >= FSM_INIT_DURATION_S) {
      _baselineMag = (_initCount > 0) ? (_initSum / (float)_initCount) : 9.81f;
      _calibrated  = true;
      _state       = STATE_IDLE;

      Serial.print("[FSM] Calibrated. Baseline |a| = ");
      Serial.print(_baselineMag, 3);
      Serial.println(" m/s²");
    }
    return false;  // no meaningful state change during init
  }

  // ---- Phase 2: state machine -----------------------------------
  _deltaMag = fabsf(rawMag_ms2 - _baselineMag);

  switch (_state) {

    case STATE_IDLE:
      if (_deltaMag >= FSM_DELTA_LAUNCH_THRESH) {
        _state       = STATE_IN_FLIGHT;
        _apogeeTimer = 0.0f;
        _resetTimer  = 0.0f;
        Serial.println("[FSM] LAUNCH detected -> IN_FLIGHT");
      }
      break;

    case STATE_IN_FLIGHT:
      // At apogee the rocket is in near-freefall: |accel| drops well below
      // the 9.81 m/s² baseline, so delta becomes LARGE (>= threshold).
      if (_deltaMag >= FSM_DELTA_APOGEE_THRESH) {
        _apogeeTimer += dt_s;
        if (_apogeeTimer >= FSM_APOGEE_DWELL_S) {
          _state      = STATE_APOGEE;
          _resetTimer = 0.0f;
          Serial.println("[FSM] APOGEE detected -> APOGEE (latched)");
        }
      } else {
        _apogeeTimer = 0.0f;
      }
      break;

    case STATE_APOGEE:
      // Permanently latched — no auto-reset.
      // Power-cycle to re-arm for next flight.
      break;

    default:
      break;
  }

  return (_state != prevState);
}