#include "PController.h"

namespace controller {

  PController::PController(float Kp, uint32_t dt_ms, float min_output, float max_output, bool debug)
      : BaseController(dt_ms, min_output, max_output, debug), Kp(Kp) {

    // Validate proportional gain to prevent common mistakes
    if (Kp < 0.0f) {
      Serial.println(F("WARNING: PController - Negative Kp can cause instability"));
    }
    if (Kp == 0.0f) {
      Serial.println(F("WARNING: PController - Zero Kp means no control action"));
    }

    if (debug_enabled) {
      Serial.print(F("PController: Created with Kp="));
      Serial.print(Kp);
      Serial.print(F(", dt="));
      Serial.print(dt * 1000.0f);
      Serial.println(F("ms"));
    }
  }

  bool PController::init() {
    // Call base class initialization first
    if (!BaseController::init()) {
      Serial.println(F("ERROR: PController::init() - Base initialization failed"));
      return false;
    }

    // Validate P controller specific parameters
    if (Kp < 0.0f) {
      Serial.println(F("ERROR: PController::init() - Kp cannot be negative"));
      return false;
    }

    // Reset controller state to ensure clean start
    reset();

    debugLog(F("PController initialized successfully"));
    return true;
  }

  void PController::reset() {
    // P controller has no internal state to reset, just clear output
    output = 0.0f;
    debugLog(F("PController state reset"));
  }

  float PController::compute(float error) {
    // Implement the core P control algorithm: Output = Kp × Error
    // This is the fundamental equation of proportional control

    // Calculate proportional term
    float p_term = Kp * error;

    // Apply output limits to prevent actuator damage
    // This is critical in embedded systems to protect hardware
    output = applyLimits(p_term, min_output, max_output);

    // Debug output shows the control action for tuning purposes
    if (debug_enabled) {
      Serial.print(F("P: error="));
      Serial.print(error, 3); // 3 decimal places for precision
      Serial.print(F(", P_term="));
      Serial.print(p_term, 3);
      Serial.print(F(", output="));
      Serial.println(output, 3);
    }

    return output;
  }

  void PController::setKp(float Kp) {
    // Validate new gain before setting
    if (Kp < 0.0f) {
      debugLog(F("WARNING: setKp() - Negative Kp can cause instability"));
    }

    this->Kp = Kp;

    if (debug_enabled) {
      Serial.print(F("Kp updated to "));
      Serial.println(Kp, 3);
    }
  }

  float PController::getKp() const {
    return Kp;
  }

} // namespace controller