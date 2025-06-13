#include "PDController.h"
#include <math.h> // For fabs() function

namespace controller {

  PDController::PDController(float Kp, float Kd, uint32_t dt_ms, float min_output, float max_output, bool debug)
      : BaseController(dt_ms, min_output, max_output, debug),
        Kp(Kp), Kd(Kd), prev_error(0.0f) {

    // Validate PD parameters and provide guidance for common mistakes
    if (Kp < 0.0f) {
      Serial.println(F("WARNING: PDController - Negative Kp can cause instability"));
    }
    if (Kd < 0.0f) {
      Serial.println(F("WARNING: PDController - Negative Kd reduces damping effect"));
    }
    if (Kp == 0.0f && Kd == 0.0f) {
      Serial.println(F("WARNING: PDController - Both gains are zero, no control action"));
    }

    // Check for unusual gain relationships that might indicate tuning issues
    if (Kd > Kp * 2.0f) {
      Serial.println(F("WARNING: PDController - Very high Kd relative to Kp may cause sluggish response"));
    }
    if (Kd > 0.0f && dt > 0.1f) { // dt > 100ms
      Serial.println(F("WARNING: PDController - Large sample time may cause derivative noise"));
    }

    if (debug_enabled) {
      Serial.print(F("PDController: Created with Kp="));
      Serial.print(Kp, 3);
      Serial.print(F(", Kd="));
      Serial.print(Kd, 3);
      Serial.print(F(", dt="));
      Serial.print(dt * 1000.0f);
      Serial.println(F("ms"));
    }
  }

  bool PDController::init() {
    // Call base class initialization first
    if (!BaseController::init()) {
      Serial.println(F("ERROR: PDController::init() - Base initialization failed"));
      return false;
    }

    // Validate PD-specific parameters
    if (Kp < 0.0f || Kd < 0.0f) {
      Serial.println(F("ERROR: PDController::init() - Gains cannot be negative"));
      return false;
    }

    // Ensure at least one gain is non-zero for meaningful control
    if (Kp == 0.0f && Kd == 0.0f) {
      Serial.println(F("ERROR: PDController::init() - At least one gain must be non-zero"));
      return false;
    }

    // Warn about derivative term with large sample times
    // Large dt makes derivative calculation less accurate and more noisy
    if (Kd > 0.0f && dt > 0.05f) { // Warning if dt > 50ms and using derivative
      Serial.print(F("WARNING: PDController::init() - Large sample time ("));
      Serial.print(dt * 1000.0f);
      Serial.println(F("ms) may cause derivative noise"));
    }

    // Initialize state variables for clean startup
    reset();

    debugLog(F("PDController initialized successfully"));
    return true;
  }

  void PDController::reset() {
    // Clear derivative calculation history
    // This prevents startup transients from old error values
    prev_error = 0.0f;
    output = 0.0f;

    debugLog(F("PDController state reset - derivative history cleared"));
  }

  float PDController::compute(float error) {
    // Implement the PD control algorithm
    // Output = Kp*error + Kd*(error - prev_error)/dt
    //
    // This algorithm provides both immediate response to errors (P term)
    // and predictive damping based on error trends (D term)

    // 1. PROPORTIONAL TERM: Immediate response to current error
    //    This term provides the primary driving force to correct errors
    //    Larger errors produce proportionally larger corrections
    float p_term = Kp * error;

    // 2. DERIVATIVE TERM: Predictive response based on error rate of change
    //    This term acts like a "brake" when approaching the setpoint
    //    It helps prevent overshoot and reduces oscillations
    //
    //    Key insight: We calculate derivative of ERROR, not just setpoint change
    //    This approach avoids "derivative kick" - a sudden spike in output
    //    when the setpoint changes rapidly (like in step inputs)
    float error_rate = (error - prev_error) / dt;
    float d_term = Kd * error_rate;

    // Store current error for next iteration's derivative calculation
    // This is critical - the derivative term depends on the history of errors
    prev_error = error;

    // 3. COMBINE PROPORTIONAL AND DERIVATIVE TERMS
    //    The P term provides the "muscle" - the main corrective force
    //    The D term provides the "intelligence" - it moderates the response
    //    based on how quickly things are changing
    float pd_output = p_term + d_term;

    // 4. APPLY OUTPUT LIMITS
    //    Essential for protecting actuators and ensuring system stability
    //    Without limits, the controller could command impossible actuator values
    output = applyLimits(pd_output, min_output, max_output);

    // Debug output shows how each term contributes to the final result
    // This is invaluable for understanding controller behavior during tuning
    if (debug_enabled) {
      Serial.print(F("PD: error="));
      Serial.print(error, 3);
      Serial.print(F(", error_rate="));
      Serial.print(error_rate, 3);
      Serial.print(F(", P="));
      Serial.print(p_term, 2);
      Serial.print(F(", D="));
      Serial.print(d_term, 2);
      Serial.print(F(", output="));
      Serial.println(output, 2);
    }

    return output;
  }

  void PDController::setKp(float Kp) {
    // Validate proportional gain
    if (Kp < 0.0f) {
      debugLog(F("WARNING: setKp() - Negative Kp can cause instability"));
    }

    this->Kp = Kp;

    if (debug_enabled) {
      Serial.print(F("Kp updated to "));
      Serial.println(Kp, 3);
    }
  }

  void PDController::setKd(float Kd) {
    // Validate derivative gain
    if (Kd < 0.0f) {
      debugLog(F("WARNING: setKd() - Negative Kd reduces damping effect"));
    }

    this->Kd = Kd;

    if (debug_enabled) {
      Serial.print(F("Kd updated to "));
      Serial.println(Kd, 3);
    }
  }

  void PDController::setGains(float Kp, float Kd) {
    // Validate gain combination
    if (Kp < 0.0f || Kd < 0.0f) {
      debugLog(F("WARNING: setGains() - Negative gains can cause instability"));
    }

    // Update both gains simultaneously
    // This is more efficient than individual updates and ensures consistency
    this->Kp = Kp;
    this->Kd = Kd;

    // Note: Unlike PI or PID controllers, we don't need to reset any
    // accumulated state when changing PD gains. The only state is prev_error,
    // which should maintain continuity for proper derivative calculation.

    if (debug_enabled) {
      Serial.print(F("PD gains updated: Kp="));
      Serial.print(Kp, 3);
      Serial.print(F(", Kd="));
      Serial.println(Kd, 3);
    }
  }

  float PDController::getKp() const {
    return Kp;
  }

  float PDController::getKd() const {
    return Kd;
  }

} // namespace controller