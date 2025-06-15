#include "PIController.h"
#include <math.h> // For fabs() and min() functions

namespace controller {

  PIController::PIController(float Kp, float Ki, uint32_t dt_ms, float min_output, float max_output, bool debug)
      : BaseController(dt_ms, min_output, max_output, debug),
        Kp(Kp), Ki(Ki), integral(0.0f),
        anti_windup(fabs(max_output)) { // Default anti-windup = max output magnitude

    // Validate PI parameters and provide educational feedback about common mistakes
    if (Kp < 0.0f) {
      Serial.println(F("WARNING: PIController - Negative Kp can cause instability"));
    }
    if (Ki < 0.0f) {
      Serial.println(F("WARNING: PIController - Negative Ki can cause instability"));
    }
    if (Kp == 0.0f && Ki == 0.0f) {
      Serial.println(F("WARNING: PIController - Both gains are zero, no control action"));
    }

    // Educational checks for common tuning mistakes
    if (Ki > Kp) {
      Serial.println(F("INFO: PIController - Ki > Kp is unusual, may cause aggressive integral action"));
    }
    if (Ki > 0.0f && dt > 0.1f) { // dt > 100ms
      Serial.println(F("WARNING: PIController - Large sample time reduces integral accuracy"));
    }

    // Explain anti-windup setting to help users understand this critical parameter
    if (debug_enabled) {
      Serial.print(F("PIController: Created with Kp="));
      Serial.print(Kp, 3);
      Serial.print(F(", Ki="));
      Serial.print(Ki, 3);
      Serial.print(F(", dt="));
      Serial.print(dt * 1000.0f);
      Serial.print(F("ms, anti-windup="));
      Serial.println(anti_windup, 1);
    }
  }

  bool PIController::init() {
    // Call base class initialization first
    if (!BaseController::init()) {
      Serial.println(F("ERROR: PIController::init() - Base initialization failed"));
      return false;
    }

    // Validate PI-specific parameters
    if (Kp < 0.0f || Ki < 0.0f) {
      Serial.println(F("ERROR: PIController::init() - Gains cannot be negative"));
      return false;
    }

    // Ensure at least one gain is meaningful
    if (Kp == 0.0f && Ki == 0.0f) {
      Serial.println(F("ERROR: PIController::init() - At least one gain must be non-zero"));
      return false;
    }

    // Validate anti-windup limit makes sense
    if (anti_windup <= 0.0f) {
      Serial.println(F("ERROR: PIController::init() - Anti-windup limit must be positive"));
      return false;
    }

    // Educational warning about integral control with large sample times
    // Large dt makes integral accumulation less precise and can cause instability
    if (Ki > 0.0f && dt > 0.05f) { // Warning if dt > 50ms and using integral
      Serial.print(F("WARNING: PIController::init() - Large sample time ("));
      Serial.print(dt * 1000.0f);
      Serial.println(F("ms) may reduce integral control effectiveness"));
    }

    // Initialize state variables for clean startup
    reset();

    debugLog(F("PIController initialized successfully"));
    return true;
  }

  void PIController::reset() {
    // Clear integral accumulation - this is critical for PI controllers
    // The integral term represents "error debt" accumulated over time
    // Resetting it prevents startup transients from old accumulated errors
    integral = 0.0f;
    output = 0.0f;

    debugLog(F("PIController state reset - integral accumulation cleared"));
  }

  float PIController::compute(float error) {
    // Implement the PI control algorithm with anti-windup protection
    // Output = Kp*error + Ki*∫error*dt
    //
    // The proportional term provides immediate response to current error
    // The integral term accumulates error over time to eliminate steady-state error
    // This combination ensures both responsiveness and precision

    // 1. PROPORTIONAL TERM: Immediate response to current error magnitude
    //    This term provides the primary driving force, just like in P control
    //    It responds instantly to deviations from the setpoint
    float p_term = Kp * error;

    // 2. INTEGRAL TERM: The "memory" of the controller
    //    This is where the magic happens for steady-state error elimination
    //
    //    Mathematical insight: ∫error*dt is approximated as sum(error*dt)
    //    Each control cycle, we add (error * dt * Ki) to the running total
    //
    //    Physical interpretation: If your robot consistently runs to the left
    //    of the line, the integral term builds up a positive correction that
    //    eventually becomes large enough to eliminate the bias

    // Accumulate error over time (Riemann sum approximation of integral)
    integral += Ki * error * dt;

    // 3. ANTI-WINDUP PROTECTION: Critical for practical PI controllers
    //    Without this, the integral term can grow huge during system saturation
    //    (when output hits limits), causing massive overshoot when limits are released
    //
    //    Practical example: If your line follower hits a wall and can't turn,
    //    the error accumulates in the integral term. When the obstacle is removed,
    //    the huge integral value causes massive overshoot. Anti-windup prevents this.
    integral = applyLimits(integral, -anti_windup, anti_windup);

    // The integral term contributes directly to the output
    float i_term = integral;

    // 4. COMBINE PROPORTIONAL AND INTEGRAL TERMS
    //    P term: "Where am I now?" (immediate error response)
    //    I term: "Where have I been?" (accumulated error correction)
    //    Together they provide both responsiveness and precision
    float pi_output = p_term + i_term;

    // 5. APPLY OUTPUT LIMITS
    //    Final protection against commanding impossible actuator values
    //    Note: We limit the integral term separately (anti-windup) AND
    //    the final output. This dual protection is essential for robust operation.
    output = applyLimits(pi_output, min_output, max_output);

    // Debug output reveals the inner workings of the PI algorithm
    // Understanding how P and I terms contribute helps with tuning
    if (debug_enabled) {
      Serial.print(F("PI: error="));
      Serial.print(error, 3);
      Serial.print(F(", P="));
      Serial.print(p_term, 2);
      Serial.print(F(", I="));
      Serial.print(i_term, 2);
      Serial.print(F(", integral_raw="));
      Serial.print(integral, 2);
      Serial.print(F(", output="));
      Serial.println(output, 2);
    }

    return output;
  }

  void PIController::setKp(float Kp) {
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

  void PIController::setKi(float Ki) {
    // Validate integral gain
    if (Ki < 0.0f) {
      debugLog(F("WARNING: setKi() - Negative Ki can cause instability"));
    }

    this->Ki = Ki;

    // Critical step: Reset integral when changing Ki
    // This prevents sudden output jumps because the integral accumulation
    // was calculated with the old Ki value. Changing Ki without resetting
    // the integral can cause the controller to "remember" corrections
    // that are now scaled differently, leading to discontinuous behavior.
    integral = 0.0f;

    if (debug_enabled) {
      Serial.print(F("Ki updated to "));
      Serial.print(Ki, 3);
      Serial.println(F(" (integral reset to prevent output jump)"));
    }
  }

  void PIController::setGains(float Kp, float Ki) {
    // Validate gain combination
    if (Kp < 0.0f || Ki < 0.0f) {
      debugLog(F("WARNING: setGains() - Negative gains can cause instability"));
    }

    // Update both gains simultaneously for consistency
    this->Kp = Kp;
    this->Ki = Ki;

    // Reset integral accumulation when changing gains
    // This is especially important when changing both gains because
    // their relative contributions to the output have changed
    integral = 0.0f;

    if (debug_enabled) {
      Serial.print(F("PI gains updated: Kp="));
      Serial.print(Kp, 3);
      Serial.print(F(", Ki="));
      Serial.print(Ki, 3);
      Serial.println(F(" (integral reset for consistency)"));
    }
  }

  void PIController::setAntiWindupLimit(float limit) {
    // Ensure limit is positive and reasonable
    limit = fabs(limit); // Force positive value

    // Educational insight: Anti-windup limit should be meaningful relative to output range
    // Too small: Prevents integral term from contributing meaningfully
    // Too large: Doesn't prevent windup effectively
    // Sweet spot: Usually 50-100% of maximum output range

    float max_possible = fabs(max_output);
    if (limit > max_possible * 2.0f) {
      // Warn if limit is much larger than output range - probably a mistake
      Serial.print(F("WARNING: Anti-windup limit ("));
      Serial.print(limit);
      Serial.print(F(") is much larger than max output ("));
      Serial.print(max_possible);
      Serial.println(F(") - consider reducing"));
    }

    // Set the new limit
    this->anti_windup = limit;

    // Re-clamp existing integral term to new limit immediately
    // This ensures the new limit takes effect right away rather than
    // waiting for the next compute() cycle
    integral = applyLimits(integral, -anti_windup, anti_windup);

    if (debug_enabled) {
      Serial.print(F("Anti-windup limit set to "));
      Serial.print(limit, 2);
      Serial.print(F(" ("));
      Serial.print((limit / max_possible) * 100.0f, 1);
      Serial.println(F("% of max output)"));
    }
  }

  float PIController::getKp() const {
    return Kp;
  }

  float PIController::getKi() const {
    return Ki;
  }

  float PIController::getIntegral() const {
    // This getter is invaluable for debugging PI controllers
    // Monitoring the integral value helps you understand:
    // - Is the controller accumulating error as expected?
    // - Is integral windup occurring (value approaching anti-windup limit)?
    // - Are you getting the steady-state error elimination you expect?
    return integral;
  }

} // namespace controller