#include "PIDController.h"
#include <math.h> // For fabs() function

namespace controller {

  PIDController::PIDController(float Kp, float Ki, float Kd, uint32_t dt_ms,
                               float min_output, float max_output, bool debug)
      : BaseController(dt_ms, min_output, max_output, debug),
        Kp(Kp), Ki(Ki), Kd(Kd),
        integral(0.0f), prev_error(0.0f),
        anti_windup(fabs(max_output)) { // Default anti-windup limit = max output

    // Validate PID parameters and warn about common mistakes
    if (Kp < 0.0f) {
      Serial.println(F("WARNING: PIDController - Negative Kp can cause instability"));
    }
    if (Ki < 0.0f) {
      Serial.println(F("WARNING: PIDController - Negative Ki can cause instability"));
    }
    if (Kd < 0.0f) {
      Serial.println(F("WARNING: PIDController - Negative Kd can cause instability"));
    }

    // Check for unrealistic gain combinations
    if (Ki > 0.0f && Kp == 0.0f) {
      Serial.println(F("WARNING: PIDController - Ki without Kp may cause oscillation"));
    }
    if (Kd > Kp * 10.0f) {
      Serial.println(F("WARNING: PIDController - Very high Kd relative to Kp may cause noise sensitivity"));
    }

    if (debug_enabled) {
      Serial.print(F("PIDController: Created with Kp="));
      Serial.print(Kp, 3);
      Serial.print(F(", Ki="));
      Serial.print(Ki, 3);
      Serial.print(F(", Kd="));
      Serial.print(Kd, 3);
      Serial.print(F(", dt="));
      Serial.print(dt * 1000.0f);
      Serial.println(F("ms"));
    }
  }

  bool PIDController::init() {
    // Call base class initialization first
    if (!BaseController::init()) {
      Serial.println(F("ERROR: PIDController::init() - Base initialization failed"));
      return false;
    }

    // Validate PID-specific parameters
    if (Kp < 0.0f || Ki < 0.0f || Kd < 0.0f) {
      Serial.println(F("ERROR: PIDController::init() - Gains cannot be negative"));
      return false;
    }

    // Ensure we have at least one non-zero gain
    if (Kp == 0.0f && Ki == 0.0f && Kd == 0.0f) {
      Serial.println(F("ERROR: PIDController::init() - All gains are zero"));
      return false;
    }

    // Initialize state variables to ensure clean start
    reset();

    debugLog(F("PIDController initialized successfully"));
    return true;
  }

  void PIDController::reset() {
    // Clear all state variables for a fresh start
    // This is critical when starting control or changing setpoints significantly
    integral = 0.0f;
    prev_error = 0.0f;
    output = 0.0f;

    debugLog(F("PIDController state reset - integral and derivative history cleared"));
  }

  float PIDController::compute(float error) {
    // Implement the complete PID algorithm
    // Output = Kp*error + Ki*∫error*dt + Kd*derror/dt

    // 1. PROPORTIONAL TERM: Responds to current error magnitude
    //    Higher error = higher proportional response
    float p_term = Kp * error;

    // 2. INTEGRAL TERM: Responds to accumulated error over time
    //    Eliminates steady-state error by building up correction over time
    integral += Ki * error * dt; // Accumulate error × time × gain

    // Apply anti-windup protection to prevent integral term from growing too large
    // This prevents overshoot when the system is saturated (output at limits)
    integral = applyLimits(integral, -anti_windup, anti_windup);
    float i_term = integral;

    // 3. DERIVATIVE TERM: Responds to rate of error change
    //    Provides predictive action and damping to reduce overshoot
    //    Note: We calculate derivative of ERROR, not just setpoint change
    //    This avoids "derivative kick" when setpoint changes suddenly
    float error_rate = (error - prev_error) / dt;
    float d_term = Kd * error_rate;

    // Store current error for next iteration's derivative calculation
    prev_error = error;

    // 4. COMBINE ALL THREE TERMS
    //    Each term contributes to the final control output
    float pid_output = p_term + i_term + d_term;

    // 5. APPLY OUTPUT LIMITS
    //    Protect actuators and ensure output stays within safe bounds
    output = applyLimits(pid_output, min_output, max_output);

    // Debug output shows each term's contribution for tuning purposes
    if (debug_enabled) {
      Serial.print(F("PID: error="));
      Serial.print(error, 3);
      Serial.print(F(", P="));
      Serial.print(p_term, 2);
      Serial.print(F(", I="));
      Serial.print(i_term, 2);
      Serial.print(F(", D="));
      Serial.print(d_term, 2);
      Serial.print(F(", output="));
      Serial.println(output, 2);
    }

    return output;
  }

  void PIDController::setKp(float Kp) {
    if (Kp < 0.0f) {
      debugLog(F("WARNING: setKp() - Negative Kp can cause instability"));
    }

    this->Kp = Kp;

    if (debug_enabled) {
      Serial.print(F("Kp updated to "));
      Serial.println(Kp, 3);
    }
  }

  void PIDController::setKi(float Ki) {
    if (Ki < 0.0f) {
      debugLog(F("WARNING: setKi() - Negative Ki can cause instability"));
    }

    this->Ki = Ki;

    // Reset integral when changing Ki to prevent sudden jumps in output
    // This is important because the integral term represents accumulated error
    // and changing Ki changes the "weight" of that accumulation
    integral = 0.0f;

    if (debug_enabled) {
      Serial.print(F("Ki updated to "));
      Serial.print(Ki, 3);
      Serial.println(F(" (integral reset)"));
    }
  }

  void PIDController::setKd(float Kd) {
    if (Kd < 0.0f) {
      debugLog(F("WARNING: setKd() - Negative Kd can cause instability"));
    }

    this->Kd = Kd;

    if (debug_enabled) {
      Serial.print(F("Kd updated to "));
      Serial.println(Kd, 3);
    }
  }

  void PIDController::setGains(float Kp, float Ki, float Kd) {
    // Validate all gains together
    if (Kp < 0.0f || Ki < 0.0f || Kd < 0.0f) {
      debugLog(F("WARNING: setGains() - Negative gains can cause instability"));
    }

    // Update all gains simultaneously
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    // Reset integral accumulation when changing gains to prevent sudden jumps
    integral = 0.0f;

    if (debug_enabled) {
      Serial.print(F("PID gains updated: Kp="));
      Serial.print(Kp, 3);
      Serial.print(F(", Ki="));
      Serial.print(Ki, 3);
      Serial.print(F(", Kd="));
      Serial.print(Kd, 3);
      Serial.println(F(" (integral reset)"));
    }
  }

  void PIDController::setAntiWindupLimit(float limit) {
    // Ensure limit is positive and reasonable
    limit = fabs(limit); // Make positive

    // Cap the anti-windup limit to the maximum possible output
    // This prevents setting unrealistic limits
    float max_possible = fabs(max_output);
    if (limit > max_possible) {
      limit = max_possible;
      debugLog(F("Anti-windup limit capped to max_output"));
    }

    this->anti_windup = limit;

    // Re-clamp existing integral term to new limit
    // This ensures immediate compliance with the new limit
    integral = applyLimits(integral, -anti_windup, anti_windup);

    if (debug_enabled) {
      Serial.print(F("Anti-windup limit set to "));
      Serial.println(limit, 2);
    }
  }

  float PIDController::getKp() const {
    return Kp;
  }

  float PIDController::getKi() const {
    return Ki;
  }

  float PIDController::getKd() const {
    return Kd;
  }

  float PIDController::getIntegral() const {
    return integral;
  }

} // namespace controller