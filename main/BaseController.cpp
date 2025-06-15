#include "BaseController.h"

namespace controller {

  BaseController::BaseController(uint32_t dt_ms, float min_output, float max_output, bool debug)
      : setpoint(0.0f), output(0.0f), min_output(min_output), max_output(max_output), debug_enabled(debug) {

    // Convert milliseconds to seconds for internal calculations
    // This is crucial for proper integral and derivative calculations
    dt = dt_ms / 1000.0f;

    // Validate constructor parameters to prevent common setup mistakes
    if (dt_ms == 0) {
      Serial.println(F("WARNING: BaseController - dt_ms cannot be zero, setting to 1ms"));
      dt = 0.001f; // 1ms default
    }

    if (min_output >= max_output) {
      Serial.println(F("WARNING: BaseController - min_output >= max_output, swapping values"));
      float temp = min_output;
      this->min_output = max_output;
      this->max_output = temp;
    }

    if (debug_enabled) {
      Serial.print(F("BaseController: Created with dt="));
      Serial.print(dt * 1000.0f);
      Serial.print(F("ms, limits=["));
      Serial.print(this->min_output);
      Serial.print(F(", "));
      Serial.print(this->max_output);
      Serial.println(F("]"));
    }
  }

  bool BaseController::init() {
    // Basic initialization - validate parameters and reset state
    if (dt <= 0.0f) {
      Serial.println(F("ERROR: BaseController::init() - Invalid sample time"));
      return false;
    }

    if (min_output >= max_output) {
      Serial.println(F("ERROR: BaseController::init() - Invalid output limits"));
      return false;
    }

    // Initialize output to safe value (zero)
    output = 0.0f;
    setpoint = 0.0f;

    debugLog(F("BaseController initialized successfully"));
    return true;
  }

  float BaseController::applyLimits(float value, float min, float max) const {
    // Implement saturation limiting with clear logic flow
    if (value > max) {
      return max;
    }
    if (value < min) {
      return min;
    }
    return value;
  }

  void BaseController::debugLog(const String &message) const {
    // Centralized debug logging with timestamp for better debugging
    if (debug_enabled) {
      Serial.print(F("["));
      Serial.print(millis());
      Serial.print(F("ms] "));
      Serial.println(message);
    }
  }

  float BaseController::computeWithSetpoint(float measured_value) {
    // Calculate error and call the pure virtual compute function
    // Error = desired - actual (positive error means we need to increase output)
    float error = setpoint - measured_value;

    if (debug_enabled) {
      Serial.print(F("ComputeWithSetpoint: setpoint="));
      Serial.print(setpoint);
      Serial.print(F(", measured="));
      Serial.print(measured_value);
      Serial.print(F(", error="));
      Serial.println(error);
    }

    return compute(error);
  }

  void BaseController::setSampleTime(uint32_t dt_ms) {
    // Validate and update sample time
    if (dt_ms == 0) {
      debugLog(F("WARNING: setSampleTime() - dt_ms cannot be zero, ignoring"));
      return;
    }

    dt = dt_ms / 1000.0f;

    if (debug_enabled) {
      Serial.print(F("Sample time set to "));
      Serial.print(dt_ms);
      Serial.println(F("ms"));
    }
  }

  void BaseController::setOutputLimits(float min_output, float max_output) {
    // Validate and update output limits
    if (min_output >= max_output) {
      debugLog(F("WARNING: setOutputLimits() - min >= max, swapping values"));
      float temp = min_output;
      min_output = max_output;
      max_output = temp;
    }

    this->min_output = min_output;
    this->max_output = max_output;

    // Clamp current output to new limits to prevent sudden jumps
    output = applyLimits(output, min_output, max_output);

    if (debug_enabled) {
      Serial.print(F("Output limits set to ["));
      Serial.print(min_output);
      Serial.print(F(", "));
      Serial.print(max_output);
      Serial.println(F("]"));
    }
  }

  void BaseController::setSetpoint(float setpoint) {
    this->setpoint = setpoint;

    if (debug_enabled) {
      Serial.print(F("Setpoint set to "));
      Serial.println(setpoint);
    }
  }

  void BaseController::setDebugEnabled(bool enable) {
    debug_enabled = enable;
    if (enable) {
      Serial.println(F("Debug output enabled"));
    }
  }

  float BaseController::getSampleTime() const {
    return dt;
  }

  float BaseController::getSetpoint() const {
    return setpoint;
  }

  float BaseController::getOutput() const {
    return output;
  }

} // namespace controller