#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace controller {
  /**
   * @brief Base Controller class for different control strategies
   *
   * Abstract base class that defines the common interface and functionality
   * for all controller types (P, PI, PD, PID).
   *
   * This class provides the foundation for implementing various control algorithms
   * commonly used in robotics and embedded systems.
   */
  class BaseController {
  protected:
    /**
     * @brief Common controller parameters
     *
     * @var setpoint: Desired target value (e.g., desired position, speed, etc.)
     * @var output: Controller output (e.g., motor PWM value, servo position)
     * @var dt: Time step in seconds (converted from milliseconds for calculations)
     * @var min_output: Minimum output value (prevents actuator damage)
     * @var max_output: Maximum output value (prevents actuator damage)
     * @var debug_enabled: Flag to enable/disable debug output
     */
    float setpoint;
    float output;
    float dt;
    float min_output;
    float max_output;
    bool debug_enabled;

    /**
     * @brief Apply limits to a value
     *
     * This function implements saturation limiting to prevent actuator damage
     * and ensure the output stays within safe operational bounds.
     *
     * @param value: Value to limit
     * @param min: Minimum allowed value
     * @param max: Maximum allowed value
     * @return float Limited value
     */
    float applyLimits(float value, float min, float max) const;

    /**
     * @brief Debug logging function
     *
     * Provides consistent debug output formatting across all controller types
     *
     * @param message: Debug message to print
     */
    void debugLog(const String &message) const;

  public:
    /**
     * @brief Construct a new Base Controller
     *
     * @param dt_ms: Time step in milliseconds (default 1 ms for high-frequency control)
     * @param min_output: Minimum output value (default -1023 for 10-bit resolution)
     * @param max_output: Maximum output value (default 1023 for 10-bit resolution)
     * @param debug: Enable debug output (default false)
     */
    BaseController(uint32_t dt_ms = 1, float min_output = -1023.0f, float max_output = 1023.0f, bool debug = false);

    /**
     * @brief Virtual destructor for proper cleanup in derived classes
     *
     * Ensures proper cleanup when deleting objects through base class pointers
     */
    virtual ~BaseController() = default;

    /**
     * @brief Initialize the controller
     *
     * Performs basic initialization common to all controller types.
     * Derived classes should call this method in their init() implementation.
     *
     * @return bool true on success, false otherwise
     */
    virtual bool init();

    /**
     * @brief Reset the controller state
     *
     * Pure virtual function that must be implemented by derived classes
     * to reset their specific state variables (integral terms, previous errors, etc.)
     */
    virtual void reset() = 0;

    /**
     * @brief Calculate controller output based on error
     *
     * Pure virtual function that implements the core control algorithm.
     * Each controller type (P, PI, PD, PID) implements this differently.
     *
     * @param error: Current error (setpoint - measured_value)
     * @return float Controller output between min_output and max_output
     */
    virtual float compute(float error) = 0;

    /**
     * @brief Calculate controller output based on setpoint and measured value
     *
     * Convenience function that calculates error and calls compute()
     *
     * @param measured_value: Current measured value from sensor
     * @return float Controller output between min_output and max_output
     */
    float computeWithSetpoint(float measured_value);

    /**
     * @brief Set the sample time
     *
     * Updates the time step used in integral and derivative calculations
     *
     * @param dt_ms: Time step in milliseconds
     */
    void setSampleTime(uint32_t dt_ms);

    /**
     * @brief Set the output limits
     *
     * Updates the bounds for controller output to prevent actuator damage
     *
     * @param min_output: Minimum output value
     * @param max_output: Maximum output value
     */
    void setOutputLimits(float min_output, float max_output);

    /**
     * @brief Set the setpoint
     *
     * Updates the target value the controller tries to reach
     *
     * @param setpoint: Desired target value
     */
    void setSetpoint(float setpoint);

    /**
     * @brief Enable or disable debug output
     *
     * @param enable: true to enable debug output, false to disable
     */
    void setDebugEnabled(bool enable);

    /**
     * @brief Get the sample time
     *
     * @return float Time step in seconds
     */
    float getSampleTime() const;

    /**
     * @brief Get the setpoint
     *
     * @return float Setpoint
     */
    float getSetpoint() const;

    /**
     * @brief Get the last output
     *
     * @return float Last computed output
     */
    float getOutput() const;
  };
} // namespace controller