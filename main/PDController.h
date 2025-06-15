#pragma once

#include "BaseController.h"

namespace controller {

  /**
   * @brief Proportional-Derivative Controller implementation
   *
   * The PD controller combines proportional and derivative control actions.
   * It provides excellent stability and transient response characteristics
   * while remaining simpler to tune than a full PID controller.
   *
   * Mathematical representation: Output = (Kp × Error) + (Kd × dError/dt)
   *
   * Characteristics:
   * - Excellent stability and damping
   * - Fast response with minimal overshoot
   * - Resistant to oscillations
   * - May have steady-state error (no integral term)
   * - Less sensitive to measurement noise than full PID
   * - Only two parameters to tune (simpler than PID)
   *
   * Best used for:
   * - Systems where stability is more important than precision
   * - Noisy environments where derivative filtering is beneficial
   * - Applications where some steady-state error is acceptable
   * - Fast-responding systems that tend to overshoot
   *
   * For line following robots:
   * - Excellent for high-speed line following where stability matters most
   * - Provides good cornering performance with minimal oscillation
   * - Handles sudden line direction changes well
   * - May run consistently to one side of line (steady-state error)
   * - Less sensitive to sensor noise than full PID
   *
   * Real-world applications:
   * - Robotic arm damping control
   * - Drone attitude stabilization
   * - Vehicle suspension systems
   * - High-speed manufacturing equipment
   * - Antenna tracking systems
   */
  class PDController : public BaseController {
  private:
    /**
     * @brief PD Controller parameters and state variables
     *
     * @var Kp: Proportional gain - controls reaction to current error magnitude
     *          Higher values = more aggressive response to position errors
     *          Typical range for line following: 1.0 to 20.0
     *
     * @var Kd: Derivative gain - controls reaction to error rate of change
     *          Higher values = more damping, slower response
     *          Typical range: 0.1 to 5.0 (often 1/4 to 1/2 of Kp)
     *          Critical for preventing overshoot and oscillation
     *
     * @var prev_error: Previous error value stored for derivative calculation
     *                  Used in formula: derivative = (current_error - prev_error) / dt
     *                  Reset to 0 when controller is reset or initialized
     *
     * Note: No integral term means no anti-windup protection needed
     *       This simplifies the controller but may allow steady-state error
     */
    float Kp;
    float Kd;
    float prev_error;

  public:
    /**
     * @brief Construct a new PD Controller
     *
     * @param Kp: Proportional gain (should be > 0 for stable operation)
     * @param Kd: Derivative gain (should be > 0 for damping effect)
     * @param dt_ms: Time step in milliseconds (critical for derivative calculation accuracy)
     * @param min_output: Minimum output value (default -1023 for 10-bit PWM)
     * @param max_output: Maximum output value (default 1023 for 10-bit PWM)
     * @param debug: Enable debug output for tuning (default false)
     */
    PDController(float Kp, float Kd, uint32_t dt_ms = 1, float min_output = -1023.0f,
                 float max_output = 1023.0f, bool debug = false);

    /**
     * @brief Initialize the PD controller
     *
     * Validates parameters and prepares controller for operation.
     * Ensures gains are reasonable and timing is consistent.
     *
     * @return bool true on success, false if parameters are invalid
     */
    bool init() override;

    /**
     * @brief Reset the controller state
     *
     * Clears derivative calculation history to prevent startup transients.
     * Call this when starting control or when setpoint changes dramatically.
     */
    void reset() override;

    /**
     * @brief Calculate PD output based on error
     *
     * Implements the core PD algorithm with proper derivative calculation.
     * The derivative term is calculated on the error signal to avoid
     * "derivative kick" when the setpoint changes suddenly.
     *
     * @param error: Current error (setpoint - measured_value)
     *               For line following: 0 = centered, +/- = off to sides
     * @return float Controller output between min_output and max_output
     */
    float compute(float error) override;

    /**
     * @brief Set the proportional gain
     *
     * @param Kp: Proportional gain (affects response speed and stability)
     */
    void setKp(float Kp);

    /**
     * @brief Set the derivative gain
     *
     * @param Kd: Derivative gain (affects stability and damping)
     */
    void setKd(float Kd);

    /**
     * @brief Set both PD gains simultaneously
     *
     * More efficient than setting gains individually and ensures
     * consistent controller state during parameter updates.
     *
     * @param Kp: Proportional gain
     * @param Kd: Derivative gain
     */
    void setGains(float Kp, float Kd);

    /**
     * @brief Get the proportional gain
     *
     * @return float Current Kp value
     */
    float getKp() const;

    /**
     * @brief Get the derivative gain
     *
     * @return float Current Kd value
     */
    float getKd() const;
  };

} // namespace controller