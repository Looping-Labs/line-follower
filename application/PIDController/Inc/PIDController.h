#pragma once

#include "BaseController.h"

namespace controller {

  /**
   * @brief Full Proportional-Integral-Derivative Controller implementation
   *
   * The PID controller combines three control actions:
   * 1. Proportional (P): Responds to current error
   * 2. Integral (I): Responds to accumulated past errors
   * 3. Derivative (D): Responds to rate of error change
   *
   * Mathematical representation:
   * Output = (Kp × Error) + (Ki × ∫Error×dt) + (Kd × dError/dt)
   *
   * Characteristics:
   * - Excellent steady-state accuracy (zero steady-state error)
   * - Fast response with minimal overshoot when properly tuned
   * - Can handle complex system dynamics
   * - Requires careful tuning of three parameters
   * - May be sensitive to measurement noise (derivative term)
   *
   * For line following robots:
   * - Best performance for high-speed, precision line following
   * - Handles curved tracks and varying surface conditions well
   * - Maintains center-line tracking even with disturbances
   * - Ideal for competitive line following where speed and accuracy matter
   *
   * Common applications:
   * - High-performance line following robots
   * - Motor speed control
   * - Temperature control systems
   * - Robotic arm positioning
   * - Drone attitude control
   */
  class PIDController : public BaseController {
  private:
    /**
     * @brief PID controller parameters and state variables
     *
     * @var Kp: Proportional gain - controls reaction to current error
     *          For line following: affects how quickly robot steers when off-line
     *          Typical range: 0.1 to 50.0 depending on robot speed
     *
     * @var Ki: Integral gain - controls reaction to accumulated error
     *          For line following: corrects for systematic biases (sensor mounting, motor differences)
     *          Typical range: 0.0 to 10.0 (start with 0 and increase slowly)
     *
     * @var Kd: Derivative gain - controls reaction to error rate of change
     *          For line following: provides damping and predictive steering
     *          Typical range: 0.0 to 5.0 (often 1/4 to 1/10 of Kp)
     *
     * @var integral: Accumulated error over time (I term state)
     *                Reset to zero when controller is reset or gains change
     *
     * @var prev_error: Previous error value (needed for D term calculation)
     *                  Stored to calculate error rate: (current_error - prev_error) / dt
     *
     * @var anti_windup: Maximum allowed value for integral term
     *                   Prevents integral windup which can cause large overshoots
     *                   Should be set to a reasonable fraction of max_output
     */
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float anti_windup;

  public:
    /**
     * @brief Construct a new PID Controller
     *
     * @param Kp: Proportional gain (typically largest parameter)
     * @param Ki: Integral gain (start small, increase if steady-state error exists)
     * @param Kd: Derivative gain (provides damping, often 1/4 to 1/10 of Kp)
     * @param dt_ms: Time step in milliseconds (should match your control loop frequency)
     * @param min_output: Minimum output value (default -1023 for 10-bit PWM)
     * @param max_output: Maximum output value (default 1023 for 10-bit PWM)
     * @param debug: Enable debug output for tuning (default false)
     */
    PIDController(float Kp, float Ki, float Kd, uint32_t dt_ms = 1, float min_output = -1023.0f, float max_output = 1023.0f, bool debug = false);

    /**
     * @brief Initialize the PID controller
     *
     * Validates all parameters and sets up the controller for operation.
     * Call this before using the controller.
     *
     * @return bool true on success, false if parameters are invalid
     */
    bool init() override;

    /**
     * @brief Reset the controller state
     *
     * Clears integral accumulation and derivative history.
     * Call this when starting control or when setpoint changes significantly.
     */
    void reset() override;

    /**
     * @brief Calculate PID output based on error
     *
     * Implements the complete PID algorithm with anti-windup protection.
     * This is where the magic happens - all three control actions combine.
     *
     * @param error: Current error (setpoint - measured_value)
     *               For line following: 0 = centered, positive = right of line, negative = left of line
     * @return float Controller output between min_output and max_output
     */
    float compute(float error) override;

    /**
     * @brief Set the proportional gain
     *
     * @param Kp: Proportional gain (affects responsiveness)
     */
    void setKp(float Kp);

    /**
     * @brief Set the integral gain
     *
     * Resets integral accumulation to prevent sudden jumps.
     *
     * @param Ki: Integral gain (affects steady-state accuracy)
     */
    void setKi(float Ki);

    /**
     * @brief Set the derivative gain
     *
     * @param Kd: Derivative gain (affects stability and damping)
     */
    void setKd(float Kd);

    /**
     * @brief Set all PID gains simultaneously
     *
     * More efficient than setting gains individually.
     * Resets integral accumulation to prevent sudden jumps.
     *
     * @param Kp: Proportional gain
     * @param Ki: Integral gain
     * @param Kd: Derivative gain
     */
    void setGains(float Kp, float Ki, float Kd);

    /**
     * @brief Set the anti-windup limit for integral term
     *
     * Prevents integral windup which can cause large overshoots when
     * the system is saturated (output at limits) for extended periods.
     *
     * @param limit: Maximum absolute value for integral term
     *               Recommended: 0.5 to 1.0 times max_output
     */
    void setAntiWindupLimit(float limit);

    /**
     * @brief Get the proportional gain
     *
     * @return float Current Kp value
     */
    float getKp() const;

    /**
     * @brief Get the integral gain
     *
     * @return float Current Ki value
     */
    float getKi() const;

    /**
     * @brief Get the derivative gain
     *
     * @return float Current Kd value
     */
    float getKd() const;

    /**
     * @brief Get the current integral term value
     *
     * Useful for monitoring integral windup and tuning.
     *
     * @return float Current integral accumulation
     */
    float getIntegral() const;
  };
} // namespace controller