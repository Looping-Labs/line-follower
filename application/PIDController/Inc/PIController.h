#pragma once

#include "BaseController.h"

namespace controller {
  /**
   * @brief Proportional-Integral Controller implementation
   *
   * The PI controller combines proportional and integral control actions.
   * This combination provides excellent steady-state accuracy while maintaining
   * reasonable transient response, though it may exhibit some overshoot.
   *
   * Mathematical representation: Output = (Kp × Error) + (Ki × ∫Error×dt)
   *
   * The magic of the integral term: It accumulates error over time, which means
   * even small consistent errors will eventually build up enough correction
   * to eliminate them completely. This is why PI controllers achieve zero
   * steady-state error for step inputs.
   *
   * Characteristics:
   * - Zero steady-state error for step inputs (the integral term ensures this)
   * - Good transient response when properly tuned
   * - May exhibit overshoot due to integral accumulation
   * - Requires anti-windup protection to prevent excessive overshoot
   * - Less sensitive to measurement noise than PID (no derivative term)
   * - Two parameters to tune (simpler than full PID)
   *
   * Best used for:
   * - Systems where precise setpoint tracking is essential
   * - Applications with steady disturbances that need to be rejected
   * - Processes where some overshoot is acceptable
   * - Systems with relatively slow dynamics
   * - Environments with measurement noise (derivative-free advantage)
   *
   * For line following robots:
   * - Excellent for precision line following where exact centering is critical
   * - Compensates for systematic biases (sensor mounting, motor differences)
   * - Handles track imperfections and lighting variations well
   * - May overshoot on sharp turns (can be mitigated with anti-windup tuning)
   * - Good for moderate-speed applications where precision matters most
   *
   * Real-world applications:
   * - Temperature control systems (thermostats, ovens, heaters)
   * - Level control in tanks and vessels
   * - Pressure regulation systems
   * - Motor speed control where precision is critical
   * - Chemical process control with steady disturbances
   */
  class PIController : public BaseController {
  private:
    /**
     * @brief PI Controller parameters and state variables
     *
     * @var Kp: Proportional gain - controls immediate response to current error
     *          Higher values = faster response but potential instability
     *          Typical range for line following: 0.5 to 10.0
     *
     * @var Ki: Integral gain - controls response to accumulated error over time
     *          Higher values = faster elimination of steady-state error
     *          But also higher risk of overshoot and oscillation
     *          Typical range: 0.01 to 2.0 (usually much smaller than Kp)
     *
     *          Critical insight: Ki units are "per second" - it's the rate
     *          at which integral correction accumulates. This is why sample
     *          time (dt) is so important for integral calculations.
     *
     * @var integral: Accumulated error over time (the "memory" of the controller)
     *                This value grows when there's consistent error in one direction
     *                and shrinks when error is in the opposite direction
     *                Reset to zero when controller is reset or gains change
     *
     *                Physical meaning: Represents the total "error debt" that
     *                needs to be corrected to achieve perfect tracking
     *
     * @var anti_windup: Maximum allowed absolute value for integral term
     *                   Critical for preventing integral windup - a condition
     *                   where integral accumulates to huge values during
     *                   system saturation, causing massive overshoot when
     *                   the system comes out of saturation
     *
     *                   Should typically be set to 50-100% of max_output
     */
    float Kp;
    float Ki;
    float integral;
    float anti_windup;

  public:
    /**
     * @brief Construct a new PI Controller
     *
     * @param Kp: Proportional gain (primary response parameter)
     * @param Ki: Integral gain (steady-state error elimination parameter)
     * @param dt_ms: Time step in milliseconds (critical for integral accuracy)
     * @param min_output: Minimum output value (default -1023 for 10-bit PWM)
     * @param max_output: Maximum output value (default 1023 for 10-bit PWM)
     * @param debug: Enable debug output for tuning (default false)
     */
    PIController(float Kp, float Ki, uint32_t dt_ms = 1, float min_output = -1023.0f,
                 float max_output = 1023.0f, bool debug = false);

    /**
     * @brief Initialize the PI controller
     *
     * Validates parameters and sets up anti-windup protection.
     * The anti-windup limit is automatically set to the maximum output
     * unless explicitly changed later.
     *
     * @return bool true on success, false if parameters are invalid
     */
    bool init() override;

    /**
     * @brief Reset the controller state
     *
     * Clears integral accumulation to prevent startup transients.
     * This is essential when starting control, changing setpoints significantly,
     * or recovering from system faults.
     */
    void reset() override;

    /**
     * @brief Calculate PI output based on error
     *
     * Implements the PI algorithm with anti-windup protection.
     * The integral term is the key differentiator - it accumulates
     * error over time and provides the correction needed for
     * zero steady-state error.
     *
     * @param error: Current error (setpoint - measured_value)
     *               For line following: 0 = centered, +/- = off to sides
     * @return float Controller output between min_output and max_output
     */
    float compute(float error) override;

    /**
     * @brief Set the proportional gain
     *
     * @param Kp: Proportional gain (affects response speed)
     */
    void setKp(float Kp);

    /**
     * @brief Set the integral gain
     *
     * Resets integral accumulation to prevent sudden output jumps.
     * This is necessary because changing Ki changes the "weight" of
     * the accumulated error, which could cause discontinuous output.
     *
     * @param Ki: Integral gain (affects steady-state accuracy)
     */
    void setKi(float Ki);

    /**
     * @brief Set both PI gains simultaneously
     *
     * More efficient than setting gains individually.
     * Resets integral accumulation to ensure smooth operation.
     *
     * @param Kp: Proportional gain
     * @param Ki: Integral gain
     */
    void setGains(float Kp, float Ki);

    /**
     * @brief Set the anti-windup limit for integral term
     *
     * This is one of the most important tuning parameters for PI controllers.
     * Too high: allows integral windup and massive overshoot
     * Too low: prevents integral term from doing its job
     *
     * Good starting point: 50-80% of your maximum output range
     *
     * @param limit: Maximum absolute value for integral term
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
     * @brief Get the current integral term value
     *
     * This is extremely useful for debugging and tuning:
     * - Large positive/negative values indicate potential windup
     * - Oscillating values suggest Ki might be too high
     * - Values that don't accumulate suggest Ki might be too low
     *
     * @return float Current integral accumulation
     */
    float getIntegral() const;
  };
} // namespace controller