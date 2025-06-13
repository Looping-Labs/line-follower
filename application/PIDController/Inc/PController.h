#pragma once

#include "BaseController.h"

namespace controller {

  /**
   * @brief Proportional-Only Controller implementation
   *
   * The P controller is the simplest form of feedback control.
   * It provides output proportional to the current error.
   *
   * Mathematical representation: Output = Kp × Error
   *
   * Characteristics:
   * - Fast response to errors
   * - Simple to tune (only one parameter)
   * - May have steady-state error (doesn't reach exact setpoint)
   * - Can be unstable with high gains
   *
   * Best used for:
   * - Simple positioning systems where some error is acceptable
   * - First-order systems (like RC circuits)
   * - Quick prototyping and testing
   * - Systems that don't require precision
   *
   * For line following robots:
   * - Good for basic line following where approximate centering is sufficient
   * - Fast response to line deviations
   * - May oscillate around the line with high speeds
   */
  class PController : public BaseController {
  private:
    /**
     * @brief Proportional gain
     *
     * @var Kp: Proportional gain - determines how aggressively the controller responds to error
     *          Higher values = faster response but potential instability
     *          Lower values = slower response but more stable
     *
     * Typical ranges for line following:
     * - Slow robots (< 0.5 m/s): Kp = 0.1 to 2.0
     * - Medium robots (0.5-2 m/s): Kp = 2.0 to 10.0
     * - Fast robots (> 2 m/s): Kp = 10.0 to 50.0
     */
    float Kp;

  public:
    /**
     * @brief Construct a new P Controller
     *
     * @param Kp: Proportional gain (must be > 0 for stable operation)
     * @param dt_ms: Time step in milliseconds (used for consistency with other controllers)
     * @param min_output: Minimum output value (default -1023 for 10-bit PWM)
     * @param max_output: Maximum output value (default 1023 for 10-bit PWM)
     * @param debug: Enable debug output (default false)
     */
    PController(float Kp, uint32_t dt_ms = 1, float min_output = -1023.0f, float max_output = 1023.0f, bool debug = false);

    /**
     * @brief Initialize the P controller
     *
     * Validates parameters and prepares the controller for operation
     *
     * @return bool true on success, false if parameters are invalid
     */
    bool init() override;

    /**
     * @brief Reset the controller state
     *
     * For P controller, only resets the output to zero
     * (no internal state to reset unlike PI or PID controllers)
     */
    void reset() override;

    /**
     * @brief Calculate P output based on error
     *
     * Implements the core P control algorithm: Output = Kp × Error
     *
     * @param error: Current error (setpoint - measured_value)
     *               For line following: 0 = on line, +/- = off to sides
     * @return float Controller output between min_output and max_output
     */
    float compute(float error) override;

    /**
     * @brief Set the proportional gain
     *
     * Updates Kp value. Use this for real-time tuning during operation.
     *
     * @param Kp: Proportional gain (should be > 0)
     */
    void setKp(float Kp);

    /**
     * @brief Get the proportional gain
     *
     * @return float Current proportional gain
     */
    float getKp() const;
  };

} // namespace controller