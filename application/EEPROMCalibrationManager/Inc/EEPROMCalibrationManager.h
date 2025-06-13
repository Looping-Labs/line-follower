/**
 * @file EEPROMCalibrationManager.h
 * @brief EEPROM Calibration Data Manager for Line Following Robots
 *
 * This class encapsulates all functionality related to storing and retrieving
 * sensor calibration data in EEPROM memory. It provides a clean, safe interface
 * for calibration persistence while hiding the complexity of data validation,
 * error handling, and EEPROM management.
 *
 * Key Design Principles:
 * - Encapsulation: All EEPROM operations are contained within this class
 * - Data Integrity: Multiple validation layers prevent corruption issues
 * - Error Recovery: Graceful handling of all failure modes
 * - Future-Proofing: Versioned data format supports evolution
 * - Simplicity: Easy-to-use interface hides implementation complexity
 *
 * @author json-dev
 * @date 2025-13-06
 * @version 1.0
 * @license MIT License
 */

#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <QTRSensors.h>

/**
 * @brief EEPROM Calibration Manager Class
 *
 * This class manages all aspects of storing and retrieving QTR sensor
 * calibration data in EEPROM. It handles data validation, corruption
 * detection, version management, and provides a simple interface for
 * the main application.
 *
 * The class follows the RAII (Resource Acquisition Is Initialization)
 * pattern - it initializes EEPROM in the constructor and ensures
 * proper cleanup. This guarantees that EEPROM is always in a valid
 * state when the class is used.
 *
 * Thread Safety: This class is NOT thread-safe. If used in a multi-
 * threaded environment, external synchronization is required.
 *
 * Example Usage:
 * @code
 * EEPROMCalibrationManager calibManager(8, true);
 * if (calibManager.isInitialized()) {
 *     if (calibManager.loadCalibration(qtr)) {
 *         Serial.println("Calibration loaded successfully");
 *     } else {
 *         Serial.println("Need to calibrate sensors");
 *     }
 * }
 * @endcode
 */
class EEPROMCalibrationManager {
public:
  /**
   * @brief Configuration constants
   *
   * These constants define the EEPROM layout and data format.
   * Changing these values may make existing stored data incompatible.
   */
  static const uint16_t CALIBRATION_MAGIC = 0xCAFE; ///< Magic number for data validation
  static const uint8_t CALIBRATION_VERSION = 1;     ///< Data format version
  static const uint16_t DEFAULT_EEPROM_SIZE = 512;  ///< Default EEPROM allocation
  static const uint16_t DEFAULT_START_ADDRESS = 0;  ///< Default storage start address
  static const uint8_t MAX_SENSORS = 16;            ///< Maximum supported sensors

  /**
   * @brief Error codes returned by class methods
   *
   * These codes provide specific information about why operations failed,
   * enabling appropriate error handling and user feedback.
   */
  enum class ErrorCode {
    SUCCESS = 0,               ///< Operation completed successfully
    EEPROM_INIT_FAILED,        ///< EEPROM initialization failed
    INVALID_SENSOR_COUNT,      ///< Sensor count out of valid range
    NO_VALID_DATA,             ///< No valid calibration data found
    MAGIC_NUMBER_MISMATCH,     ///< Magic number doesn't match expected value
    VERSION_MISMATCH,          ///< Data version incompatible with current code
    CHECKSUM_FAILED,           ///< Data corruption detected via checksum
    SENSOR_COUNT_MISMATCH,     ///< Stored sensor count doesn't match current hardware
    INVALID_CALIBRATION_RANGE, ///< Calibration min >= max for one or more sensors
    ADC_RANGE_EXCEEDED,        ///< Calibration values exceed ADC range
    EEPROM_WRITE_FAILED,       ///< Failed to write data to EEPROM
    EEPROM_COMMIT_FAILED,      ///< Failed to commit changes to EEPROM
    VERIFICATION_FAILED,       ///< Written data doesn't match what was intended
    NULL_POINTER_ERROR         ///< Null pointer passed to method
  };

private:
  /**
   * @brief Internal calibration data structure
   *
   * This structure defines the exact layout of data stored in EEPROM.
   * The order and size of fields is critical - changing this structure
   * will break compatibility with existing stored data.
   *
   * Design Notes:
   * - Fixed-size types ensure consistent behavior across platforms
   * - Magic number and version enable data validation
   * - Checksum provides corruption detection
   * - Flexible sensor count supports different hardware configurations
   *
   * Memory Layout (40 bytes for 8 sensors):
   * Offset 0-1:   Magic number (uint16_t)
   * Offset 2:     Version (uint8_t)
   * Offset 3:     Sensor count (uint8_t)
   * Offset 4-35:  Min/Max arrays (16 × uint16_t)
   * Offset 36-39: Checksum (uint32_t)
   */
  struct CalibrationData {
    uint16_t magic;                ///< Magic number for validation
    uint8_t version;               ///< Data format version
    uint8_t sensorCount;           ///< Number of sensors in this calibration
    uint16_t minimum[MAX_SENSORS]; ///< Minimum calibration values
    uint16_t maximum[MAX_SENSORS]; ///< Maximum calibration values
    uint32_t checksum;             ///< Data integrity checksum
  };

  // Instance variables - the class's internal state
  uint8_t sensorCount_;   ///< Number of sensors this instance manages
  uint16_t eepromSize_;   ///< Allocated EEPROM size in bytes
  uint16_t startAddress_; ///< EEPROM start address for data storage
  bool debugEnabled_;     ///< Enable debug output
  bool initialized_;      ///< Whether EEPROM was successfully initialized
  ErrorCode lastError_;   ///< Last error that occurred

  /**
   * @brief Calculate checksum for data integrity verification
   *
   * This method creates a hash of the calibration data that changes
   * dramatically if any bit is modified. The algorithm uses bit
   * rotation to ensure good distribution of hash values.
   *
   * @param data Pointer to calibration data structure
   * @return uint32_t Calculated checksum
   */
  uint32_t calculateChecksum(const CalibrationData *data) const;

  /**
   * @brief Validate calibration data structure
   *
   * Performs comprehensive validation of loaded calibration data:
   * - Magic number verification
   * - Version compatibility check
   * - Sensor count validation
   * - Checksum verification
   * - Range validation for all sensor values
   *
   * @param data Pointer to data structure to validate
   * @return ErrorCode indicating validation result
   */
  ErrorCode validateCalibrationData(const CalibrationData *data) const;

  /**
   * @brief Print debug message if debugging is enabled
   *
   * Centralized debug output with consistent formatting.
   * Uses F() macro to store strings in flash memory.
   *
   * @param message Debug message to print
   */
  void debugPrint(const String &message) const;

  /**
   * @brief Print error code information
   *
   * Converts error codes to human-readable descriptions
   * for better debugging and user feedback.
   *
   * @param error Error code to describe
   */
  void printErrorCode(ErrorCode error) const;

public:
  /**
   * @brief Construct a new EEPROM Calibration Manager
   *
   * This constructor initializes the EEPROM subsystem and prepares
   * the class for calibration data operations. It follows the RAII
   * pattern - if construction succeeds, the object is fully ready
   * to use.
   *
   * @param sensorCount Number of sensors in the calibration array (1-16)
   * @param debugEnabled Enable debug output for troubleshooting
   * @param eepromSize Total EEPROM size to allocate (default 512 bytes)
   * @param startAddress EEPROM start address for data storage (default 0)
   */
  EEPROMCalibrationManager(uint8_t sensorCount,
                           bool debugEnabled = false,
                           uint16_t eepromSize = DEFAULT_EEPROM_SIZE,
                           uint16_t startAddress = DEFAULT_START_ADDRESS);

  /**
   * @brief Destructor - clean up EEPROM resources
   *
   * The destructor ensures proper cleanup of EEPROM resources.
   * In the current Arduino EEPROM implementation, this is minimal,
   * but it provides a place for future cleanup if needed.
   */
  ~EEPROMCalibrationManager();

  /**
   * @brief Check if the manager was successfully initialized
   *
   * Always call this method after construction to verify the
   * object is ready for use. If initialization failed, all
   * other methods will return appropriate error codes.
   *
   * @return bool true if initialized successfully, false otherwise
   */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief Get the last error that occurred
   *
   * When any method returns false or fails, this method provides
   * detailed information about what went wrong. Essential for
   * debugging and providing meaningful user feedback.
   *
   * @return ErrorCode describing the last error
   */
  ErrorCode getLastError() const { return lastError_; }

  /**
   * @brief Save QTR sensor calibration to EEPROM
   *
   * Takes the current calibration data from a QTRSensors object
   * and stores it persistently in EEPROM with full validation
   * and error checking.
   *
   * This method performs the following operations:
   * 1. Validates that meaningful calibration data exists
   * 2. Creates calibration data structure with metadata
   * 3. Calculates and stores integrity checksum
   * 4. Writes data to EEPROM with error checking
   * 5. Verifies written data by reading it back
   *
   * @param qtr Reference to QTRSensors object containing calibration
   * @return bool true if save successful, false on any error
   */
  bool saveCalibration(const QTRSensors &qtr);

  /**
   * @brief Load calibration data from EEPROM and apply to QTR sensors
   *
   * Reads stored calibration data from EEPROM, validates it thoroughly,
   * and applies it to the provided QTRSensors object. This method
   * implements comprehensive error checking and will only apply
   * data that passes all validation tests.
   *
   * @param qtr Reference to QTRSensors object to receive calibration
   * @return bool true if load and apply successful, false otherwise
   */
  bool loadCalibration(QTRSensors &qtr);

  /**
   * @brief Check if valid calibration data exists in EEPROM
   *
   * Performs a non-destructive check for valid calibration data
   * without modifying any QTR sensor objects. Useful for
   * startup logic and user interface decisions.
   *
   * @return bool true if valid calibration data is available
   */
  bool hasValidCalibration();

  /**
   * @brief Clear all calibration data from EEPROM
   *
   * Securely erases calibration data from EEPROM by overwriting
   * the storage area with zeros. This ensures no remnants of
   * old calibration data remain that could cause confusion.
   *
   * @return bool true if clear operation successful
   */
  bool clearCalibration();

  /**
   * @brief Display current calibration data from EEPROM
   *
   * Reads and displays calibration data for debugging and
   * verification purposes. Shows all stored values along
   * with metadata like version and checksum.
   *
   * This method is read-only and safe to call anytime.
   */
  void displayStoredCalibration();

  /**
   * @brief Get human-readable description of error code
   *
   * Converts error codes into descriptive strings for
   * user interfaces and debugging output.
   *
   * @param error Error code to describe
   * @return String containing error description
   */
  String getErrorDescription(ErrorCode error) const;

  /**
   * @brief Enable or disable debug output
   *
   * Controls whether the class produces detailed debug output.
   * Can be changed at runtime for dynamic debugging control.
   *
   * @param enabled true to enable debug output, false to disable
   */
  void setDebugEnabled(bool enabled) { debugEnabled_ = enabled; }

  /**
   * @brief Get current sensor count configuration
   *
   * @return uint8_t Number of sensors this manager handles
   */
  uint8_t getSensorCount() const { return sensorCount_; }

  /**
   * @brief Calculate required EEPROM storage size
   *
   * Static utility method that calculates how much EEPROM space
   * is needed for a given sensor configuration. Useful for
   * planning EEPROM layout in complex systems.
   *
   * @param sensorCount Number of sensors to calculate for
   * @return uint16_t Required EEPROM bytes
   */
  static uint16_t calculateStorageSize(uint8_t sensorCount);
};