/**
 * @file EEPROMCalibrationManager.h
 * @brief Production-Ready EEPROM Calibration Manager for ESP32 Line Following Robots
 *
 * This class represents the culmination of systematic embedded systems design,
 * incorporating lessons learned through real-world debugging and optimization.
 * It demonstrates professional approaches to:
 *
 * - Cooperative resource management (works with system-level EEPROM initialization)
 * - Memory-efficient data structures (40-byte structure fits in 64-byte EEPROM)
 * - Comprehensive error handling with specific diagnostic information
 * - Robust data integrity protection through checksums and validation
 * - Graceful degradation when resources are constrained
 * - Clear separation between system concerns and application logic
 *
 * Key Architectural Decisions Explained:
 *
 * 1. Cooperative Resource Management: Instead of trying to manage EEPROM
 *    initialization internally, this class works with system-level EEPROM
 *    initialization. This prevents resource conflicts
 *
 * 2. Memory-Optimized Data Structure: The CalibrationData structure is
 *    carefully designed to use exactly 40 bytes, fitting comfortably
 *    within typical ESP32 EEPROM allocations while leaving room for growth.
 *
 * 3. Comprehensive Error Taxonomy: Each possible failure mode has a specific
 *    error code that guides appropriate recovery strategies, enabling
 *    sophisticated error handling in calling code.
 *
 * 4. Data Integrity Through Multiple Layers: Magic numbers, version tracking,
 *    checksums, and semantic validation create multiple barriers against
 *    data corruption, ensuring reliable operation even in electrically
 *    noisy environments.
 *
 * @author json-dev
 * @version 2.0
 * @date 15-06-2025
 */

#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <QTRSensors.h>

class EEPROMCalibrationManager {
public:
  /**
   * @brief System Configuration Constants
   *
   * These constants define the data format and validation parameters.
   * @var CALIBRATION_MAGIC Magic number for data validation
   * @var CALIBRATION_VERSION Format version (incremented for new structure)
   * @var DEFAULT_EEPROM_SIZE Realistic default based on ESP32 capabilities
   * @var DEFAULT_START_ADDRESS Standard start address for calibration data
   * @var MAX_SENSORS Optimized for typical line following robots
   */
  static const uint16_t CALIBRATION_MAGIC = 0xCAFE;
  static const uint8_t CALIBRATION_VERSION = 2;
  static const uint16_t DEFAULT_EEPROM_SIZE = 64;
  static const uint16_t DEFAULT_START_ADDRESS = 0;
  static const uint8_t MAX_SENSORS = 8;

  /**
   * @brief Comprehensive Error Code System
   *
   * These error codes represent every failure mode we discovered during
   * development and testing. Each code provides specific information that
   * enables appropriate recovery strategies and debugging approaches.
   *
   * The error codes are organized by category to help users understand
   * the type of problem they're dealing with and choose appropriate solutions.
   *
   * @enum ErrorCode
   * @var SUCCESS Operation completed successfully
   * @var EEPROM_NOT_READY EEPROM system not initialized at system level
   * @var INVALID_SENSOR_COUNT Sensor count outside valid range (1-8)
   * @var INSUFFICIENT_SPACE Not enough EEPROM space for calibration data
   * @var NO_VALID_DATA No meaningful calibration data to save
   * @var MAGIC_NUMBER_MISMATCH Stored data doesn't have valid signature
   * @var VERSION_MISMATCH Data format version incompatible
   * @var SENSOR_COUNT_MISMATCH Stored sensor count doesn't match hardware
   * @var CHECKSUM_FAILED Data corruption detected
   * @var INVALID_CALIBRATION_RANGE Calibration values don't make sense
   * @var ADC_RANGE_EXCEEDED Values exceed ESP32 ADC capabilities
   * @var EEPROM_WRITE_FAILED Failed to write data to EEPROM
   * @var EEPROM_COMMIT_FAILED Failed to commit changes to flash
   * @var VERIFICATION_FAILED Written data doesn't match intended data
   * @var NULL_POINTER_ERROR Internal programming error detected
   */
  enum class ErrorCode {
    SUCCESS = 0, ///< Operation completed successfully

    // Initialization and Configuration Errors
    EEPROM_NOT_READY,
    INVALID_SENSOR_COUNT,
    INSUFFICIENT_SPACE,

    // Data Validation Errors
    NO_VALID_DATA,
    MAGIC_NUMBER_MISMATCH,
    VERSION_MISMATCH,
    SENSOR_COUNT_MISMATCH,
    CHECKSUM_FAILED,
    INVALID_CALIBRATION_RANGE,
    ADC_RANGE_EXCEEDED,

    // Storage Operation Errors
    EEPROM_WRITE_FAILED,
    EEPROM_COMMIT_FAILED,
    VERIFICATION_FAILED,

    // Programming Errors
    NULL_POINTER_ERROR
  };

private:
/**
 * @brief Memory-Optimized Calibration Data Structure
 *
 * At exactly 40 bytes, it fits comfortably within 64-byte EEPROM allocations
 * while providing all necessary functionality for 8-sensor line following robots.
 *
 * Memory Layout Analysis (40 bytes total):
 * - Offset 0-1:   Magic number (2 bytes) - quick validation
 * - Offset 2:     Version (1 byte) - format compatibility
 * - Offset 3:     Sensor count (1 byte) - hardware validation
 * - Offset 4-19:  Minimum values (16 bytes: 8 sensors × 2 bytes)
 * - Offset 20-35: Maximum values (16 bytes: 8 sensors × 2 bytes)
 * - Offset 36-39: Checksum (4 bytes) - data integrity
 *
 * The #pragma pack directives ensure this exact layout across all
 * compilers and platforms, preventing the structure padding issues
 * @var magic Data validation magic number (0xCAFE)
 * @var version Data format version for compatibility
 * @var sensorCount Number of sensors (must be 8 for this version)
 * @var minimum Minimum calibration values per sensor
 * @var maximum Maximum calibration values per sensor
 * @var checksum Data integrity verification checksum
 */
#pragma pack(push, 1) // Force byte-perfect packing

  struct CalibrationData {
    uint16_t magic;                 
    uint8_t version;                
    uint8_t sensorCount;            
    uint16_t minimum[MAX_SENSORS];  
    uint16_t maximum[MAX_SENSORS];  
    uint32_t checksum;              
  } __attribute__((packed));       // Additional packing enforcement for GCC

#pragma pack(pop) // Restore default packing

  // Instance State Variables - The Manager's Internal Configuration
  uint8_t sensorCount_;   ///< Number of sensors this instance manages
  uint16_t eepromSize_;   ///< Available EEPROM space (set by system level)
  uint16_t startAddress_; ///< EEPROM start address for calibration data
  bool debugEnabled_;     ///< Enable detailed diagnostic output
  bool initialized_;      ///< Whether manager is ready for use
  ErrorCode lastError_;   ///< Most recent error for diagnostic purposes

  /**
   * @brief Calculate Data Integrity Checksum
   *
   * This method implements a robust checksum algorithm designed specifically
   * for embedded systems. It uses bit rotation to ensure excellent error
   * detection while remaining computationally efficient for microcontrollers.
   *
   * The algorithm processes all data fields using addition combined with
   * bit rotation. This ensures that even single-bit changes produce
   * dramatically different checksum values, providing reliable corruption
   * detection while being fast enough for real-time embedded use.
   *
   * @param data Pointer to calibration data structure
   * @return uint32_t Calculated checksum value
   */
  uint32_t calculateChecksum(const CalibrationData *data) const;

  /**
   * @brief Multi-Layer Data Validation System
   *
   * This method implements the comprehensive validation strategy we developed
   * through debugging real-world data corruption scenarios. Each layer
   * catches different types of problems, creating a robust defense against
   * data integrity issues.
   *
   * Validation Layers:
   * 1. Magic number check - detects completely wrong data
   * 2. Version compatibility - handles format evolution
   * 3. Hardware compatibility - ensures sensor count matches
   * 4. Data integrity - checksum verification
   * 5. Semantic validation - ensures values make physical sense
   *
   * This layered approach ensures that only completely validated,
   * compatible data is ever used by the calibration system.
   *
   * @param data Pointer to calibration data structure to validate
   * @return ErrorCode indicating specific validation result
   */
  ErrorCode validateCalibrationData(const CalibrationData *data) const;

  /**
   * @brief Centralized Debug Output with Memory Efficiency
   *
   * This method provides consistent debug output while using the F() macro
   * to store format strings in flash memory rather than precious RAM.
   * This memory efficiency consideration is crucial in embedded systems.
   *
   * @param message Debug message to output (only if debugging enabled)
   */
  void debugPrint(const String &message) const;

  /**
   * @brief Internal Data Loading with Comprehensive Validation
   *
   * This helper method handles the low-level mechanics of reading calibration
   * data from EEPROM and validating it thoroughly. It's designed for reuse
   * across multiple public methods that need access to stored data.
   *
   * The separation of data loading from data application enables non-destructive
   * operations like hasValidCalibration() and displayStoredCalibration()
   * that can check data validity without affecting sensor configurations.
   *
   * @param data Pointer to structure to fill with loaded data
   * @return ErrorCode indicating success or specific failure reason
   */
  ErrorCode loadCalibrationData(CalibrationData *data);

  /**
   * @brief Formatted Data Display for Debugging and Verification
   *
   * This method provides consistent, detailed formatting of calibration data
   * that's useful across multiple contexts including debugging, verification,
   * and diagnostic reporting. Centralizing the display logic ensures
   * consistency and makes it easy to enhance the output format.
   *
   * @param data Pointer to calibration data structure to display
   */
  void displayCalibrationData(const CalibrationData *data) const;

public:
  /**
   * @brief Constructor for System-Level Resource Management
   *
   * This constructor embodies the cooperative resource management approach. Instead of trying to manage
   * EEPROM initialization independently, it works with system-level EEPROM
   * initialization, preventing resource conflicts and ensuring predictable
   * behavior.
   *
   * The constructor validates that EEPROM is already initialized and accessible,
   * then configures itself to work within the established system environment.
   * This approach is more robust and reflects professional embedded systems
   * architecture where shared resources are managed centrally.
   *
   * Initialization Strategy:
   * 1. Validate input parameters for correctness
   * 2. Calculate required storage space
   * 3. Test that EEPROM is accessible and working
   * 4. Configure internal state for operation
   * 5. Report status through comprehensive error codes
   *
   * @param sensorCount Number of sensors in calibration array (1-8)
   * @param debugEnabled Enable detailed diagnostic output
   * @param eepromSize Total EEPROM space available (from system initialization)
   * @param startAddress EEPROM start address for calibration data
   */
  EEPROMCalibrationManager(uint8_t sensorCount,
                           bool debugEnabled = false,
                           uint16_t eepromSize = DEFAULT_EEPROM_SIZE,
                           uint16_t startAddress = DEFAULT_START_ADDRESS);

  /**
   * @brief Destructor with Diagnostic Output
   *
   * The destructor provides a clean shutdown point and can report final
   * status if debugging is enabled. While cleanup is minimal for the
   * current implementation, this provides a foundation for future
   * enhancements that might require explicit resource cleanup.
   */
  ~EEPROMCalibrationManager();

  /**
   * @brief Check Manager Initialization Status
   *
   * This method allows calling code to verify that the calibration manager
   * is ready for use. It's essential to check this after construction since
   * EEPROM accessibility can vary depending on system conditions.
   *
   * @return bool true if manager is initialized and ready for use
   */
  bool isInitialized() const;

  /**
   * @brief Get Detailed Error Information
   *
   * When any operation fails, this method provides specific information
   * about what went wrong. The detailed error codes enable intelligent
   * recovery strategies and meaningful user feedback.
   *
   * @return ErrorCode describing the most recent error condition
   */
  ErrorCode getLastError() const;

  /**
   * @brief Save Calibration Data with Transactional Integrity
   *
   * This method performs a complete, atomic save operation that includes
   * comprehensive validation, data integrity protection, and verification.
   * Either the entire operation succeeds completely, or it fails cleanly
   * with detailed error information.
   *
   * The save process includes:
   * - Input validation to ensure meaningful calibration data
   * - Data structure preparation with metadata and checksum
   * - Atomic write operation to EEPROM
   * - Read-back verification to confirm data integrity
   * - Comprehensive error reporting for any failure mode
   *
   * @param qtr Reference to QTRSensors object containing calibration data
   * @return bool true if save operation completed successfully
   */
  bool saveCalibration(const QTRSensors &qtr);

  /**
   * @brief Load and Apply Calibration Data Safely
   *
   * This method loads stored calibration data and applies it to a QTRSensors
   * object only after thorough validation. The multi-layer validation ensures
   * that only completely compatible, uncorrupted data is applied to sensors.
   *
   * The loading process includes:
   * - Data retrieval from EEPROM
   * - Comprehensive validation through all layers
   * - Safe application to sensor object
   * - Detailed error reporting for troubleshooting
   *
   * @param qtr Reference to QTRSensors object to receive calibration
   * @return bool true if load and application succeeded
   */
  bool loadCalibration(QTRSensors &qtr);

  /**
   * @brief Non-Destructive Calibration Data Check
   *
   * This method performs a safe check for valid calibration data without
   * modifying any sensor configurations. It's perfect for startup logic
   * and user interface decisions about whether calibration is needed.
   *
   * @return bool true if valid, compatible calibration data exists
   */
  bool hasValidCalibration();

  /**
   * @brief Secure Calibration Data Erasure
   *
   * This method completely removes calibration data from EEPROM using
   * secure overwriting techniques. It ensures no recoverable remnants
   * remain, making it suitable for factory reset scenarios.
   *
   * @return bool true if clear operation completed successfully
   */
  bool clearCalibration();

  /**
   * @brief Comprehensive Diagnostic Data Display
   *
   * This method provides detailed information about stored calibration data
   * including technical metadata and practical calibration quality metrics.
   * It's designed to be safe to call anytime and provides valuable insights
   * for debugging and system verification.
   */
  void displayStoredCalibration();

  /**
   * @brief System Status and Diagnostic Report
   *
   * This method generates a comprehensive report about the calibration
   * manager's current state, EEPROM accessibility, and stored data status.
   * It's invaluable for debugging initialization issues and understanding
   * system configuration.
   */
  void reportSystemStatus();

  /**
   * @brief Convert Error Codes to Human-Readable Descriptions
   *
   * This method translates technical error codes into descriptive explanations
   * that are suitable for user interfaces and debugging output. Each
   * description provides specific guidance about likely causes and solutions.
   *
   * @param error Error code to translate into human-readable form
   * @return String containing detailed error description and guidance
   */
  String getErrorDescription(ErrorCode error) const;

  /**
   * @brief Runtime Debug Control
   *
   * This method allows dynamic control of debug output, enabling debugging
   * to be turned on when problems occur and off during normal operation
   * to reduce output clutter and improve performance.
   *
   * @param enabled true to enable detailed debug output
   */
  void setDebugEnabled(bool enabled);

  /**
   * @brief Get Current Sensor Configuration
   *
   * @return uint8_t Number of sensors this manager is configured for
   */
  uint8_t getSensorCount() const;

  /**
   * @brief Calculate Storage Requirements for Planning
   *
   * This static utility method calculates the exact EEPROM space needed
   * for different sensor configurations. It's useful for system-level
   * EEPROM layout planning and capacity verification.
   *
   * @param sensorCount Number of sensors to calculate storage for
   * @return uint16_t Required EEPROM bytes for the specified configuration
   */
  static uint16_t calculateStorageSize(uint8_t sensorCount);
};

// ============================
// INLINE METHOD IMPLEMENTATIONS
// ============================

/**
 * @brief High-Performance Inline Accessors
 *
 * These methods are implemented inline because they're simple accessors that
 * benefit significantly from compiler optimization. Inlining eliminates
 * function call overhead, making these operations as fast as direct variable
 * access while maintaining proper encapsulation.
 *
 * This is a standard pattern in professional C++ for frequently-called
 * accessor methods that would otherwise create unnecessary performance overhead.
 */

inline bool EEPROMCalibrationManager::isInitialized() const {
  return initialized_;
}

inline EEPROMCalibrationManager::ErrorCode EEPROMCalibrationManager::getLastError() const {
  return lastError_;
}

inline uint8_t EEPROMCalibrationManager::getSensorCount() const {
  return sensorCount_;
}

inline void EEPROMCalibrationManager::setDebugEnabled(bool enabled) {
  debugEnabled_ = enabled;
  if (enabled) {
    debugPrint(F("Debug output enabled for calibration manager"));
  }
}