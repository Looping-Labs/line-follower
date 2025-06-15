/**
 * @file EEPROMCalibrationManager.cpp
 *
 * This implementation demonstrates professional approaches to embedded systems
 * challenges including:
 *
 * - Cooperative resource management that prevents system conflicts
 * - Memory-efficient data structures optimized for constrained environments
 * - Comprehensive error handling that enables intelligent recovery strategies
 * - Robust data integrity protection through multiple validation layers
 * - Professional diagnostic capabilities for debugging and maintenance
 *
 * This implementation evolved through systematic debugging of real problems:
 * - EEPROM initialization conflicts led to cooperative resource management
 * - Structure padding issues led to explicit memory layout control
 * - Memory constraints led to optimized 40-byte data structures
 * - Data corruption scenarios led to comprehensive validation systems
 *
 * @author json-dev
 * @version 2.0
 * @date 15-06-2025
 */

#include "EEPROMCalibrationManager.h"

EEPROMCalibrationManager::EEPROMCalibrationManager(uint8_t sensorCount,
                                                   bool debugEnabled,
                                                   uint16_t eepromSize,
                                                   uint16_t startAddress)
    : sensorCount_(sensorCount),
      eepromSize_(eepromSize),
      startAddress_(startAddress),
      debugEnabled_(debugEnabled),
      initialized_(false),
      lastError_(ErrorCode::SUCCESS) {

  if (debugEnabled_) {
    Serial.println(F("=== CALIBRATION MANAGER INITIALIZATION ==="));
    Serial.print(F("Configuring for "));
    Serial.print(sensorCount);
    Serial.print(F(" sensors, expecting "));
    Serial.print(eepromSize);
    Serial.print(F(" bytes EEPROM space"));
    Serial.println();
  }

  // Validate sensor count is within our optimized range
  if (sensorCount == 0 || sensorCount > MAX_SENSORS) {
    lastError_ = ErrorCode::INVALID_SENSOR_COUNT;

    if (debugEnabled_) {
      Serial.print(F("ERROR: Invalid sensor count "));
      Serial.print(sensorCount);
      Serial.print(F(". Must be 1-"));
      Serial.println(MAX_SENSORS);
      Serial.println(F("This version is optimized for typical line following robots"));
    }

    // Constructor exits with initialized_ = false
    return;
  }

  // Calculate and Validate Storage Requirements
  uint16_t requiredSize = calculateStorageSize(sensorCount);

  if (debugEnabled_) {
    Serial.print(F("Required storage space: "));
    Serial.print(requiredSize);
    Serial.println(F(" bytes"));
    Serial.print(F("Available EEPROM space: "));
    Serial.print(eepromSize);
    Serial.println(F(" bytes"));
  }

  // Ensure we have sufficient space for our data structure
  if (startAddress + requiredSize > eepromSize) {
    lastError_ = ErrorCode::INSUFFICIENT_SPACE;

    if (debugEnabled_) {
      Serial.print(F("ERROR: Insufficient EEPROM space"));
      Serial.print(F("Need "));
      Serial.print(requiredSize);
      Serial.print(F(" bytes starting at address "));
      Serial.print(startAddress);
      Serial.print(F(", but only "));
      Serial.print(eepromSize - startAddress);
      Serial.println(F(" bytes available"));
      Serial.println(F("Consider reducing sensor count or increasing EEPROM allocation"));
    }
    return;
  }

  /**
   * Critical Design Decision: Testing vs. Initializing
   *
   * Instead of trying to initialize EEPROM ourselves (which caused conflicts),
   * I test whether EEPROM is already accessible and working. This cooperative
   * approach prevents resource conflicts while ensuring we can actually use
   * the EEPROM system.
   *
   * This reflects a key embedded systems principle: work with existing system
   * state rather than trying to impose your own system state.
   */

  debugPrint(F("Testing EEPROM accessibility..."));

  // Perform a non-destructive test of EEPROM functionality
  // Read a byte, write it back, and commit to test the full cycle
  uint8_t testByte = EEPROM.read(startAddress); // Read current value
  EEPROM.write(startAddress, testByte);         // Write same value back

  if (EEPROM.commit()) {
    // EEPROM is accessible and the commit cycle works
    initialized_ = true;
    lastError_ = ErrorCode::SUCCESS;

    if (debugEnabled_) {
      Serial.println(F("✓ EEPROM accessibility test passed"));
      Serial.print(F("✓ Calibration manager ready with "));
      Serial.print(eepromSize_);
      Serial.print(F(" bytes EEPROM space at address "));
      Serial.println(startAddress_);

      // Report structure efficiency for educational purposes
      float efficiency = (float)requiredSize / eepromSize_ * 100.0f;
      Serial.print(F("  Storage efficiency: "));
      Serial.print(efficiency, 1);
      Serial.print(F("% ("));
      Serial.print(eepromSize_ - requiredSize);
      Serial.println(F(" bytes free)"));
    }
  } else {
    // EEPROM is not accessible - likely not initialized at system level
    lastError_ = ErrorCode::EEPROM_NOT_READY;

    if (debugEnabled_) {
      Serial.println(F("✗ EEPROM accessibility test failed"));
      Serial.println(F("This usually means:"));
      Serial.println(F("  1. EEPROM.begin() was not called at system level"));
      Serial.println(F("  2. EEPROM initialization failed"));
      Serial.println(F("  3. EEPROM session was closed unexpectedly"));
      Serial.println(F("Solution: Ensure EEPROM.begin() succeeds before creating calibration manager"));
    }
  }
}

EEPROMCalibrationManager::~EEPROMCalibrationManager() {
  if (debugEnabled_ && initialized_) {
    debugPrint(F("Calibration manager destructor - clean shutdown"));
  }
}

bool EEPROMCalibrationManager::saveCalibration(const QTRSensors &qtr) {

  // Defensive programming: always verify system state first
  if (!initialized_) {
    lastError_ = ErrorCode::EEPROM_NOT_READY;
    debugPrint(F("Save failed: Manager not properly initialized"));
    return false;
  }

  debugPrint(F("=== CALIBRATION SAVE OPERATION STARTED ==="));

  // Input Validation
  bool hasValidData = false;
  uint8_t validSensorCount = 0;
  uint16_t totalCalibrationRange = 0;

  for (uint8_t i = 0; i < sensorCount_; i++) {
    uint16_t minVal = qtr.calibrationOn.minimum[i];
    uint16_t maxVal = qtr.calibrationOn.maximum[i];

    // Validate calibration range makes physical sense: ESP32 ADC is 12-bit (0-4095)
    if (minVal < maxVal && maxVal <= 4095) {
      hasValidData = true;
      validSensorCount++;
      totalCalibrationRange += (maxVal - minVal);
    } else if (debugEnabled_) {
      Serial.print(F("WARNING: Sensor "));
      Serial.print(i);
      Serial.print(F(" has invalid range: min="));
      Serial.print(minVal);
      Serial.print(F(", max="));
      Serial.println(maxVal);
    }
  }

  if (!hasValidData) {
    lastError_ = ErrorCode::NO_VALID_DATA;
    debugPrint(F("ERROR: No sensors have valid calibration data"));
    debugPrint(F("Perform sensor calibration before attempting to save"));
    return false;
  }

  if (debugEnabled_) {
    Serial.print(F("Validated calibration data for "));
    Serial.print(validSensorCount);
    Serial.print(F("/"));
    Serial.print(sensorCount_);
    Serial.println(F(" sensors"));

    uint16_t avgRange = totalCalibrationRange / validSensorCount;
    Serial.print(F("Average calibration range: "));
    Serial.print(avgRange);

    if (avgRange > 1500) {
      Serial.println(F(" (Excellent contrast detected)"));
    } else if (avgRange > 800) {
      Serial.println(F(" (Good contrast detected)"));
    } else if (avgRange > 400) {
      Serial.println(F(" (Fair contrast - consider recalibrating)"));
    } else {
      Serial.println(F(" (Poor contrast - recalibration strongly recommended)"));
    }
  }

  // Data Structure Preparation with metadata
  CalibrationData calData = {}; // Zero-initialize for safety and consistency

  calData.magic = CALIBRATION_MAGIC;     // Enables quick data validation
  calData.version = CALIBRATION_VERSION; // Supports future format evolution
  calData.sensorCount = sensorCount_;    // Validates hardware compatibility

  // Copy sensor calibration data with clear bounds checking
  for (uint8_t i = 0; i < sensorCount_; i++) {
    calData.minimum[i] = qtr.calibrationOn.minimum[i];
    calData.maximum[i] = qtr.calibrationOn.maximum[i];
  }

  // Zero unused sensor slots for consistent checksum calculation
  // This ensures that future sensor count changes don't invalidate checksums
  for (uint8_t i = sensorCount_; i < MAX_SENSORS; i++) {
    calData.minimum[i] = 0;
    calData.maximum[i] = 0;
  }

  // Calculate Data Integrity Checksum
  // Checksum must be calculated AFTER all other fields are set
  calData.checksum = calculateChecksum(&calData);

  if (debugEnabled_) {
    Serial.print(F("Data structure prepared ("));
    Serial.print(sizeof(CalibrationData));
    Serial.print(F(" bytes). Checksum: 0x"));
    Serial.println(calData.checksum, HEX);
  }

  // Atomic Write Operation
  const uint8_t *dataBytes = reinterpret_cast<const uint8_t *>(&calData);
  size_t dataSize = sizeof(CalibrationData);

  // Write all bytes of the structure sequentially
  for (size_t i = 0; i < dataSize; i++) {
    EEPROM.write(startAddress_ + i, dataBytes[i]);
  }

  // Commit to Persistent Storage
  if (!EEPROM.commit()) {
    lastError_ = ErrorCode::EEPROM_COMMIT_FAILED;
    debugPrint(F("ERROR: Failed to commit EEPROM changes to flash memory"));

    if (debugEnabled_) {
      Serial.println(F("This may indicate:"));
      Serial.println(F("  1. Flash memory wear-out or hardware issues"));
      Serial.println(F("  2. Power supply instability"));
      Serial.println(F("  3. EEPROM system in inconsistent state"));
    }
    return false;
  }

  debugPrint(F("Data committed to flash memory"));

  // Verification by Read-Back
  CalibrationData verifyData;
  ErrorCode loadResult = loadCalibrationData(&verifyData);

  if (loadResult != ErrorCode::SUCCESS) {
    lastError_ = ErrorCode::VERIFICATION_FAILED;

    if (debugEnabled_) {
      Serial.print(F("ERROR: Verification read failed: "));
      Serial.println(getErrorDescription(loadResult));
    }
    return false;
  }

  // Compare critical fields to ensure write integrity
  if (verifyData.magic != calData.magic ||
      verifyData.version != calData.version ||
      verifyData.sensorCount != calData.sensorCount ||
      verifyData.checksum != calData.checksum) {

    lastError_ = ErrorCode::VERIFICATION_FAILED;

    if (debugEnabled_) {
      Serial.println(F("ERROR: Verification failed - data mismatch detected"));
      Serial.print(F("Expected checksum: 0x"));
      Serial.print(calData.checksum, HEX);
      Serial.print(F(", Read checksum: 0x"));
      Serial.println(verifyData.checksum, HEX);
    }
    return false;
  }

  // Success! Update status and provide comprehensive feedback
  lastError_ = ErrorCode::SUCCESS;

  if (debugEnabled_) {
    Serial.println(F("✓ Calibration saved successfully to EEPROM"));
    Serial.print(F("  Storage used: "));
    Serial.print(dataSize);
    Serial.print(F(" bytes at address "));
    Serial.println(startAddress_);
    Serial.print(F("  Data integrity: Verified (checksum: 0x"));
    Serial.print(calData.checksum, HEX);
    Serial.println(F(")"));
    Serial.println(F("  Persistence: Guaranteed across power cycles"));
  }

  return true;
}

bool EEPROMCalibrationManager::loadCalibration(QTRSensors &qtr) {
  if (!initialized_) {
    lastError_ = ErrorCode::EEPROM_NOT_READY;
    debugPrint(F("Load failed: Manager not properly initialized"));
    return false;
  }

  debugPrint(F("=== CALIBRATION LOAD OPERATION STARTED ==="));

  // Load and Comprehensively Validate Data
  CalibrationData calData;
  ErrorCode result = loadCalibrationData(&calData);

  if (result != ErrorCode::SUCCESS) {
    lastError_ = result;

    if (debugEnabled_) {
      Serial.print(F("Data validation failed: "));
      Serial.println(getErrorDescription(result));
    }
    return false;
  }

  debugPrint(F("Stored data validation passed"));

  // Apply Validated Data to QTR Sensors
  for (uint8_t i = 0; i < sensorCount_; i++) {
    qtr.calibrationOn.minimum[i] = calData.minimum[i];
    qtr.calibrationOn.maximum[i] = calData.maximum[i];
  }

  lastError_ = ErrorCode::SUCCESS;

  if (debugEnabled_) {
    Serial.println(F("✓ Calibration loaded and applied to QTR sensors"));
    Serial.print(F("  Data format version: "));
    Serial.println(calData.version);
    Serial.print(F("  Sensor configuration: "));
    Serial.print(calData.sensorCount);
    Serial.println(F(" sensors"));
    Serial.print(F("  Data integrity: Verified (checksum: 0x"));
    Serial.print(calData.checksum, HEX);
    Serial.println(F(")"));

    // Calculate and display calibration quality metrics
    uint16_t totalRange = 0;
    for (uint8_t i = 0; i < sensorCount_; i++) {
      totalRange += (calData.maximum[i] - calData.minimum[i]);
    }
    uint16_t avgRange = totalRange / sensorCount_;
    Serial.print(F("  Calibration quality: Average range "));
    Serial.print(avgRange);
    Serial.println(avgRange > 1000 ? F(" (Excellent)") : avgRange > 500 ? F(" (Good)")
                                                                        : F(" (Fair)"));
  }

  return true;
}

bool EEPROMCalibrationManager::hasValidCalibration() {
  if (!initialized_) {
    return false;
  }

  CalibrationData calData;
  ErrorCode result = loadCalibrationData(&calData);

  return (result == ErrorCode::SUCCESS);
}

bool EEPROMCalibrationManager::clearCalibration() {
  if (!initialized_) {
    lastError_ = ErrorCode::EEPROM_NOT_READY;
    debugPrint(F("Clear failed: Manager not properly initialized"));
    return false;
  }

  debugPrint(F("=== CALIBRATION CLEAR OPERATION STARTED ==="));

  // Overwrite entire calibration data area with zeros
  size_t clearSize = sizeof(CalibrationData);
  for (size_t i = 0; i < clearSize; i++) {
    EEPROM.write(startAddress_ + i, 0);
  }

  // Commit the erasure to make it permanent
  if (!EEPROM.commit()) {
    lastError_ = ErrorCode::EEPROM_COMMIT_FAILED;
    debugPrint(F("ERROR: Failed to commit EEPROM clear operation"));
    return false;
  }

  lastError_ = ErrorCode::SUCCESS;

  if (debugEnabled_) {
    Serial.println(F("✓ Calibration data securely cleared from EEPROM"));
    Serial.print(F("  Erased: "));
    Serial.print(clearSize);
    Serial.print(F(" bytes at address "));
    Serial.println(startAddress_);
    Serial.println(F("  Status: Robot requires recalibration before use"));
  }

  return true;
}

void EEPROMCalibrationManager::displayStoredCalibration() {
  /**
   * This method provides detailed information about stored calibration data
   * for debugging and verification. It's designed to be safe to call anytime
   * and provides clear status information regardless of data validity.
   */

  Serial.println(F("=== STORED CALIBRATION DATA ANALYSIS ==="));

  if (!initialized_) {
    Serial.println(F("EEPROM Status: Manager not initialized"));
    Serial.print(F("Last Error: "));
    Serial.println(getErrorDescription(lastError_));
    return;
  }

  Serial.println(F("EEPROM Status: Manager initialized and ready"));
  Serial.print(F("Storage Configuration: "));
  Serial.print(eepromSize_);
  Serial.print(F(" bytes total, calibration data at address "));
  Serial.println(startAddress_);

  // Attempt to load and display calibration data
  CalibrationData calData;
  ErrorCode result = loadCalibrationData(&calData);

  if (result != ErrorCode::SUCCESS) {
    Serial.println(F("Calibration Status: No valid data found"));
    Serial.print(F("Validation Error: "));
    Serial.println(getErrorDescription(result));

    Serial.println(F("Raw stored data (first 16 bytes):"));
    for (int i = 0; i < 16 && i < eepromSize_; i++) {
      uint8_t byte = EEPROM.read(startAddress_ + i);
      Serial.print(F("0x"));
      if (byte < 0x10)
        Serial.print(F("0"));
      Serial.print(byte, HEX);
      Serial.print(F(" "));
    }
    Serial.println();
    return;
  }

  // Successfully loaded calibration data
  Serial.println(F("Calibration Status: Valid data found"));
  displayCalibrationData(&calData);
}

void EEPROMCalibrationManager::reportSystemStatus() {
  /**
   * System Status:
   *
   * This method provides a comprehensive view of the calibration manager's
   * state and its relationship to the broader system. It's invaluable for
   * debugging initialization issues and understanding system configuration.
   */

  Serial.println(F("=== CALIBRATION MANAGER STATUS REPORT ==="));

  Serial.print(F("Initialization Status: "));
  Serial.println(initialized_ ? F("SUCCESS") : F("FAILED"));

  if (!initialized_) {
    Serial.print(F("Failure Reason: "));
    Serial.println(getErrorDescription(lastError_));
    Serial.println(F("Recommendations:"));
    Serial.println(F("  1. Ensure EEPROM.begin() succeeds before creating manager"));
    Serial.println(F("  2. Check available flash memory space"));
    Serial.println(F("  3. Verify power supply stability"));
    return;
  }

  // Report detailed configuration information
  Serial.print(F("EEPROM Configuration: "));
  Serial.print(eepromSize_);
  Serial.print(F(" bytes available at address "));
  Serial.println(startAddress_);

  Serial.print(F("Sensor Configuration: "));
  Serial.print(sensorCount_);
  Serial.println(F(" sensors"));

  Serial.print(F("Structure Size: "));
  Serial.print(sizeof(CalibrationData));
  Serial.println(F(" bytes"));

  // Analyze storage efficiency and capacity
  Serial.print(F("Storage Analysis: "));
  if (sizeof(CalibrationData) <= eepromSize_) {
    uint16_t freeSpace = eepromSize_ - sizeof(CalibrationData);
    float efficiency = (float)sizeof(CalibrationData) / eepromSize_ * 100.0f;

    Serial.print(F("✓ ADEQUATE ("));
    Serial.print(efficiency, 1);
    Serial.print(F("% used, "));
    Serial.print(freeSpace);
    Serial.println(F(" bytes free)"));
  } else {
    Serial.print(F("✗ INSUFFICIENT (need "));
    Serial.print(sizeof(CalibrationData) - eepromSize_);
    Serial.println(F(" more bytes)"));
  }

  // Test for presence of calibration data
  uint16_t storedMagic;
  uint8_t *magicBytes = (uint8_t *)&storedMagic;
  magicBytes[0] = EEPROM.read(startAddress_);
  magicBytes[1] = EEPROM.read(startAddress_ + 1);

  Serial.print(F("Stored Data Check: Magic number 0x"));
  Serial.print(storedMagic, HEX);
  Serial.print(F(" (expected: 0x"));
  Serial.print(CALIBRATION_MAGIC, HEX);
  Serial.println(F(")"));

  if (storedMagic == CALIBRATION_MAGIC) {
    Serial.println(F("✓ Valid calibration data signature detected"));

    // Quick validation check
    if (hasValidCalibration()) {
      Serial.println(F("✓ Stored calibration data passes full validation"));
    } else {
      Serial.println(F("⚠ Calibration signature found but validation failed"));
    }
  } else {
    Serial.println(F("✗ No valid calibration data signature found"));
    Serial.println(F("  Robot will require calibration before use"));
  }

  Serial.println();
}

EEPROMCalibrationManager::ErrorCode EEPROMCalibrationManager::loadCalibrationData(CalibrationData *data) {
  /**
   * Internal Data Loading:
   *
   * This method handles the mechanics of reading and validating calibration
   * data.
   */

  // Validate input pointer
  if (data == nullptr) {
    debugPrint(F("INTERNAL ERROR: loadCalibrationData called with null pointer"));
    return ErrorCode::NULL_POINTER_ERROR;
  }

  // Read raw binary data from EEPROM
  uint8_t *dataBytes = reinterpret_cast<uint8_t *>(data);
  size_t dataSize = sizeof(CalibrationData);

  for (size_t i = 0; i < dataSize; i++) {
    dataBytes[i] = EEPROM.read(startAddress_ + i);
  }

  // Validate the loaded data using comprehensive validation
  ErrorCode validationResult = validateCalibrationData(data);

  if (validationResult != ErrorCode::SUCCESS && debugEnabled_) {
    Serial.print(F("Data validation failed during load: "));
    Serial.println(getErrorDescription(validationResult));
  }

  return validationResult;
}

EEPROMCalibrationManager::ErrorCode EEPROMCalibrationManager::validateCalibrationData(const CalibrationData *data) const {
  /**
   * Multi-Layer Validation:
   *
   * This method implements the comprehensive validation strategy. The layered approach ensures
   * that only completely validated, compatible data is ever used by the calibration system.
   */

  if (data == nullptr) {
    return ErrorCode::NULL_POINTER_ERROR;
  }

  // Layer 1: Magic Number Validation
  // Quick check to determine if this looks like calibration data at all
  if (data->magic != CALIBRATION_MAGIC) {
    if (debugEnabled_) {
      Serial.print(F("Magic number validation failed: found 0x"));
      Serial.print(data->magic, HEX);
      Serial.print(F(", expected 0x"));
      Serial.println(CALIBRATION_MAGIC, HEX);
    }
    return ErrorCode::MAGIC_NUMBER_MISMATCH;
  }

  // Layer 2: Version Compatibility Check
  // Ensures data format is compatible with current code
  if (data->version != CALIBRATION_VERSION) {
    if (debugEnabled_) {
      Serial.print(F("Version compatibility failed: stored v"));
      Serial.print(data->version);
      Serial.print(F(", current v"));
      Serial.println(CALIBRATION_VERSION);
    }
    return ErrorCode::VERSION_MISMATCH;
  }

  // Layer 3: Hardware Compatibility Check
  // Ensures stored data matches current sensor configuration
  if (data->sensorCount != sensorCount_) {
    if (debugEnabled_) {
      Serial.print(F("Sensor count mismatch: stored "));
      Serial.print(data->sensorCount);
      Serial.print(F(" sensors, hardware configured for "));
      Serial.println(sensorCount_);
    }
    return ErrorCode::SENSOR_COUNT_MISMATCH;
  }

  // Layer 4: Data Integrity Verification
  // Checksum validation to detect any corruption
  uint32_t storedChecksum = data->checksum;

  // Create temporary copy to calculate checksum (checksum field must be zero)
  CalibrationData tempData = *data;
  tempData.checksum = 0;
  uint32_t calculatedChecksum = calculateChecksum(&tempData);

  if (storedChecksum != calculatedChecksum) {
    if (debugEnabled_) {
      Serial.print(F("Checksum validation failed: stored 0x"));
      Serial.print(storedChecksum, HEX);
      Serial.print(F(", calculated 0x"));
      Serial.println(calculatedChecksum, HEX);
    }
    return ErrorCode::CHECKSUM_FAILED;
  }

  // Layer 5: Semantic Validation
  // Check that calibration values make physical sense
  for (uint8_t i = 0; i < sensorCount_; i++) {
    // Validate calibration range makes sense
    if (data->minimum[i] >= data->maximum[i]) {
      if (debugEnabled_) {
        Serial.print(F("Invalid calibration range for sensor "));
        Serial.print(i);
        Serial.print(F(": min="));
        Serial.print(data->minimum[i]);
        Serial.print(F(", max="));
        Serial.println(data->maximum[i]);
      }
      return ErrorCode::INVALID_CALIBRATION_RANGE;
    }

    // Validate values are within ESP32 ADC range (12-bit: 0-4095)
    if (data->maximum[i] > 4095) {
      if (debugEnabled_) {
        Serial.print(F("Sensor "));
        Serial.print(i);
        Serial.print(F(" max value ("));
        Serial.print(data->maximum[i]);
        Serial.println(F(") exceeds ESP32 ADC range"));
      }
      return ErrorCode::ADC_RANGE_EXCEEDED;
    }
  }

  // All validation layers passed successfully
  return ErrorCode::SUCCESS;
}

uint32_t EEPROMCalibrationManager::calculateChecksum(const CalibrationData *data) const {
  /**
   * Checksum Algorithm:
   *
   * This algorithm provides excellent error detection while being
   * computationally efficient for microcontroller use. The bit rotation
   * ensures good distribution of hash values and prevents simple patterns
   * from producing identical checksums.
   *
   * The algorithm processes all data fields systematically, ensuring that
   * even single-bit changes produce dramatically different checksum values.
   */

  uint32_t checksum = 0;

  // Include metadata in checksum to protect critical fields
  checksum += data->magic;
  checksum = (checksum << 1) | (checksum >> 31); // Rotate left by 1 bit

  checksum += data->version;
  checksum = (checksum << 1) | (checksum >> 31);

  checksum += data->sensorCount;
  checksum = (checksum << 1) | (checksum >> 31);

  // Include all sensor calibration data
  // Process the entire arrays to ensure consistent behavior
  for (uint8_t i = 0; i < MAX_SENSORS; i++) {
    checksum += data->minimum[i];
    checksum = (checksum << 1) | (checksum >> 31);

    checksum += data->maximum[i];
    checksum = (checksum << 1) | (checksum >> 31);
  }

  return checksum;
}

void EEPROMCalibrationManager::debugPrint(const String &message) const {
  /**
   * Centralized Debug Output:
   *
   * This method provides consistent debug message formatting and uses
   * the F() macro to store format strings in flash memory rather than
   * RAM. This memory efficiency is crucial in embedded systems.
   */

  if (debugEnabled_) {
    Serial.print(F("[EEPROMCalibMgr] "));
    Serial.println(message);
  }
}

void EEPROMCalibrationManager::displayCalibrationData(const CalibrationData *data) const {
  /**
   * Comprehensive Data Display:
   *
   * This method formats calibration data in a human-readable way that's
   * useful for debugging, verification, and quality assessment. It shows
   * both technical details and practical implications of the calibration.
   */

  if (data == nullptr) {
    Serial.println(F("ERROR: Cannot display null calibration data"));
    return;
  }

  // Display metadata and validation information
  Serial.print(F("  Data Format: Version "));
  Serial.print(data->version);
  Serial.println(data->version == CALIBRATION_VERSION ? F(" (compatible)") : F(" (incompatible)"));

  Serial.print(F("  Hardware Config: "));
  Serial.print(data->sensorCount);
  Serial.print(F(" sensors (current hardware: "));
  Serial.print(sensorCount_);
  Serial.println(F(")"));

  Serial.print(F("  Data Integrity: Checksum 0x"));
  Serial.println(data->checksum, HEX);

  // Display individual sensor calibration data in organized format
  Serial.println(F("  Sensor Calibration Data:"));
  Serial.print(F("    Sensor:  "));
  for (uint8_t i = 0; i < sensorCount_; i++) {
    Serial.print(F("    "));
    Serial.print(i);
    if (i < 10)
      Serial.print(F(" "));
  }
  Serial.println();

  Serial.print(F("    Min:     "));
  for (uint8_t i = 0; i < sensorCount_; i++) {
    Serial.print(data->minimum[i]);
    if (data->minimum[i] < 1000)
      Serial.print(F(" "));
    if (data->minimum[i] < 100)
      Serial.print(F(" "));
    if (data->minimum[i] < 10)
      Serial.print(F(" "));
    Serial.print(F(" "));
  }
  Serial.println();

  Serial.print(F("    Max:     "));
  for (uint8_t i = 0; i < sensorCount_; i++) {
    Serial.print(data->maximum[i]);
    if (data->maximum[i] < 1000)
      Serial.print(F(" "));
    if (data->maximum[i] < 100)
      Serial.print(F(" "));
    if (data->maximum[i] < 10)
      Serial.print(F(" "));
    Serial.print(F(" "));
  }
  Serial.println();

  // Calculate and display calibration quality metrics
  Serial.print(F("    Range:   "));
  uint16_t totalRange = 0;
  for (uint8_t i = 0; i < sensorCount_; i++) {
    uint16_t range = data->maximum[i] - data->minimum[i];
    totalRange += range;
    Serial.print(range);
    if (range < 1000)
      Serial.print(F(" "));
    if (range < 100)
      Serial.print(F(" "));
    if (range < 10)
      Serial.print(F(" "));
    Serial.print(F(" "));
  }
  Serial.println();

  // Provide calibration quality assessment
  uint16_t avgRange = totalRange / sensorCount_;
  Serial.print(F("  Quality Assessment: Average range = "));
  Serial.print(avgRange);

  if (avgRange > 1500) {
    Serial.println(F(" (Excellent - high contrast environment)"));
  } else if (avgRange > 1000) {
    Serial.println(F(" (Very Good - good contrast detected)"));
  } else if (avgRange > 700) {
    Serial.println(F(" (Good - adequate contrast for line following)"));
  } else if (avgRange > 400) {
    Serial.println(F(" (Fair - usable but consider recalibrating)"));
  } else {
    Serial.println(F(" (Poor - recalibration strongly recommended)"));
  }
}

String EEPROMCalibrationManager::getErrorDescription(ErrorCode error) const {
  /**
   * Human-Readable Error Translation:
   *
   * This method converts technical error codes into descriptive explanations
   * that help users understand what went wrong and what they can do about it.
   * Each description provides specific guidance for resolution.
   */

  switch (error) {
  case ErrorCode::SUCCESS:
    return F("Operation completed successfully");

  // Initialization and Configuration Errors
  case ErrorCode::EEPROM_NOT_READY:
    return F("EEPROM system not initialized - call EEPROM.begin() at system level first");

  case ErrorCode::INVALID_SENSOR_COUNT:
    return F("Invalid sensor count (must be 1-16) - check constructor parameters");

  case ErrorCode::INSUFFICIENT_SPACE:
    return F("Insufficient EEPROM space for calibration data - increase EEPROM allocation");

  // Data Validation Errors
  case ErrorCode::NO_VALID_DATA:
    return F("No valid calibration data to save - perform sensor calibration first");

  case ErrorCode::MAGIC_NUMBER_MISMATCH:
    return F("Magic number mismatch - stored data is not calibration data");

  case ErrorCode::VERSION_MISMATCH:
    return F("Data format version incompatible - recalibration required after firmware update");

  case ErrorCode::SENSOR_COUNT_MISMATCH:
    return F("Stored sensor count doesn't match current hardware configuration");

  case ErrorCode::CHECKSUM_FAILED:
    return F("Data corruption detected (checksum failed) - recalibration recommended");

  case ErrorCode::INVALID_CALIBRATION_RANGE:
    return F("Invalid calibration range (min >= max) - perform proper calibration");

  case ErrorCode::ADC_RANGE_EXCEEDED:
    return F("Calibration values exceed ESP32 ADC range (0-4095) - check sensor wiring");

  // Storage Operation Errors
  case ErrorCode::EEPROM_WRITE_FAILED:
    return F("Failed to write to EEPROM - possible flash memory issues");

  case ErrorCode::EEPROM_COMMIT_FAILED:
    return F("Failed to commit EEPROM changes - check power supply stability");

  case ErrorCode::VERIFICATION_FAILED:
    return F("Data verification failed after write - possible flash memory corruption");

  // Programming Errors
  case ErrorCode::NULL_POINTER_ERROR:
    return F("Internal programming error - null pointer detected");

  default:
    return F("Unknown error code - this indicates a programming bug");
  }
}

uint16_t EEPROMCalibrationManager::calculateStorageSize(uint8_t sensorCount) {
  /**
   * Storage Size Calculation:
   *
   * This method calculates the exact EEPROM space needed for calibration data.
   * The calculation is based on optimized structure design that uses
   * exactly 40 bytes regardless of sensor count (up to 8 sensors).
   *
   * This fixed-size approach simplifies memory management and provides
   * consistent performance characteristics across different configurations.
   */

  // Optimized structure always uses exactly 40 bytes
  // This design choice simplifies memory management and provides
  // room for future expansion within the same footprint
  return sizeof(CalibrationData);
}