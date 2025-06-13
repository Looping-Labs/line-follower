/**
 * @file EEPROMCalibrationManager.cpp
 * @brief Implementation of EEPROM Calibration Data Manager
 *
 * This implementation demonstrates advanced embedded systems programming
 * techniques including defensive programming, comprehensive error handling,
 * data validation, and resource management following RAII principles.
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

  // Input validation - catch configuration errors early
  // This prevents mysterious failures later in execution
  if (sensorCount == 0 || sensorCount > MAX_SENSORS) {
    lastError_ = ErrorCode::INVALID_SENSOR_COUNT;
    debugPrint(F("ERROR: Invalid sensor count. Must be 1-16."));
    return; // Exit constructor - object will be in uninitialized state
  }

  // Verify we have enough EEPROM space for our data structure
  // This calculation ensures we don't accidentally overwrite other data
  uint16_t requiredSize = calculateStorageSize(sensorCount);
  if (startAddress + requiredSize > eepromSize) {
    lastError_ = ErrorCode::EEPROM_INIT_FAILED;
    debugPrint(F("ERROR: Insufficient EEPROM space for calibration data"));
    return;
  }

  // Initialize EEPROM subsystem
  // This is the critical step that determines if the object will be usable
  if (!EEPROM.begin(eepromSize_)) {
    lastError_ = ErrorCode::EEPROM_INIT_FAILED;
    debugPrint(F("ERROR: Failed to initialize EEPROM"));
    return;
  }

  // If we reach this point, initialization was successful
  initialized_ = true;
  lastError_ = ErrorCode::SUCCESS;

  if (debugEnabled_) {
    Serial.print(F("EEPROMCalibrationManager: Initialized for "));
    Serial.print(sensorCount_);
    Serial.print(F(" sensors, "));
    Serial.print(requiredSize);
    Serial.print(F(" bytes at address "));
    Serial.println(startAddress_);
  }
}

EEPROMCalibrationManager::~EEPROMCalibrationManager() {
  // The Arduino EEPROM library doesn't require explicit cleanup,
  // but this destructor provides a place for future cleanup code
  // if we later switch to a different storage mechanism

  if (debugEnabled_ && initialized_) {
    debugPrint(F("EEPROMCalibrationManager: Destructor called"));
  }
}

// ============================
// PUBLIC INTERFACE METHODS
// ============================

bool EEPROMCalibrationManager::saveCalibration(const QTRSensors &qtr) {
  // Defensive programming: always check initialization state first
  if (!initialized_) {
    lastError_ = ErrorCode::EEPROM_INIT_FAILED;
    return false;
  }

  debugPrint(F("=== SAVING CALIBRATION TO EEPROM ==="));

  // Step 1: Validate that we have meaningful calibration data
  // This prevents saving garbage data that would cause problems later
  bool hasValidData = false;
  for (uint8_t i = 0; i < sensorCount_; i++) {
    // Check that calibration actually happened (min != max)
    // and that values are in reasonable ranges for ESP32 ADC
    if (qtr.calibrationOn.minimum[i] < qtr.calibrationOn.maximum[i] &&
        qtr.calibrationOn.maximum[i] <= 4095) { // ESP32 12-bit ADC maximum
      hasValidData = true;
      break; // Found at least one valid sensor
    }
  }

  if (!hasValidData) {
    lastError_ = ErrorCode::NO_VALID_DATA;
    debugPrint(F("ERROR: No valid calibration data to save"));
    return false;
  }

  // Step 2: Prepare calibration data structure
  // We're building the exact structure that will be stored in EEPROM
  CalibrationData calData = {}; // Zero-initialize entire structure
  calData.magic = CALIBRATION_MAGIC;
  calData.version = CALIBRATION_VERSION;
  calData.sensorCount = sensorCount_;

  // Copy calibration arrays from QTR library to our storage structure
  // We copy all sensor data, but only the first sensorCount_ entries are valid
  for (uint8_t i = 0; i < sensorCount_; i++) {
    calData.minimum[i] = qtr.calibrationOn.minimum[i];
    calData.maximum[i] = qtr.calibrationOn.maximum[i];
  }

  // Zero out unused sensor slots to ensure consistent checksum calculation
  for (uint8_t i = sensorCount_; i < MAX_SENSORS; i++) {
    calData.minimum[i] = 0;
    calData.maximum[i] = 0;
  }

  // Step 3: Calculate checksum for data integrity verification
  // This must be done AFTER all other fields are set
  calData.checksum = calculateChecksum(&calData);

  // Step 4: Write data to EEPROM
  // We treat our structure as an array of bytes for storage
  const uint8_t *dataBytes = reinterpret_cast<const uint8_t *>(&calData);
  for (size_t i = 0; i < sizeof(CalibrationData); i++) {
    EEPROM.write(startAddress_ + i, dataBytes[i]);
  }

  // Step 5: Commit changes to permanent storage
  // This is the point where data actually gets written to flash memory
  if (!EEPROM.commit()) {
    lastError_ = ErrorCode::EEPROM_COMMIT_FAILED;
    debugPrint(F("ERROR: Failed to commit EEPROM changes"));
    return false;
  }

  // Step 6: Verification - read back and compare
  // This extra step catches write failures and ensures data integrity
  CalibrationData verifyData;
  ErrorCode loadResult = loadCalibrationData(&verifyData);
  if (loadResult != ErrorCode::SUCCESS) {
    lastError_ = ErrorCode::VERIFICATION_FAILED;
    debugPrint(F("ERROR: Failed to verify written data"));
    return false;
  }

  // Compare checksums to ensure data was written correctly
  if (verifyData.checksum != calData.checksum) {
    lastError_ = ErrorCode::VERIFICATION_FAILED;
    debugPrint(F("ERROR: Verification failed - checksum mismatch"));
    return false;
  }

  // Success! Update status and provide user feedback
  lastError_ = ErrorCode::SUCCESS;
  if (debugEnabled_) {
    Serial.println(F("✓ Calibration saved successfully"));
    Serial.print(F("  Data size: "));
    Serial.print(sizeof(CalibrationData));
    Serial.println(F(" bytes"));
    Serial.print(F("  Checksum: 0x"));
    Serial.println(calData.checksum, HEX);
  }

  return true;
}

bool EEPROMCalibrationManager::loadCalibration(QTRSensors &qtr) {
  if (!initialized_) {
    lastError_ = ErrorCode::EEPROM_INIT_FAILED;
    return false;
  }

  debugPrint(F("=== LOADING CALIBRATION FROM EEPROM ==="));

  // Load and validate calibration data
  CalibrationData calData;
  ErrorCode result = loadCalibrationData(&calData);

  if (result != ErrorCode::SUCCESS) {
    lastError_ = result;
    return false;
  }

  // Apply validated data to QTR sensor library
  // At this point we know the data is valid, so it's safe to use
  for (uint8_t i = 0; i < sensorCount_; i++) {
    qtr.calibrationOn.minimum[i] = calData.minimum[i];
    qtr.calibrationOn.maximum[i] = calData.maximum[i];
  }

  lastError_ = ErrorCode::SUCCESS;
  debugPrint(F("✓ Calibration loaded and applied to sensors"));

  if (debugEnabled_) {
    displayCalibrationData(&calData);
  }

  return true;
}

bool EEPROMCalibrationManager::hasValidCalibration() {
  if (!initialized_) {
    return false;
  }

  // Perform a non-destructive check for valid data
  // This method doesn't modify anything, just validates
  CalibrationData calData;
  ErrorCode result = loadCalibrationData(&calData);

  return (result == ErrorCode::SUCCESS);
}

bool EEPROMCalibrationManager::clearCalibration() {
  if (!initialized_) {
    lastError_ = ErrorCode::EEPROM_INIT_FAILED;
    return false;
  }

  debugPrint(F("=== CLEARING CALIBRATION FROM EEPROM ==="));

  // Overwrite the entire calibration data area with zeros
  // This ensures no remnants of old data remain
  for (size_t i = 0; i < sizeof(CalibrationData); i++) {
    EEPROM.write(startAddress_ + i, 0);
  }

  // Commit the changes to make them permanent
  if (!EEPROM.commit()) {
    lastError_ = ErrorCode::EEPROM_COMMIT_FAILED;
    debugPrint(F("ERROR: Failed to commit EEPROM clear operation"));
    return false;
  }

  lastError_ = ErrorCode::SUCCESS;
  debugPrint(F("✓ Calibration data cleared from EEPROM"));
  return true;
}

void EEPROMCalibrationManager::displayStoredCalibration() {
  if (!initialized_) {
    Serial.println(F("ERROR: Manager not initialized"));
    return;
  }

  Serial.println(F("=== STORED CALIBRATION DATA ==="));

  CalibrationData calData;
  ErrorCode result = loadCalibrationData(&calData);

  if (result != ErrorCode::SUCCESS) {
    Serial.print(F("No valid calibration data: "));
    Serial.println(getErrorDescription(result));
    return;
  }

  displayCalibrationData(&calData);
}

// ============================
// PRIVATE HELPER METHODS
// ============================

EEPROMCalibrationManager::ErrorCode EEPROMCalibrationManager::loadCalibrationData(CalibrationData *data) {
  // This private method handles the actual EEPROM reading and validation
  // It's separate from the public loadCalibration method to enable reuse
  // in verification and display operations

  if (data == nullptr) {
    return ErrorCode::NULL_POINTER_ERROR;
  }

  // Read raw data from EEPROM
  uint8_t *dataBytes = reinterpret_cast<uint8_t *>(data);
  for (size_t i = 0; i < sizeof(CalibrationData); i++) {
    dataBytes[i] = EEPROM.read(startAddress_ + i);
  }

  // Validate the loaded data using our comprehensive validation method
  return validateCalibrationData(data);
}

EEPROMCalibrationManager::ErrorCode EEPROMCalibrationManager::validateCalibrationData(const CalibrationData *data) const {
  // This method implements the multi-layer validation strategy
  // Each check catches different types of problems

  if (data == nullptr) {
    return ErrorCode::NULL_POINTER_ERROR;
  }

  // Layer 1: Magic number validation
  // This catches completely invalid data (uninitialized EEPROM, wrong data type)
  if (data->magic != CALIBRATION_MAGIC) {
    if (debugEnabled_) {
      Serial.print(F("Magic number mismatch: found 0x"));
      Serial.print(data->magic, HEX);
      Serial.print(F(", expected 0x"));
      Serial.println(CALIBRATION_MAGIC, HEX);
    }
    return ErrorCode::MAGIC_NUMBER_MISMATCH;
  }

  // Layer 2: Version compatibility check
  // This enables graceful handling of data format changes
  if (data->version != CALIBRATION_VERSION) {
    if (debugEnabled_) {
      Serial.print(F("Version mismatch: found "));
      Serial.print(data->version);
      Serial.print(F(", expected "));
      Serial.println(CALIBRATION_VERSION);
    }
    return ErrorCode::VERSION_MISMATCH;
  }

  // Layer 3: Sensor count validation
  // This ensures the stored data matches our current hardware configuration
  if (data->sensorCount != sensorCount_) {
    if (debugEnabled_) {
      Serial.print(F("Sensor count mismatch: stored "));
      Serial.print(data->sensorCount);
      Serial.print(F(", current "));
      Serial.println(sensorCount_);
    }
    return ErrorCode::SENSOR_COUNT_MISMATCH;
  }

  // Layer 4: Checksum verification
  // This detects any corruption that occurred during storage or retrieval
  uint32_t storedChecksum = data->checksum;

  // We need to temporarily clear the checksum field to calculate the hash
  // This is safe because we're working with a copy of the data
  CalibrationData tempData = *data;
  tempData.checksum = 0;
  uint32_t calculatedChecksum = calculateChecksum(&tempData);

  if (storedChecksum != calculatedChecksum) {
    if (debugEnabled_) {
      Serial.print(F("Checksum mismatch: stored 0x"));
      Serial.print(storedChecksum, HEX);
      Serial.print(F(", calculated 0x"));
      Serial.println(calculatedChecksum, HEX);
    }
    return ErrorCode::CHECKSUM_FAILED;
  }

  // Layer 5: Semantic validation of calibration values
  // This ensures the data makes sense for our application
  for (uint8_t i = 0; i < sensorCount_; i++) {
    // Check that min < max (basic calibration requirement)
    if (data->minimum[i] >= data->maximum[i]) {
      if (debugEnabled_) {
        Serial.print(F("Invalid range for sensor "));
        Serial.print(i);
        Serial.print(F(": min="));
        Serial.print(data->minimum[i]);
        Serial.print(F(", max="));
        Serial.println(data->maximum[i]);
      }
      return ErrorCode::INVALID_CALIBRATION_RANGE;
    }

    // Check that values are within ESP32 ADC range (0-4095 for 12-bit)
    if (data->maximum[i] > 4095) {
      if (debugEnabled_) {
        Serial.print(F("Sensor "));
        Serial.print(i);
        Serial.print(F(" max value "));
        Serial.print(data->maximum[i]);
        Serial.println(F(" exceeds ADC range"));
      }
      return ErrorCode::ADC_RANGE_EXCEEDED;
    }
  }

  // If we reach this point, all validation layers passed
  return ErrorCode::SUCCESS;
}

uint32_t EEPROMCalibrationManager::calculateChecksum(const CalibrationData *data) const {
  // This checksum algorithm provides good error detection while being
  // computationally efficient for microcontroller use

  uint32_t checksum = 0;

  // Include metadata fields in checksum calculation
  // This protects these critical fields from corruption
  checksum += data->magic;
  checksum = (checksum << 1) | (checksum >> 31); // Rotate left
  checksum += data->version;
  checksum = (checksum << 1) | (checksum >> 31);
  checksum += data->sensorCount;
  checksum = (checksum << 1) | (checksum >> 31);

  // Include all sensor calibration values
  // We hash the entire array to ensure consistent behavior
  for (uint8_t i = 0; i < MAX_SENSORS; i++) {
    checksum += data->minimum[i];
    checksum = (checksum << 1) | (checksum >> 31);
    checksum += data->maximum[i];
    checksum = (checksum << 1) | (checksum >> 31);
  }

  return checksum;
}

void EEPROMCalibrationManager::debugPrint(const String &message) const {
  // Centralized debug output with consistent formatting
  if (debugEnabled_) {
    Serial.print(F("[EEPROMCalibMgr] "));
    Serial.println(message);
  }
}

void EEPROMCalibrationManager::displayCalibrationData(const CalibrationData *data) const {
  // Helper method to display calibration data in a readable format
  if (data == nullptr)
    return;

  Serial.print(F("  Version: "));
  Serial.println(data->version);
  Serial.print(F("  Sensor count: "));
  Serial.println(data->sensorCount);
  Serial.print(F("  Checksum: 0x"));
  Serial.println(data->checksum, HEX);

  Serial.print(F("  Min values: "));
  for (uint8_t i = 0; i < sensorCount_; i++) {
    Serial.print(data->minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print(F("  Max values: "));
  for (uint8_t i = 0; i < sensorCount_; i++) {
    Serial.print(data->maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
}

String EEPROMCalibrationManager::getErrorDescription(ErrorCode error) const {
  // Convert error codes to human-readable descriptions
  // This is essential for user interfaces and debugging

  switch (error) {
  case ErrorCode::SUCCESS:
    return F("Operation successful");
  case ErrorCode::EEPROM_INIT_FAILED:
    return F("EEPROM initialization failed");
  case ErrorCode::INVALID_SENSOR_COUNT:
    return F("Invalid sensor count (must be 1-16)");
  case ErrorCode::NO_VALID_DATA:
    return F("No valid calibration data to save");
  case ErrorCode::MAGIC_NUMBER_MISMATCH:
    return F("Magic number mismatch - not calibration data");
  case ErrorCode::VERSION_MISMATCH:
    return F("Data version incompatible");
  case ErrorCode::CHECKSUM_FAILED:
    return F("Data corruption detected (checksum failed)");
  case ErrorCode::SENSOR_COUNT_MISMATCH:
    return F("Stored sensor count doesn't match hardware");
  case ErrorCode::INVALID_CALIBRATION_RANGE:
    return F("Invalid calibration range (min >= max)");
  case ErrorCode::ADC_RANGE_EXCEEDED:
    return F("Calibration values exceed ADC range");
  case ErrorCode::EEPROM_WRITE_FAILED:
    return F("Failed to write to EEPROM");
  case ErrorCode::EEPROM_COMMIT_FAILED:
    return F("Failed to commit EEPROM changes");
  case ErrorCode::VERIFICATION_FAILED:
    return F("Data verification failed after write");
  case ErrorCode::NULL_POINTER_ERROR:
    return F("Null pointer error");
  default:
    return F("Unknown error");
  }
}

uint16_t EEPROMCalibrationManager::calculateStorageSize(uint8_t sensorCount) {
  // Static utility method for calculating storage requirements
  // Useful for EEPROM layout planning in complex systems

  // For now, we always allocate space for MAX_SENSORS regardless of
  // actual sensor count. This ensures consistent storage size and
  // allows for future hardware expansion without data migration.
  return sizeof(CalibrationData);
}