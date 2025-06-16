#include "EEPROMCalibrationManager.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <QTRSensors.h>

// Hardware configuration
#define D1 36
#define D2 39
#define D3 34
#define D4 35
#define D5 32
#define D6 33
#define D7 25
#define D8 26
#define CALIB_BUTTON_PIN 16
#define START_BUTTON_PIN 17
#define LED_PIN 2
#define SENSOR_COUNT 8

// EEPROM Configuration
#define EEPROM_SIZE 64
#define CALIB_START_ADDRESS 0

// Hardware arrays
const uint8_t sensorPins[SENSOR_COUNT] = {D1, D2, D3, D4, D5, D6, D7, D8};
const double sensorWeights[SENSOR_COUNT] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
uint16_t sensorValues[SENSOR_COUNT];

// Global objects
QTRSensors qtr;
EEPROMCalibrationManager *calibManager = nullptr;

// System state tracking
bool systemInitialized = false;
bool calibrationLoaded = false;
bool qtrMemoryAllocated = false;

// Interrupt handling
volatile bool calibRequested = false;
volatile bool startRequested = false;
const uint32_t DEBOUNCE_DELAY = 50;
uint32_t lastCalibInterrupt = 0;
uint32_t lastStartInterrupt = 0;

void IRAM_ATTR handleCalibInterrupt() {
  uint32_t now = millis();
  if (now - lastCalibInterrupt > DEBOUNCE_DELAY) {
    calibRequested = true;
    lastCalibInterrupt = now;
  }
}

void IRAM_ATTR handleStartInterrupt() {
  uint32_t now = millis();
  if (now - lastStartInterrupt > DEBOUNCE_DELAY) {
    startRequested = true;
    lastStartInterrupt = now;
  }
}

void initializeQTRSensors() {
  Serial.println(F("=== QTR SENSOR INITIALIZATION WITH MEMORY ALLOCATION ==="));

  // Step 1: Configure QTR library basic settings
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SENSOR_COUNT);
  Serial.println(F("✓ QTR library configured"));

  // Step 2: CRITICAL - Trigger memory allocation for calibration arrays
  // This is the key insight from the forum post: we must call calibrate()
  // at least once to ensure the calibration arrays are allocated
  Serial.println(F("Allocating QTR calibration memory..."));
  Serial.println(F("Performing minimal calibration to trigger memory allocation"));

  // Perform minimal calibration just to allocate memory
  // We don't need meaningful calibration data here - just memory allocation
  for (int i = 0; i < 10; i++) {
    qtr.calibrate(); // This triggers memory allocation on first call
    delay(10);
  }

  Serial.println(F("✓ QTR calibration arrays allocated and ready"));

  // Step 3: Verify that memory allocation worked
  // Now we can safely check that the pointers are valid
  if (qtr.calibrationOn.minimum != nullptr && qtr.calibrationOn.maximum != nullptr) {
    Serial.println(F("✓ Calibration array pointers verified as valid"));
    qtrMemoryAllocated = true;

    // Initialize arrays to safe default values
    // This ensures we have known-good starting values
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      qtr.calibrationOn.minimum[i] = 0;    // Safe minimum value
      qtr.calibrationOn.maximum[i] = 4095; // Safe maximum value (ESP32 ADC range)
    }

    Serial.println(F("✓ Calibration arrays initialized with safe default values"));

  } else {
    Serial.println(F("✗ WARNING: QTR calibration arrays still not allocated"));
    Serial.println(F("This indicates a deeper issue with QTR library initialization"));
    qtrMemoryAllocated = false;
  }

  // Display sensor pin mapping for verification
  Serial.print(F("Sensor pin mapping: "));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F("=GPIO"));
    Serial.print(sensorPins[i]);
    if (i < SENSOR_COUNT - 1)
      Serial.print(F(", "));
  }
  Serial.println();
}

/**
 * @brief Initialize All Systems with Proper Sequence
 *
 * This function implements the correct initialization sequence that respects
 * the QTR library's memory allocation requirements.
 */
bool initializeAllSystems() {
  Serial.println(F("=== COMPLETE SYSTEM INITIALIZATION ==="));

  // Phase 1: Initialize EEPROM system
  Serial.println(F("Phase 1: EEPROM System"));
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println(F("✗ EEPROM initialization failed"));
    return false;
  }
  Serial.println(F("✓ EEPROM system ready"));

  // Phase 2: Initialize QTR sensors with proper memory allocation
  Serial.println(F("Phase 2: QTR Sensors with Memory Allocation"));
  initializeQTRSensors();

  if (!qtrMemoryAllocated) {
    Serial.println(F("✗ QTR memory allocation failed"));
    return false;
  }

  // Phase 3: Initialize calibration manager
  Serial.println(F("Phase 3: Calibration Manager"));
  calibManager = new EEPROMCalibrationManager(SENSOR_COUNT, true, EEPROM_SIZE, 0);

  if (!calibManager || !calibManager->isInitialized()) {
    Serial.println(F("✗ Calibration manager initialization failed"));
    return false;
  }
  Serial.println(F("✓ Calibration manager ready"));

  // Phase 4: GPIO and interrupts
  Serial.println(F("Phase 4: GPIO and Interrupts"));
  pinMode(CALIB_BUTTON_PIN, INPUT_PULLUP);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(CALIB_BUTTON_PIN), handleCalibInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), handleStartInterrupt, FALLING);
  Serial.println(F("✓ GPIO and interrupts configured"));

  return true;
}

/**
 * @brief Safe Calibration Loading with Memory Verification
 *
 * This function implements the proper sequence for loading saved calibration
 * data, ensuring that QTR memory is allocated before attempting to write to it.
 */
bool loadSavedCalibration() {
  Serial.println(F("=== SAFE CALIBRATION LOADING ==="));

  // Verify QTR memory is allocated before attempting to load
  if (!qtrMemoryAllocated) {
    Serial.println(F("✗ Cannot load calibration: QTR memory not allocated"));
    Serial.println(F("This should have been handled during QTR initialization"));
    return false;
  }

  // Check if valid calibration data exists
  if (!calibManager->hasValidCalibration()) {
    Serial.println(F("No valid calibration data found in EEPROM"));
    Serial.println(F("This is normal for first-time use"));
    return false;
  }

  Serial.println(F("Valid calibration data found - loading..."));

  // Now it's safe to load calibration because we know:
  // 1. QTR memory is allocated (we verified this)
  // 2. Valid calibration data exists
  // 3. Calibration manager is initialized

  if (calibManager->loadCalibration(qtr)) {
    Serial.println(F("✓ Calibration loaded successfully"));

    // Display loaded calibration for verification
    Serial.println(F("Loaded calibration summary:"));
    Serial.print(F("Min values: "));
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(qtr.calibrationOn.minimum[i]);
      if (i < SENSOR_COUNT - 1)
        Serial.print(F(" "));
    }
    Serial.println();

    Serial.print(F("Max values: "));
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(qtr.calibrationOn.maximum[i]);
      if (i < SENSOR_COUNT - 1)
        Serial.print(F(" "));
    }
    Serial.println();

    calibrationLoaded = true;
    return true;

  } else {
    Serial.println(F("✗ Failed to load calibration"));
    Serial.println(calibManager->getErrorDescription(calibManager->getLastError()));
    return false;
  }
}

/**
 * @brief Perform New Calibration with Proper Memory Management
 *
 * This function performs fresh sensor calibration, ensuring that the process
 * works correctly with the QTR library's memory allocation patterns.
 */
void performCalibration() {
  Serial.println(F("\n=== SENSOR CALIBRATION WITH PROPER MEMORY MANAGEMENT ==="));

  // Verify QTR memory is ready
  if (!qtrMemoryAllocated) {
    Serial.println(F("✗ Cannot calibrate: QTR memory not allocated"));
    Serial.println(F("This indicates a system initialization problem"));
    return;
  }

  Serial.println(F("QTR memory verified ready for calibration"));
  Serial.println(F("Starting calibration sequence..."));

  // Clear any existing calibration data first
  Serial.println(F("Clearing existing EEPROM calibration data..."));
  calibManager->clearCalibration();

  // Reset calibration arrays to ensure clean start
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    qtr.calibrationOn.minimum[i] = 4095; // Will be reduced during calibration
    qtr.calibrationOn.maximum[i] = 0;    // Will be increased during calibration
  }

  Serial.println(F("Move robot over light and dark surfaces for 3 seconds..."));

  // Countdown for user preparation
  for (int i = 3; i > 0; i--) {
    Serial.print(F("Starting in "));
    Serial.println(i);
    delay(1000);
  }

  Serial.println(F("CALIBRATING NOW - Move robot over different surfaces!"));

  // Perform actual calibration
  uint32_t startTime = millis();
  const uint32_t calibrationDuration = 4000;

  while (millis() - startTime < calibrationDuration) {
    qtr.calibrate(); // This updates the min/max arrays

    // Visual feedback
    digitalWrite(LED_PIN, (millis() % 200) < 100);

    // Progress feedback
    if ((millis() - startTime) % 500 == 0) {
      Serial.print(F("."));
    }

    delay(20);
  }

  digitalWrite(LED_PIN, LOW);
  Serial.println();
  Serial.println(F("Calibration data collection complete!"));

  // Display calibration results
  Serial.println(F("Calibration results:"));
  Serial.print(F("Min: "));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    if (i < SENSOR_COUNT - 1)
      Serial.print(F(" "));
  }
  Serial.println();

  Serial.print(F("Max: "));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    if (i < SENSOR_COUNT - 1)
      Serial.print(F(" "));
  }
  Serial.println();

  // Save calibration to EEPROM
  Serial.println(F("Saving calibration to EEPROM..."));

  if (calibManager->saveCalibration(qtr)) {
    Serial.println(F("✓ SUCCESS: Calibration saved to EEPROM"));
    Serial.println(F("Calibration will persist across power cycles"));
    calibrationLoaded = true;

    // Success indication
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }

  } else {
    Serial.println(F("⚠ WARNING: Failed to save calibration"));
    Serial.println(calibManager->getErrorDescription(calibManager->getLastError()));
    Serial.println(F("Calibration will work this session but won't persist"));
    calibrationLoaded = true; // Still usable for current session
  }

  Serial.println(F("Robot is ready for line following!"));
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Initialize all systems with proper sequence
  if (!initializeAllSystems()) {
    Serial.println(F("FATAL ERROR: System initialization failed"));
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }

  systemInitialized = true;
  Serial.println(F("✓ All systems initialized successfully"));

  // Now attempt to load saved calibration (this should work without crashes)
  Serial.println(F("\n=== ATTEMPTING TO LOAD SAVED CALIBRATION ==="));

  if (loadSavedCalibration()) {
    Serial.println(F("✓ Robot ready with saved calibration"));
    Serial.println(F("Press START to begin line following"));

    // Brief success indication
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }

  } else {
    Serial.println(F("⚠ No saved calibration available"));
    Serial.println(F("Press CALIB button to calibrate sensors"));

    // Slow pulse to indicate calibration needed
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }
}

void loop() {
  static bool running = false;

  // Handle calibration button
  if (calibRequested) {
    calibRequested = false;
    running = false; 
    performCalibration();
  }

  // Handle start/stop button
  if (startRequested) {
    startRequested = false;

    if (!running) {
      if (!calibrationLoaded) {
        Serial.println(F("⚠ Cannot start: No calibration loaded"));
        Serial.println(F("Press CALIB button first"));
      } else {
        running = true;
        Serial.println(F("\n=== LINE FOLLOWING STARTED ==="));
        digitalWrite(LED_PIN, HIGH);
      }
    } else {
      running = false;
      Serial.println(F("\n=== LINE FOLLOWING STOPPED ==="));
      digitalWrite(LED_PIN, LOW);
    }
  }

  // Main line following logic
  if (running) {
    static unsigned long lastOutput = 0;

    // Read calibrated sensor values
    qtr.readLineBlack(sensorValues);

    // Calculate weighted position
    double numerator = 0;
    double denominator = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      numerator += sensorWeights[i] * sensorValues[i];
      denominator += sensorValues[i];
    }

    double position = (denominator > 0) ? (numerator / denominator) : NAN;

    // Output data every 100ms
    if (millis() - lastOutput > 100) {
      lastOutput = millis();

      Serial.print(F("Pos: "));
      if (isnan(position)) {
        Serial.print(F("NO_LINE"));
      } else {
        Serial.print(position, 2);
      }

      Serial.print(F(" | Sensors: "));
      for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        Serial.print(sensorValues[i]);
        if (i < SENSOR_COUNT - 1)
          Serial.print(F(" "));
      }
      Serial.println();
    }
  }

  delay(50);
}