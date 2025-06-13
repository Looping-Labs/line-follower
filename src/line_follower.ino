#include <QTRSensors.h>
#include <Arduino.h>  

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

const uint8_t sensorPins[SENSOR_COUNT] = { D1, D2, D3, D4, D5, D6, D7, D8 };
const double sensorWeights[SENSOR_COUNT] = { -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5 };
uint16_t sensorValues[SENSOR_COUNT];

// Debounce and interuptions variables
const unsigned long DEBOUNCE_DELAY = 50;
volatile bool calibRequested = false;
volatile bool startRequested = false;
unsigned long lastCalibInterrupt = 0;
unsigned long lastStartInterrupt = 0;

QTRSensors qtr;

void IRAM_ATTR handleCalibInterrupt() {
  unsigned long now = millis();
  if (now - lastCalibInterrupt > DEBOUNCE_DELAY) {
    calibRequested = true;
    lastCalibInterrupt = now;
  }
}

void IRAM_ATTR handleStartInterrupt() {
  unsigned long now = millis();
  if (now - lastStartInterrupt > DEBOUNCE_DELAY) {
    startRequested = true;
    lastStartInterrupt = now;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
 
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SENSOR_COUNT);

  pinMode(CALIB_BUTTON_PIN, INPUT_PULLUP);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(CALIB_BUTTON_PIN), handleCalibInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), handleStartInterrupt, FALLING);

  Serial.println(F("Listo. Pulsa CALIB (PIN16) o START (PIN17)."));
}

void loop() {
  static bool running = false;

  if (calibRequested) {
    calibRequested = false;
    Serial.println(F("=== CALIBRACIÓN INICIADA ==="));
    digitalWrite(LED_PIN, HIGH);
    for (uint16_t i = 0; i < 400; i++) {
      qtr.calibrate();
      delay(5);
    }
    digitalWrite(LED_PIN, LOW);
    Serial.print(F("Min: "));
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.print(F("Max: "));
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println(F("Pulsa START (PIN17) para iniciar seguimiento."));
  }

  if (startRequested) {
    startRequested = false;
    running = true;
    Serial.println(F("=== SEGUIMIENTO INICIADO ==="));
  }

  if (running) {
    // Leer sensores calibrados
    qtr.readLineBlack(sensorValues);

    // Calcular posición con pesos negativos/positivos
    double num = 0;
    double den = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      num += sensorWeights[i] * sensorValues[i];
      den += sensorValues[i];
    }
    double position = (den > 0) ? (num / den) : NAN;  // NAN si no hay línea

    Serial.print(F("Position: "));
    if (isnan(position)) Serial.print(F("NaN")); else Serial.print(position, 2);

    Serial.print(F(" | Values: "));
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(sensorValues[i]); Serial.print(' ');
    }
    Serial.println();
  }

  delay(50);
}
