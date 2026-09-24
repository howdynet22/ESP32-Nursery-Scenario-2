/**
 * @file sketch.ino
 * @brief ESP32 Scenario 2 micro-climate nursery controller.
 * @details Fault > manual > automatic. All recurring work is scheduled with
 * millis(); the DHT is polled no more often than once every two seconds.
 */
#include <Arduino.h>
#include <Wire.h>
#include <DHTesp.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

constexpr uint8_t DHT_PIN = 15;
constexpr uint8_t LDR_PIN = 34;       // ADC1, analogue input only
constexpr uint8_t SERVO_PIN = 25;
constexpr uint8_t LED_1_PIN = 18;
constexpr uint8_t LED_2_PIN = 19;
constexpr uint8_t MANUAL_PIN = 27;    // Switch shorts to ground in manual mode
constexpr uint8_t RESET_PIN = 26;     // Normally open, shorts to ground
constexpr uint32_t DHT_PERIOD = 2000;
constexpr uint32_t LIGHT_PERIOD = 250;
constexpr uint32_t DISPLAY_PERIOD = 250;
constexpr uint32_t STATUS_PERIOD = 1000;
constexpr uint32_t RECOVERY_MS = 6000;
constexpr int VENT_MAX_DEG = 90;
constexpr int VENT_FAULT_DEG = 45;

enum class Mode { AUTO, MANUAL, FAULT };
DHTesp dht;
Servo vent;
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledReady = false;
Mode mode = Mode::AUTO;
bool dhtValid = false, ldrValid = false, haveDhtSample = false;
bool injectDht = false, injectLdr = false;
bool hotDemand = false, growLights = false;
float temperatureC = NAN, humidityPct = NAN;
int lightRaw = -1, lightPct = -1, ventDegrees = 0;
float hotOnC = 29.0f;
int darkOnPct = 30;
uint32_t nextDht = 0, nextLight = 0, nextDisplay = 0, nextStatus = 0;
uint32_t healthySince = 0;
bool wasHealthy = false;
bool manualStable = false, manualCandidate = false;
uint32_t manualChanged = 0;
bool resetWasPressed = false;
char command[64];
size_t commandLength = 0;

/** @brief Test whether a timer deadline has passed across millis wraparound.
 * @param now Current millis counter.
 * @param deadline Scheduled time.
 * @return True if the deadline is due. */
bool due(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

/** @brief Constrain a floating point value to a range.
 * @param value Proposed value.
 * @param minimum Inclusive lower bound.
 * @param maximum Inclusive upper bound.
 * @return Bounded value. */
float bound(float value, float minimum, float maximum) {
  return fminf(maximum, fmaxf(minimum, value));
}

/** @brief Read the ADC and derive a relative light percentage.
 * @param none No arguments.
 * @return Nothing; updates lightRaw, lightPct and ldrValid. */
void sampleLight() {
  lightRaw = analogRead(LDR_PIN);
  // On this module the analogue voltage rises as the room gets darker.
  ldrValid = !injectLdr && lightRaw > 1 && lightRaw < 4094;
  lightPct = ldrValid ? (int)lroundf((4095 - lightRaw) * 100.0f / 4095.0f) : -1;
}

/** @brief Poll DHT22 and reject missing or out-of-range readings.
 * @param none No arguments.
 * @return Nothing; updates sensor readings and validity. */
void sampleDht() {
  TempAndHumidity reading = dht.getTempAndHumidity();
  haveDhtSample = true;
  dhtValid = !injectDht && isfinite(reading.temperature) &&
             isfinite(reading.humidity) && reading.temperature >= -40.0f &&
             reading.temperature <= 80.0f && reading.humidity >= 0.0f &&
             reading.humidity <= 100.0f;
  if (dhtValid) {
    temperatureC = reading.temperature;
    humidityPct = reading.humidity;
  } else {
    temperatureC = NAN;
    humidityPct = NAN;
  }
}

/** @brief Filter contact bounce on the manual switch and read reset button.
 * @param now Current milliseconds.
 * @return True only on a new reset-button press. */
bool readControls(uint32_t now) {
  bool candidate = digitalRead(MANUAL_PIN) == LOW;
  if (candidate != manualCandidate) {
    manualCandidate = candidate;
    manualChanged = now;
  }
  if (now - manualChanged >= 40) manualStable = manualCandidate;
  bool pressed = digitalRead(RESET_PIN) == LOW;
  bool edge = pressed && !resetWasPressed;
  resetWasPressed = pressed;
  return edge;
}

/** @brief Determine fault recovery and select the highest-priority mode.
 * @param now Current milliseconds.
 * @param resetEdge True when the worker pressed reset.
 * @return Nothing; updates the latched operating mode. */
void selectMode(uint32_t now, bool resetEdge) {
  if (!haveDhtSample) return; // DHT has a two-second minimum sampling interval.
  bool healthy = dhtValid && ldrValid;
  if (!healthy) {
    wasHealthy = false;
    mode = Mode::FAULT;
    return;
  }
  if (!wasHealthy) {
    healthySince = now;
    wasHealthy = true;
  }
  if (mode == Mode::FAULT && !(resetEdge && now - healthySince >= RECOVERY_MS)) return;
  mode = manualStable ? Mode::MANUAL : Mode::AUTO;
}

/** @brief Compute outputs from the selected mode and sensor readings.
 * @param none No arguments.
 * @return Nothing; sets ventDegrees and growLights. */
void calculateOutputs() {
  if (mode == Mode::FAULT) {
    ventDegrees = VENT_FAULT_DEG;
    growLights = true;
    return;
  }
  if (mode == Mode::MANUAL) {
    ventDegrees = VENT_MAX_DEG;
    growLights = false;
    return;
  }
  if (!haveDhtSample || !ldrValid) {
    ventDegrees = VENT_FAULT_DEG;
    growLights = true;
    return;
  }
  // Cold protection overrides heat vent demand. Hysteresis avoids chatter.
  if (temperatureC <= 17.0f) hotDemand = false;
  else if (temperatureC >= hotOnC) hotDemand = true;
  else if (temperatureC <= hotOnC - 2.0f) hotDemand = false;
  ventDegrees = hotDemand
      ? (int)lroundf(VENT_MAX_DEG * bound((temperatureC - (hotOnC - 2.0f)) /
                                      (38.0f - (hotOnC - 2.0f)), 0.0f, 1.0f))
      : 0;
  if (lightPct <= darkOnPct) growLights = true;
  else if (lightPct >= darkOnPct + 6) growLights = false;
}

/** @brief Apply GPIO and PWM outputs only when values change.
 * @param none No arguments.
 * @return Nothing; writes two LEDs and the servo. */
void applyOutputs() {
  static int lastVent = -1;
  static int lastLights = -1;
  if (ventDegrees != lastVent) {
    vent.write(ventDegrees);
    lastVent = ventDegrees;
  }
  if ((int)growLights != lastLights) {
    digitalWrite(LED_1_PIN, growLights ? HIGH : LOW);
    digitalWrite(LED_2_PIN, growLights ? HIGH : LOW);
    lastLights = growLights;
  }
}

/** @brief Name the active operating mode.
 * @param none No arguments.
 * @return Constant mode name. */
const char *modeName() {
  switch (mode) {
    case Mode::AUTO: return "AUTO";
    case Mode::MANUAL: return "MANUAL OVERRIDE";
    default: return "SENSOR FAULT";
  }
}

/** @brief Refresh all live readings and the relevant operator alert.
 * @param none No arguments.
 * @return Nothing; writes the I2C screen if present. */
void renderDisplay() {
  if (!oledReady) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0); oled.print(modeName());
  oled.setCursor(0, 11); oled.print("Temp: ");
  if (dhtValid) oled.print(temperatureC, 1); else oled.print("N/A");
  oled.print(" C");
  oled.setCursor(0, 21); oled.print("Humidity: ");
  if (dhtValid) oled.print(humidityPct, 0); else oled.print("N/A");
  oled.print(" %");
  oled.setCursor(0, 31); oled.print("Light: ");
  if (ldrValid) oled.print(lightPct); else oled.print("N/A");
  oled.print(" %");
  oled.setCursor(0, 41); oled.print("Vent: "); oled.print(ventDegrees);
  oled.print(" deg  LED:"); oled.print(growLights ? "ON" : "OFF");
  oled.setCursor(0, 53);
  if (mode == Mode::FAULT) {
    oled.print("CHECK ");
    if (!dhtValid) oled.print("DHT22 ");
    if (!ldrValid) oled.print("LDR ");
    oled.print("RESET");
  } else if (mode == Mode::MANUAL) oled.print("AUTO CONTROL PAUSED");
  else oled.print(hotDemand ? "HEAT VENT ACTIVE" : "MONITORING");
  oled.display();
}

/** @brief Print one live status record over USB UART.
 * @param none No arguments.
 * @return Nothing; emits current measurements and outputs. */
void printStatus() {
  Serial.printf("mode=%s temp=%.1fC humidity=%.1f%% light=%d%% raw=%d "
                "vent=%ddeg leds=%s dht=%s ldr=%s\n", modeName(),
                temperatureC, humidityPct, lightPct, lightRaw, ventDegrees,
                growLights ? "on" : "off", dhtValid ? "ok" : "fault",
                ldrValid ? "ok" : "fault");
}

/** @brief Parse an entered numeric configuration value safely.
 * @param input Characters after a set command.
 * @param minimum Lowest accepted value.
 * @param maximum Highest accepted value.
 * @param result Receives the parsed number.
 * @return True only for one finite value within the limits. */
bool parseNumber(const char *input, float minimum, float maximum, float &result) {
  char *end = nullptr;
  float value = strtof(input, &end);
  while (*end == ' ') ++end;
  if (end == input || *end != '\0' || !isfinite(value) || value < minimum || value > maximum) return false;
  result = value;
  return true;
}

/** @brief Execute one newline-terminated UART command.
 * @param line Writable NUL-terminated command buffer.
 * @return Nothing; prints an acknowledgement or error. */
void executeCommand(char *line) {
  if (strcmp(line, "help") == 0) {
    Serial.println("status | set hot 25..34 | set dark 5..80 | fault dht on/off | fault ldr on/off | reset");
  } else if (strcmp(line, "status") == 0) {
    printStatus();
  } else if (strncmp(line, "set hot ", 8) == 0) {
    float value;
    if (parseNumber(line + 8, 25, 34, value)) { hotOnC = value; Serial.println("OK hot threshold"); }
    else Serial.println("ERR hot must be 25..34 C");
  } else if (strncmp(line, "set dark ", 9) == 0) {
    float value;
    if (parseNumber(line + 9, 5, 80, value)) { darkOnPct = (int)lroundf(value); Serial.println("OK dark threshold"); }
    else Serial.println("ERR dark must be 5..80 percent");
  } else if (strcmp(line, "fault dht on") == 0 || strcmp(line, "fault dht off") == 0) {
    injectDht = strcmp(line, "fault dht on") == 0;
    if (injectDht) dhtValid = false;
    Serial.println("OK DHT fault injection; use reset button after recovery");
  } else if (strcmp(line, "fault ldr on") == 0 || strcmp(line, "fault ldr off") == 0) {
    injectLdr = strcmp(line, "fault ldr on") == 0;
    if (injectLdr) ldrValid = false;
    Serial.println("OK LDR fault injection; use reset button after recovery");
  } else if (strcmp(line, "reset") == 0) {
    // UART reset is useful for a remote operator with console access.
    selectMode(millis(), true);
    Serial.println(mode == Mode::FAULT ? "WAIT for healthy sensors 6s then reset" : "OK reset");
  } else Serial.println("ERR unknown command; type help");
}

/** @brief Accumulate UART input without blocking or heap allocations.
 * @param none No arguments.
 * @return Nothing; handles complete newline-terminated commands. */
void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      command[commandLength] = '\0';
      if (commandLength) executeCommand(command);
      commandLength = 0;
    } else if (commandLength < sizeof(command) - 1) command[commandLength++] = c;
    else { commandLength = 0; Serial.println("ERR command too long"); }
  }
}

/** @brief Initialise GPIO, UART, ADC, PWM, I2C, display and sensors.
 * @param none No arguments.
 * @return Nothing; Arduino entry point. */
void setup() {
  Serial.begin(115200);
  pinMode(MANUAL_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(LDR_PIN, ADC_11db);
  Wire.begin(21, 22);
  oledReady = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  dht.setup(DHT_PIN, DHTesp::DHT22);
  vent.setPeriodHertz(50);
  vent.attach(SERVO_PIN, 500, 2400);
  sampleLight();
  calculateOutputs();
  applyOutputs();
  renderDisplay();
  Serial.println("NURSERY READY; type help. Waiting for first DHT reading.");
  if (!oledReady) Serial.println("WARNING OLED missing at I2C address 0x3C");
  uint32_t now = millis();
  nextDht = now + DHT_PERIOD;
  nextLight = now + LIGHT_PERIOD;
  nextDisplay = now + DISPLAY_PERIOD;
  nextStatus = now + STATUS_PERIOD;
}

/** @brief Service inputs, fault state, outputs and scheduled work.
 * @param none No arguments.
 * @return Nothing; Arduino entry point, intentionally no delay(). */
void loop() {
  uint32_t now = millis();
  pollSerial();
  bool resetEdge = readControls(now);
  if (due(now, nextLight)) { nextLight = now + LIGHT_PERIOD; sampleLight(); }
  if (due(now, nextDht)) { nextDht = now + DHT_PERIOD; sampleDht(); }
  selectMode(now, resetEdge);
  calculateOutputs();
  applyOutputs();
  if (due(now, nextDisplay)) { nextDisplay = now + DISPLAY_PERIOD; renderDisplay(); }
  if (due(now, nextStatus)) { nextStatus = now + STATUS_PERIOD; printStatus(); }
}
