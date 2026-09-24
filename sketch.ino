/** ESP32 nursery controller: Auto, Manual, and Sensor Fault modes. */
#include <Arduino.h>
#include <Wire.h>
#include <DHTesp.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Wiring
const byte DHT_PIN = 15, LDR_PIN = 34, SERVO_PIN = 25;
const byte LED1_PIN = 18, LED2_PIN = 19;
const byte MANUAL_PIN = 27, RESET_PIN = 26;
const byte VENT_BUTTON_PIN = 32, LIGHT_BUTTON_PIN = 33;

// Timing and safety settings
const unsigned long DHT_INTERVAL = 2000;
const unsigned long LIGHT_INTERVAL = 250;
const unsigned long SCREEN_INTERVAL = 250;
const unsigned long STATUS_INTERVAL = 1000;
const unsigned long RECOVERY_TIME = 6000;
const unsigned long DEBOUNCE_TIME = 40;
const int FAULT_VENT = 45;

enum Mode { AUTO, MANUAL, FAULT };
Mode mode = AUTO;
DHTesp dht;
Servo vent;
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledReady = false;

float temperatureC = NAN, humidityPct = NAN, hotThreshold = 29.0f;
int lightRaw = -1, lightPct = -1, darkThreshold = 30;
int ventDegrees = 0, manualVent = 0;
bool dhtValid = false, ldrValid = false, haveDhtSample = false;
bool injectDht = false, injectLdr = false;
bool hotDemand = false, autoLights = false, growLights = false;
bool manualOn = false, manualCandidate = false, manualLights = false;
bool wasHealthy = false;
unsigned long manualChanged = 0, healthySince = 0;
unsigned long lastDht = 0, lastLight = 0, lastScreen = 0, lastStatus = 0;

// Each button connects its GPIO pin to ground when pressed.
struct Button {
  byte pin;
  bool candidate;
  bool stable;
  unsigned long changed;
};
Button resetButton = {RESET_PIN, false, false, 0};
Button ventButton = {VENT_BUTTON_PIN, false, false, 0};
Button lightButton = {LIGHT_BUTTON_PIN, false, false, 0};

char command[64];
byte commandLength = 0;

/** Return true once when a button has been held for 40 ms. */
bool newPress(Button &button, unsigned long now) {
  bool pressed = digitalRead(button.pin) == LOW;
  if (pressed != button.candidate) {
    button.candidate = pressed;
    button.changed = now;
  }
  if (pressed != button.stable && now - button.changed >= DEBOUNCE_TIME) {
    button.stable = pressed;
    return pressed;
  }
  return false;
}

/** Read the Auto/Manual slide switch without contact bounce. */
void readManualSwitch(unsigned long now) {
  bool reading = digitalRead(MANUAL_PIN) == LOW;
  if (reading != manualCandidate) {
    manualCandidate = reading;
    manualChanged = now;
  }
  if (manualOn != manualCandidate && now - manualChanged >= DEBOUNCE_TIME) {
    manualOn = manualCandidate;
    if (manualOn) {
      manualVent = 0;
      manualLights = false;
    }
  }
}

/** Read the light sensor and convert its ADC value to a relative percentage. */
void readLight() {
  lightRaw = analogRead(LDR_PIN);
  ldrValid = !injectLdr && lightRaw > 1 && lightRaw < 4094;
  // This module's voltage rises as the room gets darker.
  lightPct = ldrValid ? (int)round((4095 - lightRaw) * 100.0f / 4095.0f) : -1;
}

/** Read the DHT22 and reject missing or out-of-range measurements. */
void readDht() {
  TempAndHumidity reading = dht.getTempAndHumidity();
  haveDhtSample = true;
  dhtValid = !injectDht && isfinite(reading.temperature) &&
             isfinite(reading.humidity) && reading.temperature >= -40 &&
             reading.temperature <= 80 && reading.humidity >= 0 &&
             reading.humidity <= 100;
  temperatureC = dhtValid ? reading.temperature : NAN;
  humidityPct = dhtValid ? reading.humidity : NAN;
}

/** Choose Fault first, then Manual, then Auto. Fault requires a delayed reset. */
void chooseMode(unsigned long now, bool resetPressed) {
  if (!haveDhtSample) return;
  if (!dhtValid || !ldrValid) {
    mode = FAULT;
    wasHealthy = false;
    return;
  }
  if (!wasHealthy) {
    healthySince = now;
    wasHealthy = true;
  }
  if (mode == FAULT && (!resetPressed || now - healthySince < RECOVERY_TIME)) return;
  mode = manualOn ? MANUAL : AUTO;
}

/** Calculate the vent angle and LED state for the active mode. */
void chooseOutputs() {
  // Remember Auto's light decision separately from the Manual button setting.
  if (ldrValid) {
    if (lightPct <= darkThreshold) autoLights = true;
    else if (lightPct >= darkThreshold + 6) autoLights = false;
  }
  if (dhtValid) {
    if (temperatureC <= 17 || temperatureC <= hotThreshold - 2) hotDemand = false;
    else if (temperatureC >= hotThreshold) hotDemand = true;
  }

  if (mode == FAULT || !haveDhtSample) {
    ventDegrees = FAULT_VENT;
    growLights = true;
  } else if (mode == MANUAL) {
    ventDegrees = manualVent;
    growLights = manualLights;
  } else {
    // At 38 C or above the vent is fully open.
    float fraction = (temperatureC - (hotThreshold - 2)) /
                     (38.0f - (hotThreshold - 2));
    ventDegrees = hotDemand ? constrain((int)round(90 * fraction), 0, 90) : 0;
    growLights = autoLights;
  }
}

/** Send output changes to the servo and both LEDs. */
void applyOutputs() {
  static int previousVent = -1, previousLights = -1;
  if (ventDegrees != previousVent) {
    vent.write(ventDegrees);
    previousVent = ventDegrees;
  }
  if ((int)growLights != previousLights) {
    digitalWrite(LED1_PIN, growLights ? HIGH : LOW);
    digitalWrite(LED2_PIN, growLights ? HIGH : LOW);
    previousLights = growLights;
  }
}

/** Return a short name for the current operating mode. */
const char *modeName() {
  if (mode == FAULT) return "SENSOR FAULT";
  if (mode == MANUAL) return "MANUAL OVERRIDE";
  return "AUTO";
}

/** Show readings, outputs, and any fault on the OLED. */
void showScreen() {
  if (!oledReady) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);  oled.print(modeName());
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
  oled.print(" deg LED:"); oled.print(growLights ? "ON" : "OFF");
  oled.setCursor(0, 53);
  if (mode == FAULT) {
    oled.print("CHECK ");
    if (!dhtValid) oled.print("DHT22 ");
    if (!ldrValid) oled.print("LDR ");
    oled.print("RESET");
  } else if (mode == MANUAL) oled.print("VENT/LIGHT BUTTONS");
  else oled.print(hotDemand ? "HEAT VENT ACTIVE" : "MONITORING");
  oled.display();
}

/** Print a single status line to the serial monitor. */
void printStatus() {
  Serial.printf("mode=%s temp=%.1fC humidity=%.1f%% light=%d%% raw=%d "
                "vent=%ddeg leds=%s dht=%s ldr=%s\n", modeName(),
                temperatureC, humidityPct, lightPct, lightRaw, ventDegrees,
                growLights ? "on" : "off", dhtValid ? "ok" : "fault",
                ldrValid ? "ok" : "fault");
}

/** Parse a number and check its allowed range. */
bool readNumber(const char *text, float minimum, float maximum, float &value) {
  char *end;
  value = strtof(text, &end);
  while (*end == ' ') end++;
  return end != text && *end == '\0' && isfinite(value) &&
         value >= minimum && value <= maximum;
}

/** Run one complete command from the serial monitor. */
void runCommand(char *line) {
  if (strcmp(line, "help") == 0) {
    Serial.println("status | set hot 25..34 | set dark 5..80 | fault dht on/off | fault ldr on/off | reset");
  } else if (strcmp(line, "status") == 0) {
    printStatus();
  } else if (strncmp(line, "set hot ", 8) == 0) {
    float value;
    if (readNumber(line + 8, 25, 34, value)) {
      hotThreshold = value;
      Serial.println("OK hot threshold");
    } else Serial.println("ERR hot must be 25..34 C");
  } else if (strncmp(line, "set dark ", 9) == 0) {
    float value;
    if (readNumber(line + 9, 5, 80, value)) {
      darkThreshold = (int)round(value);
      Serial.println("OK dark threshold");
    } else Serial.println("ERR dark must be 5..80 percent");
  } else if (strcmp(line, "fault dht on") == 0 || strcmp(line, "fault dht off") == 0) {
    injectDht = strcmp(line, "fault dht on") == 0;
    if (injectDht) dhtValid = false;
    Serial.println("OK DHT fault injection; use reset button after recovery");
  } else if (strcmp(line, "fault ldr on") == 0 || strcmp(line, "fault ldr off") == 0) {
    injectLdr = strcmp(line, "fault ldr on") == 0;
    if (injectLdr) ldrValid = false;
    Serial.println("OK LDR fault injection; use reset button after recovery");
  } else if (strcmp(line, "reset") == 0) {
    chooseMode(millis(), true);
    Serial.println(mode == FAULT ? "WAIT for healthy sensors 6s then reset" : "OK reset");
  } else Serial.println("ERR unknown command; type help");
}

/** Collect serial bytes until a newline completes a command. */
void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      command[commandLength] = '\0';
      if (commandLength) runCommand(command);
      commandLength = 0;
    } else if (commandLength < sizeof(command) - 1) {
      command[commandLength++] = c;
    } else {
      commandLength = 0;
      Serial.println("ERR command too long");
    }
  }
}

/** Set up the hardware and start the timers. */
void setup() {
  Serial.begin(115200);
  pinMode(MANUAL_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(VENT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LIGHT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(LDR_PIN, ADC_11db);
  Wire.begin(21, 22);
  oledReady = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  dht.setup(DHT_PIN, DHTesp::DHT22);
  vent.setPeriodHertz(50);
  vent.attach(SERVO_PIN, 500, 2400);
  readLight();
  chooseOutputs();
  applyOutputs();
  showScreen();
  Serial.println("NURSERY READY; type help. Waiting for first DHT reading.");
  if (!oledReady) Serial.println("WARNING OLED missing at I2C address 0x3C");
  lastDht = lastLight = lastScreen = lastStatus = millis();
}

/** Read inputs, update the mode, and run each scheduled task. */
void loop() {
  unsigned long now = millis();
  readSerial();
  readManualSwitch(now);
  bool resetPressed = newPress(resetButton, now);
  bool ventPressed = newPress(ventButton, now);
  bool lightPressed = newPress(lightButton, now);

  if (now - lastLight >= LIGHT_INTERVAL) {
    lastLight = now;
    readLight();
  }
  if (now - lastDht >= DHT_INTERVAL) {
    lastDht = now;
    readDht();
  }
  chooseMode(now, resetPressed);
  if (mode == MANUAL) {
    if (ventPressed) manualVent = (manualVent + 45) % 135;
    if (lightPressed) manualLights = !manualLights;
  }
  chooseOutputs();
  applyOutputs();
  if (now - lastScreen >= SCREEN_INTERVAL) {
    lastScreen = now;
    showScreen();
  }
  if (now - lastStatus >= STATUS_INTERVAL) {
    lastStatus = now;
    printStatus();
  }
}
