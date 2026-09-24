# ESP32 Micro-Climate Nursery — Scenario 2

COMP50069 Hardware, Microcontrollers and Sensors. This is a teaching prototype for one nursery growing area. It measures indoor temperature, humidity, and relative light; it moves a model vent and switches two LEDs that stand in for grow lamps.

## Files

| File | Purpose |
| --- | --- |
| `sketch.ino` | ESP32 Arduino firmware, with Doxygen comments on every function |
| `diagram.json` | Wokwi circuit and connections |
| `libraries.txt` | Libraries needed by Wokwi and Arduino IDE |

The submitted report contains the full source in its appendix and must include this repository's URL and a saved Wokwi project URL. A `diagram.json` file alone is **not** a Wokwi share link.

## What the controller does

| Mode | Entry | Vent | Two indicator LEDs |
| --- | --- | --- | --- |
| Auto | Healthy sensors, Manual switch off | Opens farther as indoor temperature rises; closed when cold | On in darkness, off again after light rises |
| Manual Override | Healthy sensors, Manual switch on | Held fully open at 90° | Off; automatic light and vent decisions paused |
| Sensor Fault | Invalid DHT22 or LDR reading, from either mode | Held at 45° | On; OLED names the failed sensor |

A fault takes priority over Manual, which takes priority over Auto. A fault remains active until **both sensors have read normally for six continuous seconds** and the worker presses the reset button (or enters `reset` in the serial monitor). A reset attempted too early is ignored. The fallback position is a design choice for this model, not a guaranteed crop-safe position for every nursery.

Auto starts opening the vent at 29 °C and closes it after the room cools below the lower threshold. The two-degree temperature gap prevents repeated switching near the limit. The lights turn on at or below 30% relative brightness and off at or above 36%. The vent angle varies between 0° and 90° with temperature; it is not only on/off. Humidity is displayed but is not controlled because the circuit has no humidifier. The 17 °C indoor cold guard closes the vent, but there is no outdoor sensor, so a worker must consider outside weather.

## Open in Wokwi

1. Create an [ESP32 Arduino project](https://wokwi.com/projects/new/esp32).
2. Replace its `sketch.ino` and `diagram.json` with the files in this repository. Add `libraries.txt` with the four listed libraries, or install them through Library Manager.
3. Start the simulation. Wait about two seconds for the first DHT22 reading. Open the serial monitor at **115200 baud**.
4. Change the DHT22 temperature and the photoresistor's `lux` control. Watch the OLED, vent angle, and two LEDs. The OLED displays a **relative brightness percentage**, not lux.
5. Move the slide switch right for Manual Override and left for Auto. Use the red pushbutton to reset a repaired fault.
6. Save the project while signed in to Wokwi and paste the resulting share URL into the report. This repository does not by itself create that URL.

## Physical wiring

| Part | ESP32 connection | Power or other connection |
| --- | --- | --- |
| DHT22 | DATA GPIO15 | 3.3 V and ground; add a data pull-up if the module does not include one |
| LDR module | AO GPIO34 (ADC1) | 3.3 V and ground; the analogue signal must not exceed 3.3 V |
| SSD1306 I2C OLED | SDA GPIO21, SCL GPIO22 | 3.3 V and ground; code expects address `0x3C` |
| Servo | Signal GPIO25 | Separate regulated 5 V supply for a real build; ground shared with ESP32 |
| Two LEDs | GPIO18 and GPIO19 | One 220 Ω series resistor per LED; cathodes to ground |
| Manual slide switch | Common to GPIO27, right contact to ground | Left contact unused; built-in pull-up holds Auto when open |
| Fault reset button | GPIO26 to ground when pressed | Normally open; built-in pull-up |

Check the labels on your particular ESP32 board before wiring. Power down before changing connections. Limit the real vent linkage so the servo cannot push beyond its mechanical stops. Do not connect a real grow lamp directly to an ESP32 GPIO pin; use a correctly rated driver. The LEDs in this circuit demonstrate the control signal only.

## Serial commands

| Command | Effect |
| --- | --- |
| `help` | List commands |
| `status` | Print current readings, mode, vent angle, and LED state |
| `set hot 25..34` | Change vent opening threshold in °C for this power session |
| `set dark 5..80` | Change lighting threshold in relative percent for this power session |
| `fault dht on` / `fault dht off` | Simulate and clear a DHT22 failure |
| `fault ldr on` / `fault ldr off` | Simulate and clear an LDR failure |
| `reset` | Request recovery after six seconds of healthy readings |

End commands with a newline. Settings return to 29 °C and 30% after restarting. Fault injection is for demonstrations; it should be disabled or access-controlled in a real installation.

## Test before submission

The report has a test table for Auto, Manual, Fault, recovery, threshold changes, malformed commands, and physical disconnection. Run those cases in Wokwi and on the physical prototype where appropriate. Record **actual observations and Pass/Fail** in the report; do not mark a case as passed based only on this source code. In particular, show changing vent angles at 29, 33 and 38 °C, lights switching at 30%/36%, a fault overriding Manual, and reset being refused until the sensors are healthy for six seconds.

The DHT22 is sampled every two seconds, so a real DHT disconnection is detected on the next scheduled read. The LDR is sampled every 250 ms. An LDR value at the very low or high end of the ADC range is treated as a possible fault, but plausible stuck readings or some floating disconnections cannot be detected reliably with one sensor. The firmware cannot verify that a commanded vent actually moved. The physical build and measurements remain necessary for the assignment demonstration.
