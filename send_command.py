"""Send a command to the ESP32 running in Wokwi for VS Code."""

import sys

import serial


if len(sys.argv) < 2:
    print("Usage: python send_command.py fault dht on")
    sys.exit(1)

command = " ".join(sys.argv[1:])

try:
    with serial.serial_for_url(
        "rfc2217://localhost:4000", baudrate=115200, timeout=2
    ) as port:
        port.write((command + "\n").encode("ascii"))
        port.flush()
    print(f"Sent: {command}")
except (serial.SerialException, OSError) as error:
    print("Could not reach Wokwi on port 4000.")
    print("Start the simulator again after updating wokwi.toml.")
    print(error)
    sys.exit(1)
