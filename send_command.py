"""Send a command to the ESP32 running in Wokwi for VS Code."""

import sys
import time

import serial


if len(sys.argv) < 2:
    print("Usage: python send_command.py fault dht on")
    sys.exit(1)

command = " ".join(sys.argv[1:])

try:
    with serial.serial_for_url(
        "rfc2217://localhost:4000", baudrate=115200, timeout=0.5
    ) as port:
        port.reset_input_buffer()
        port.write((command + "\n").encode("ascii"))
        port.flush()
        print(f"Sent: {command}")
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            reply = port.readline().decode("ascii", errors="replace").strip()
            if not reply:
                continue
            if reply.startswith(("OK ", "ERR ", "WAIT ")) or (
                command == "status" and reply.startswith("mode=")
            ):
                print(f"ESP32: {reply}")
                break
        else:
            print("No ESP32 reply received. Confirm the simulator is running, then retry.")
except (serial.SerialException, OSError) as error:
    print("Could not reach Wokwi on port 4000.")
    print("Start the simulator again after updating wokwi.toml.")
    print(error)
    sys.exit(1)
