# PRIZM Controller ↔ ESP32 Communication Bridge

## Overview

This firmware turns the ESP32 into a WiFi-to-UART bridge for the PRIZM controller. It exposes a TCP server that accepts JSON messages and forwards them to the PRIZM over serial, replacing the need for a wired USB connection.

## How It Works

- A TCP server runs on **port 81**
- Incoming TCP data → forwarded to PRIZM via UART
- PRIZM responses → sent back to the TCP client (line-delimited)
- On connection, the ESP32 sends:
  ```json
  { "type": "esp32_ready" }
  ```

## Wiring

- ESP32 TX (GPIO 13) → PRIZM RX (D2)
  - No level shifter required
  - Recommended: 10kΩ pull-down resistor
- ESP32 RX (GPIO 4) ← PRIZM TX (D9)
  - Use voltage divider (the example uses 2kΩ + 3.9kΩ; pulls the voltage down to less than 3.3 V)
- PRIZM RX/TX are on Digital Sensor Port (D2/D9)

for alternative pin mappings, see page 129 of the [PRIZM Programming Guide](https://asset.pitsco.com/sharedimages/resources/tetrix-prizm-programming-guide.pdf)

Note: for ESP32 C6 (has 2 USB-C ports on it), the 10 k ohm pull down is not present

<table align="center">
  <tr>
    <td>
      <img src="images/esp32_circuit_diagram.png" alt="ESP32 Circuit Diagram" width="250">
    </td>
    <td width="50"></td>
    <td>
      <img src="images/physical_circuit.jpg" alt="Physical Circuit" width="250">
    </td>
  </tr>
</table>

## Configuration

- UART: 38400 baud, SERIAL_8N1

- WiFi credentials must be defined in env.h:

  ```c
  const char* ssid = "...";
  const char* password = "...";
  ```

## Usage

1. Flash Firmware onto the ESP32 (Using either PlatformIO or Arduino IDE)
2. Power the System
3. Check Serial Monitor to see the Connection status and the IP Address of the ESP32
4. Now you can open a connection to `<ESP_IP>:81` and send commands (this is done by the client.py)

## Notes

- Messages are processed line-by-line (\n terminated)
- Ensure PRIZM firmware matches the same baud rate
- A small delay (delay(2)) is used for stability under load
