# PropulsionV2p0 Project Overview

## Purpose

`PropulsionV2p0` is the ESP32-based propulsion controller firmware for SV Kerosheba. It is built around SensESP and publishes propulsion, battery, cooling, temperature, and runtime data to Signal K while also driving local hardware such as the LCD display, BMS Modbus link, OneWire temperature sensors, CAN/TWAI motor-controller link, relay outputs, and shaft RPM input.

The current firmware identifies itself on the network as:

```text
Propulsion5
```

## Current Git Status

At the time this document was created:

```text
Branch: main
Tracking: origin/main
State: ahead of origin/main by 1 commit
Local changes: src/main.cpp modified
```

Only this documentation file was added; the existing source change in `src/main.cpp` was left untouched.

## Build System

The project uses PlatformIO.

Default environment:

```text
pioarduino_esp32s3
```

Useful commands:

```sh
pio run
pio run -t upload
pio device monitor
```

For OTA uploads, the project defines OTA environments such as:

```text
pioarduino_esp32s3_ota
pioarduino_esp32_ota
```

The OTA upload target is currently configured for:

```text
192.168.88.239
```

Do not commit OTA passwords or other secrets in documentation.

## Main Dependencies

Core libraries and frameworks:

- Arduino on ESP32 / ESP32-S3 via PlatformIO
- SensESP `>=3.0.0-beta.4,<4.0.0`
- SensESP OneWire
- Adafruit GFX
- Custom ILI9488 LCD driver
- ESP32 TWAI/CAN support
- Local Modbus RTU implementation

The repo also contains ESP-IDF managed components under `managed_components/`.

## Hardware Interfaces

Primary hardware wiring is defined in `src/main.cpp`.

### OneWire Temperature Bus

```text
GPIO37: OneWire temperature bus
```

Configured sensors:

- Port motor temperature
- Starboard motor temperature
- Port controller temperature
- Starboard controller temperature
- Engine room temperature
- Coolant temperature

### Digital Inputs

```text
GPIO39: Coolant fan status
GPIO40: Coolant pump status
GPIO41: Ambient fan status
GPIO42: Shaft RPM proxy input
GPIO45: Spare opto input 5
GPIO46: Spare opto input 6
GPIO47: Spare opto input 7
GPIO48: Spare opto input 8
```

### Relay Outputs

```text
GPIO9:  Coolant fan control
GPIO10: Coolant pump control
GPIO11: Ambient fan control
GPIO12: Spare relay 1
GPIO13: Spare relay 2
GPIO14: Spare relay 3
```

### CAN / TWAI

```text
GPIO16: CAN TX
GPIO15: CAN RX
```

The CAN wrapper is implemented in:

```text
src/ker_can.cpp
src/ker_can.hpp
src/ESP32-TWAI-CAN.cpp
src/ESP32-TWAI-CAN.hpp
src/ezkontrolVCU.cpp
src/ezkontrolVCU.hpp
```

The code models two motor controller endpoints:

```text
Port MCU ID: 239
Starboard MCU ID: 240
```

### Modbus RTU BMS

Modbus pins:

```text
GPIO8:  Modbus DE
GPIO17: Modbus TXD
GPIO18: Modbus RXD
```

BMS handling lives in:

```text
src/bms_setup.cpp
src/bms_setup.h
src/bms_parser.h
src/esp32ModbusRTU.cpp
src/esp32ModbusRTU.h
```

The firmware polls two BMS devices and publishes Battery 01 and Battery 02 data to Signal K.

### LCD Display

ILI9488 display pins:

```text
GPIO7:  LCD CS
GPIO6:  LCD DC
GPIO35: LCD MOSI
GPIO0:  LCD CLK
GPIO5:  LCD RST
GPIO36: LCD MISO
GPIO4:  LCD LED
```

Display code:

```text
src/propDisplay.cpp
src/propDisplay.h
src/ILI9488.cpp
src/ILI9488.h
```

The display presents propulsion, BMS, cooling, temperature, uptime, Wi-Fi, and shaft RPM data.

### Analog Throttle

```text
GPIO2: ADC throttle input
```

The throttle code supports:

- ADC reference voltage configuration
- Neutral midpoint calculation
- Neutral hysteresis band
- Idle threshold and idle percentage
- Maximum phase current scaling
- Signal K output for helm throttle position
- CAN current commands to both motor controllers

Some throttle/CAN setup calls are currently commented out in `setup()`.

## Signal K Outputs

The firmware publishes data under these broad Signal K areas:

```text
propulsion.*
electrical.batteries.*
environment.*
```

Examples include:

```text
propulsion.port.temperature
propulsion.starboard.temperature
propulsion.port.motor.temperature
propulsion.starboard.motor.temperature
propulsion.port.controller.temperature
propulsion.starboard.controller.temperature
propulsion.speed
propulsion.helm.throttlePosition
electrical.batteries.bat01.voltage
electrical.batteries.bat01.current
electrical.batteries.bat01.capacity.stateOfCharge
electrical.batteries.bat02.voltage
electrical.batteries.bat02.current
electrical.batteries.bat02.capacity.stateOfCharge
environment.engineRoom.temperature
environment.coolant.temperature
```

Signal K metadata is extended with zone information using:

```text
src/signalk_extended_metadata.h
```

## Runtime Behavior

In the current `setup()` flow:

- SensESP is initialized with hostname `Propulsion5`.
- OTA support is compiled in.
- SPIFFS root files are logged after SensESP initializes storage.
- LCD display setup runs.
- BMS Modbus setup runs.
- OneWire temperature setup runs.
- SensESP networking and Signal K handling start.
- Runtime logs include Wi-Fi mode, AP SSID, AP IP, and whether AP mode is active.

Currently commented out in `setup()`:

- Throttle setup
- CAN bus setup
- Fan/pump setup
- Shaft RPM setup

These appear to be intentional staging switches during development.

## Source Layout

Key files:

```text
src/main.cpp                         Main firmware setup and application wiring
src/bms_setup.cpp/.h                 Modbus BMS polling and Signal K output setup
src/bms_parser.h                     Parsing of BMS register blocks
src/ker_can.cpp/.hpp                 Boat-level CAN wrapper for port/starboard MCUs
src/ezkontrolVCU.cpp/.hpp            Motor controller CAN protocol logic
src/ESP32-TWAI-CAN.cpp/.hpp          ESP32 TWAI/CAN abstraction
src/propDisplay.cpp/.h               LCD screen drawing and display data model
src/ILI9488.cpp/.h                   LCD driver
src/rpm_class.hpp                    Shaft frequency helper
src/signalk_extended_metadata.h      Signal K metadata with zones
```

## Notes And Risks

- The repo currently contains generated or managed dependency folders. Be careful before committing changes under `.pio/`, `build/`, or `managed_components/`.
- `src/main.cpp` had local modifications when this document was created.
- Several hardware functions are present but currently disabled by commented-out setup calls.
- OTA credentials and other secrets should be moved out of tracked configuration when practical.
