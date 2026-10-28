# Conscientia CanSat Avionics Software

Flight software, payload processing, recovery control, onboard data logging, telemetry, and ground-station integration for the **CONSCIENTIA CanSat** developed by **Team Ignition, VIT Chennai**.

The flight computer is based on the **Waveshare RP2040-Zero**. It acquires data from the onboard sensors, detects release from the carrier, controls parachute deployment, processes mission payload data, stores flight measurements on a MicroSD card, and transmits live telemetry through a **LoRa E32 433T3D** link.

The ground station uses an **ESP32 + LoRa E32 433T3D** receiver and displays live telemetry using **GLANCE**.

---

## Features

- **Drone-Drop Recovery**
  - IMU-based release detection
  - Immediate parachute deployment after confirmed drop
  - MG90 servo actuation
  - Latched deployment state

- **Flight Sensing**
  - 3-axis acceleration
  - 3-axis angular velocity
  - barometric pressure
  - temperature
  - barometric altitude
  - GNSS position, altitude and ground motion

- **Payload Experiments**
  - PM1.0, PM2.5 and PM10 profiling
  - relative air-quality response
  - three-direction light sensing and fusion
  - wind-data estimation/preparation

- **Live Telemetry**
  - 28-field ASCII telemetry packet
  - LoRa E32 433T3D radio link
  - XOR packet checksum
  - ESP32 ground receiver
  - GLANCE live visualization
  - GNSS map tracking

- **Post-Flight Data**
  - MicroSD storage using FatFs
  - 17 dedicated CSV datasets
  - post-flight graph generation

---

# System Overview

The CanSat maintains two independent data paths:

```text
                              CONSCIENTIA

                             ┌─────────────┐
                             │   Sensors   │
                             └──────┬──────┘
                                    │
                                    ▼
                            ┌───────────────┐
                            │ RP2040-Zero   │
                            │ Flight Computer│
                            └───────┬───────┘
                                    │
                     ┌──────────────┴──────────────┐
                     │                             │
                     ▼                             ▼
               ┌──────────┐                  ┌──────────┐
               │ MicroSD  │                  │ LoRa E32 │
               └────┬─────┘                  └────┬─────┘
                    │                              │
                    ▼                        433 MHz Radio
             Post-Flight CSV                      │
                    │                             ▼
                    ▼                        ┌──────────┐
              Data Analysis                 │ LoRa E32 │
                                             └────┬─────┘
                                                  │
                                                  ▼
                                             ┌─────────┐
                                             │  ESP32  │
                                             └────┬────┘
                                                  │ USB
                                                  ▼
                                               GLANCE
```

The **LoRa path** provides live telemetry.

The **MicroSD path** independently stores complete post-flight datasets.

LoRa transmits CSV-formatted text packets. It does **not** transmit `.csv` files. Actual `.csv` files are generated and stored on the MicroSD card.

---

# Hardware Requirements

## CanSat

| Component | Quantity | Purpose |
|---|---:|---|
| Waveshare RP2040-Zero | 1 | Main flight computer |
| IMU | 1 | Accelerometer + gyroscope |
| BMP280 | 1 | Barometric pressure |
| BH1750 | 3 | Multi-direction light sensing |
| PMS5003 | 1 | Particulate-matter sensing |
| MQ135 | 1 | Air-quality sensing |
| L89HA | 1 | GNSS |
| MicroSD Card Module | 1 | Flight-data storage |
| MicroSD Card | 1 | Persistent storage |
| LoRa E32 433T3D | 1 | Airborne telemetry |
| MG90 Servo | 1 | Parachute deployment |

## Ground Station

| Component | Quantity | Purpose |
|---|---:|---|
| ESP32 | 1 | Ground-station controller |
| LoRa E32 433T3D | 1 | Telemetry receiver |
| Laptop | 1 | GLANCE dashboard |

> Verify the voltage, current and logic-level requirements of the exact hardware modules before applying power. The GPIO diagram below defines signal allocation, not a universal power-wiring specification.

---

# Flight Computer Wiring

## RP2040-Zero Signal Layout

```text
                     CONSCIENTIA FLIGHT COMPUTER
                       Waveshare RP2040-Zero

                    ┌──────────────────────────┐
                    │                          │
     LoRa E32 RX ◄──│ GP0   UART0 TX           │
     LoRa E32 TX ──►│ GP1   UART0 RX           │
     LoRa E32 M0 ◄──│ GP2                      │
     LoRa E32 M1 ◄──│ GP3                      │
                    │                          │
       PMS5003 TX ──►│ GP4   PIO UART RX           │
       PMS5003 RX ◄──│ GP5   PIO UART TX           │
                    │                          │
     LoRa E32 AUX ─►│ GP6                      │
                    │                          │
       MG90 SIG ◄───│ GP7   PWM                │
                    │                          │
      PMS5003 TX ──►│ GP8   PIO UART RX        │
      PMS5003 RX ◄──│ GP9   PIO UART TX        │
                    │                          │
       SD SCK  ◄────│ GP10 ┐                   │
       SD MOSI ◄────│ GP11 │ SPI1              │
       SD MISO ────►│ GP12 │                   │
       SD CS   ◄────│ GP13 ┘                   │
                    │                          │
       I2C1 SDA ────│ GP14 ── BH1750 #3        │
       I2C1 SCL ────│ GP15                     │
                    │                          │
                    │ GP16 ── Onboard LED      │
                    │                          │
       I2C0 SDA ────│ GP26 ─┬─ IMU             │
       I2C0 SCL ────│ GP27  ├─ BMP280          │
                    │       ├─ BH1750 #1       │
                    │       ├─ BH1750 #2       │
                    │       └─ L89HA GNSS      │
                    │                          │
                    │ GP28                     │
       MQ135 AO ───►│ GP29   ADC3              │
                    │                          │
                    └──────────────────────────┘
```

## Pin Reference

| GPIO | Interface | Connection |
|---|---|---|
| GP0 | UART0 TX | LoRa E32 RX |
| GP1 | UART0 RX | LoRa E32 TX |
| GP2 | GPIO | LoRa M0 |
| GP3 | GPIO | LoRa M1 |
| GP4 | PIO UART RX | PMS5003 TX |
| GP5 | PIO UART TX | PMS5003 RX |
| GP6 | GPIO | LoRa AUX |
| GP7 | PWM | MG90 signal |
| GP10 | SPI1 SCK | MicroSD SCK |
| GP11 | SPI1 TX | MicroSD MOSI |
| GP12 | SPI1 RX | MicroSD MISO |
| GP13 | GPIO | MicroSD CS |
| GP14 | I2C1 SDA | BH1750 #3 SDA |
| GP15 | I2C1 SCL | BH1750 #3 SCL |
| GP16 | GPIO | Onboard status LED |
| GP26 | I2C0 SDA | IMU + BMP280 + BH1750 #1/#2 |
| GP27 | I2C0 SCL | IMU + BMP280 + BH1750 #1/#2 |
| GP28 | Unassigned | Currently unused |
| GP29 | ADC3 | MQ135 analog output |

Pin definitions are centralized in:

```text
src/config.h
```

Change hardware assignments there instead of hardcoding GPIO numbers inside individual drivers.

---

# I2C Configuration

## I2C0

```text
RP2040 GP26 SDA ─────┬──── IMU
                     ├──── BMP280
                     ├──── BH1750 #1
                     └──── BH1750 #2

RP2040 GP27 SCL ─────┴──── Shared Clock
```

Configured bus speed:

```text
400 kHz
```

| Device | Address |
|---|---|
| IMU | `0x68` |
| BMP280 | `0x76` |
| L89HA GNSS command endpoint | `0x50` |
| BH1750 #1 | `0x23` |
| BH1750 #2 | `0x5C` |

## I2C1

```text
GP14 SDA ───── BH1750 #3 SDA
GP15 SCL ───── BH1750 #3 SCL
```

BH1750 #3 uses:

```text
0x23
```

---

# Ground Station Wiring

```text
             LoRa E32 433T3D                 ESP32

           ┌─────────────────┐          ┌────────────────┐
           │                 │          │                │
           │ TXD ────────────┼─────────►│ GPIO16 / RX2   │
           │ RXD ◄───────────┼──────────│ GPIO17 / TX2   │
           │                 │          │                │
           │ M0  ◄───────────┼──────────│ GPIO25         │
           │ M1  ◄───────────┼──────────│ GPIO26         │
           │ AUX ────────────┼─────────►│ GPIO27         │
           │                 │          │                │
           │ GND ────────────┼──────────│ GND            │
           └─────────────────┘          └───────┬────────┘
                                                │
                                                │ USB
                                                ▼
                                             Laptop
                                                │
                                                ▼
                                             GLANCE
```

LoRa UART:

```text
9600 baud
```

ESP32 USB serial:

```text
115200 baud
```

A common ground must be maintained between connected hardware.

---

# Software Architecture

The flight software is divided into three main layers:

```text
┌───────────────────────────────────────────────────────┐
│                    PAYLOAD                            │
│                                                       │
│ Particulate │ Air Quality │ Light Fusion │ Wind Data  │
├───────────────────────────────────────────────────────┤
│                 SYSTEM MODULES                        │
│                                                       │
│ Sensor Manager │ Flight State │ Recovery              │
│ Telemetry      │ Data Logger                          │
├───────────────────────────────────────────────────────┤
│                HARDWARE DRIVERS                       │
│                                                       │
│ IMU     │ BMP280 │ BH1750 │ PMS5003 │ MQ135          │
│ L89HA   │ LoRa   │ SD      │ Servo                   │
├───────────────────────────────────────────────────────┤
│                   RP2040                              │
│                                                       │
│        I2C │ UART │ PIO │ SPI │ ADC │ PWM            │
└───────────────────────────────────────────────────────┘
```

## Drivers

`src/drivers/` contains hardware-specific interfaces.

Drivers are responsible for communicating with devices and exposing measurements to the rest of the system.

They do not contain mission decisions such as parachute deployment or flight-state transitions.

## Sensor Manager

The Sensor Manager coordinates sensor acquisition and maintains the latest valid sensor measurements, timestamps and health information.

Configured application sampling rates:

| Sensor | Sampling Rate |
|---|---:|
| IMU | 50 Hz |
| BMP280 | 10 Hz |
| BH1750 | 4 Hz |
| PMS5003 | 1 Hz |
| MQ135 | 1 Hz |
| L89HA | 1 Hz |

## System Modules

`src/modules/` contains system-level behaviour:

- flight-state estimation
- recovery
- telemetry
- data logging
- sensor management

## Payload

`src/payload/` contains mission-specific processing:

- particulate profiling
- air-quality processing
- light fusion
- wind-data inference

---

# Repository Structure

```text
ConscientiaSoftware/
│
├── ground_station/
│   ├── esp32/
│   │   └── ground_station/
│   │       └── ground_station.ino
│   └── Conscientia_GroundStation.json
│
├── sil/
│   ├── host/
│   ├── simulate_flight.py
│   ├── test_simulated_flight.py
│   ├── test_production_sil.py
│   ├── replay_to_glance.py
│   ├── generate_sd_logs.py
│   └── plot_sd_logs.py
│
├── src/
│   ├── drivers/
│   ├── modules/
│   ├── payload/
│   ├── lib/
│   ├── config.h
│   └── main.c
│
├── CMakeLists.txt
├── requirements.txt
├── LICENSE
└── README.md
```

---

# Development Environment

The RP2040 firmware uses the **Raspberry Pi Pico SDK**.

It does not use the Arduino framework on the flight computer.

The development environment used during software development was:

| Tool | Version |
|---|---|
| Pico SDK | 2.3.1 |
| ARM GNU Toolchain | 15.2.Rel1 |
| Ninja | 1.13.2 |
| CMake | 4.3.4 |
| picotool | 2.3.1 |

These versions document the development environment and are not necessarily strict minimum requirements.

---

# Pico SDK Setup

## 1. Install Required Tools

Install:

- Git
- CMake
- Ninja
- ARM GNU Embedded Toolchain
- Raspberry Pi Pico SDK

The **Raspberry Pi Pico VS Code extension** may also be used to install and manage the SDK/toolchain.

## 2. Clone the Repository

```powershell
git clone https://github.com/teamignitionvitc/ConscientiaSoftware.git
cd ConscientiaSoftware
```

The repository must be accessible through the user's GitHub account if it is private.

## 3. Configure Pico SDK

If the SDK is installed manually:

```powershell
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
```

Set the SDK path:

```powershell
$env:PICO_SDK_PATH = "C:\path\to\pico-sdk"
```

If the Pico VS Code extension manages the SDK, use the SDK/toolchain environment configured by the extension.

---

# Build the Flight Firmware

From the repository root:

```powershell
mkdir build
cd build
cmake -G Ninja ..
cmake --build .
```

For subsequent builds from the repository root:

```powershell
cmake --build build
```

The target board is:

```text
waveshare_rp2040_zero
```

A successful incremental Ninja build may report:

```text
ninja: no work to do.
```

This means the current source is already compiled.

---

# Flash the RP2040-Zero

1. Disconnect the RP2040-Zero from USB.
2. Hold the **BOOTSEL** button.
3. Connect the board to the computer.
4. Release BOOTSEL.
5. The RP2040 should appear as a USB mass-storage device.
6. Locate the generated `.uf2` file in the build output.
7. Copy the `.uf2` file to the RP2040 drive.
8. The board automatically reboots and starts the firmware.

---

# ESP32 Ground Station Setup

The ESP32 ground station firmware is located at:

```text
ground_station/esp32/ground_station/ground_station.ino
```

## Upload

1. Open the sketch in Arduino IDE.
2. Select the correct ESP32 board.
3. Select the ESP32 serial port.
4. Connect the LoRa module according to the ground-station wiring diagram.
5. Compile the sketch.
6. Upload it to the ESP32.
7. Connect the ESP32 to the ground-station laptop through USB.

The ESP32 receives complete LoRa packets, validates them, removes the packet framing/checksum and forwards the 28 numeric values over USB serial.

---

# Flight Sequence

The primary flight-state machine is:

```text
             POWER ON
                 │
                 ▼
           ┌────────────┐
           │ PRE_LAUNCH │
           └─────┬──────┘
                 │
            launch detected
                 │
                 ▼
            ┌────────┐
            │ ASCENT │
            └────┬───┘
                 │
           apogee / descent
              detected
                 │
                 ▼
           ┌─────────┐
           │ DESCENT │
           └────┬────┘
                 │
          landing detected
                 │
                 ▼
            ┌────────┐
            │ LANDED │
            └────────┘
```

The flight-state machine and parachute recovery system are intentionally separate.

---

## Telemetry Fields

| Index | Parameter | Unit / Meaning |
|---:|---|---|
| 0 | Sequence | packet count |
| 1 | Timestamp | ms |
| 2 | Temperature | °C |
| 3 | Pressure | Pa |
| 4 | Altitude | m |
| 5 | Acceleration X | g |
| 6 | Acceleration Y | g |
| 7 | Acceleration Z | g |
| 8 | Gyroscope X | °/s |
| 9 | Gyroscope Y | °/s |
| 10 | Gyroscope Z | °/s |
| 11 | Light 1 | lux |
| 12 | Light 2 | lux |
| 13 | Light 3 | lux |
| 14 | Fused Light | lux |
| 15 | PM1.0 | µg/m³ |
| 16 | PM2.5 | µg/m³ |
| 17 | PM10 | µg/m³ |
| 18 | MQ135 Raw | ADC |
| 19 | Air-Quality Response | relative % |
| 20 | Latitude | degrees |
| 21 | Longitude | degrees |
| 22 | GNSS Altitude | m |
| 23 | Ground Speed | m/s |
| 24 | Satellites | count |
| 25 | Flight State | state ID |
| 26 | Recovery Deployed | boolean |
| 27 | Validity Mask | sensor validity |

Flight-state IDs:

```text
0 = PRE_LAUNCH
1 = ASCENT
2 = DESCENT
3 = LANDED
```

---

# Live Telemetry Flow

```text
RP2040
   │
   │ 28-field telemetry packet
   ▼
Airborne LoRa E32
   │
   │ 433 MHz
   ▼
Ground LoRa E32
   │
   ▼
ESP32
   │
   │ checksum validation
   │ packet framing removed
   ▼
USB Serial
   │
   ▼
Laptop
   │
   ▼
GLANCE
```

The LoRa link sends a **CSV-formatted stream**, not physical `.csv` files.

---

# GLANCE Setup

The supplied GLANCE project configuration is:

```text
ground_station/Conscientia_GroundStation.json
```

Load this project into GLANCE instead of manually recreating all telemetry parameters where possible.

For live hardware operation, select the serial connection corresponding to the ESP32.

Configure the connection for the ground-station output:

```text
Connection: Serial
Baud Rate:  115200
Format:     CSV
Channels:   28
```

The dashboard can display:

- temperature (IMU die temperature)
- pressure
- altitude
- acceleration
- angular velocity
- light measurements
- particulate concentrations
- air-quality response
- GNSS data
- ground speed
- satellite count
- flight state
- recovery status

## GNSS Map

The Map widget uses:

```text
Latitude  → index 20
Longitude → index 21
```

With a valid L89HA fix, the map tracks the transmitted CanSat position.

---

# MicroSD Logging

The MicroSD system stores actual `.csv` files onboard the CanSat.

The card should be prepared with a supported FAT filesystem before integration.

The RP2040 communicates with the SD module using SPI1.

During flight:

```text
Sensors
   │
   ▼
Sensor Manager
   │
   ▼
Data Logger
   │
   ▼
FatFs
   │
   ▼
MicroSD
```

The SD path operates independently of LoRa telemetry.

If radio reception is interrupted, locally logged data can therefore remain available after recovery.

---

# Generated Flight Data

The logger produces the following datasets:

| File | Data |
|---|---|
| `altitude_time.csv` | Altitude vs time |
| `pressure_alt.csv` | Pressure vs altitude |
| `temp_alt.csv` | Temperature vs altitude |
| `accel_x_alt.csv` | Acceleration X vs altitude |
| `accel_y_alt.csv` | Acceleration Y vs altitude |
| `accel_z_alt.csv` | Acceleration Z vs altitude |
| `gyro_x_alt.csv` | Gyroscope X vs altitude |
| `gyro_y_alt.csv` | Gyroscope Y vs altitude |
| `gyro_z_alt.csv` | Gyroscope Z vs altitude |
| `pm1_alt.csv` | PM1.0 vs altitude |
| `pm25_alt.csv` | PM2.5 vs altitude |
| `pm10_alt.csv` | PM10 vs altitude |
| `airqual_alt.csv` | Air-quality response vs altitude |
| `light_alt.csv` | Fused light intensity vs altitude |
| `windspd_alt.csv` | Wind-speed estimate vs altitude |
| `winddir_alt.csv` | Wind-direction estimate vs altitude |
| `gnss_track.csv` | GNSS trajectory |

The logging design follows the principle:

```text
One CSV dataset → One primary post-flight graph
```

with `gnss_track.csv` serving as the mission trajectory dataset.

---

# Retrieving Flight Data

After the CanSat is recovered:

```text
Recover CanSat
      │
      ▼
Power Down
      │
      ▼
Remove MicroSD
      │
      ▼
Insert into Laptop
      │
      ▼
Copy Flight CSV Files
      │
      ▼
Post-Flight Analysis
```

Do not modify the original flight CSV files before archiving them.

Keep the raw mission data and perform filtering, smoothing or other processing on copies or during plotting.

---

# Post-Flight Graphs

The repository includes:

```text
sil/plot_sd_logs.py
```

for plotting SD-style flight datasets.

Install Python dependencies if required:

```powershell
pip install -r requirements.txt
```

Place/use the flight CSV data in the input location expected by the plotting script and run:

```powershell
python sil\plot_sd_logs.py
```

The script generates the corresponding post-flight graphs from the logged datasets.

For mission records, preserve both:

```text
RAW CSV DATA
+
GENERATED ANALYSIS / GRAPHS
```

---

# Payload Data

## Particulate Matter

The PMS5003 provides atmospheric:

```text
PM1.0
PM2.5
PM10
```

These measurements are recorded independently against altitude.

---

## Air Quality

The MQ135 ADC measurement is converted into sensor resistance:

```text
Rs = RL × (Vcc - Vsensor) / Vsensor
```

The current payload software uses the configured `R0` to produce a relative air-quality response.

The result is a **relative experimental response**.

It must not be interpreted as an official AQI, calibrated CO₂ measurement or calibrated gas concentration without appropriate physical calibration.

---

## Light Measurement

Three BH1750 sensors provide illumination measurements from different orientations.

Current fusion:

```text
3 valid sensors → average of all 3
2 valid sensors → average of valid 2
1 valid sensor  → use valid reading
0 valid sensors → fused value invalid
```

Individual sensor values are also transmitted.

Physical placement and parachute-shadow effects should be evaluated before final flight.

---

## Wind Data

The current system uses GNSS ground-motion information as a first-order wind-related estimate.

Gyroscope measurements are used to evaluate motion quality.

This is experimental mission data and should not be treated as a precision direct measurement of atmospheric wind without further physical validation.

---

# Pre-Flight Integration

Before flight, verify each subsystem independently.

## Power

- Verify module supply voltages.
- Verify common grounds.
- Check regulator/current capacity.
- Do not power the complete system until individual module requirements are confirmed.

## IMU

- Verify I2C communication at address `0x68`.
- Current physical device reports `WHO_AM_I = 0x70`; confirm the exact IMU model before flight qualification.
- Confirm sensor orientation.
- Confirm approximately 1 g magnitude while stationary.
- Confirm acceleration changes during controlled movement.
- Validate the release threshold using safe physical drop tests.

## BMP280

- Verify chip detection.
- Compare pressure with a reasonable local reference.
- Confirm altitude responds to vertical movement.

## BH1750

- Verify all three sensors independently.
- Confirm the two devices sharing I2C0 have different addresses.
- Verify the third sensor on I2C1.
- Test expected illumination orientations.

## PMS5003

- Confirm valid frames are received.
- Confirm checksum validation.
- Verify PM fields update.

## MQ135

- Verify ADC response.
- Determine/calibrate the required baseline `R0`.
- Do not rely on the provisional development baseline for scientific interpretation.

## L89HA

Test outdoors with sufficient sky visibility.

Confirm:

- valid GNSS fix
- latitude
- longitude
- altitude
- satellite count
- ground speed/course when moving

## MicroSD

Before flight:

- verify card detection
- verify filesystem mount
- write test data
- safely stop/unmount or power down
- verify the files can be read on a computer

## LoRa

Verify both E32 modules use compatible radio configuration.

Perform:

1. bench communication test
2. short-distance wireless test
3. expected operational range test
4. complete telemetry test through the ESP32 and GLANCE

## Servo / Recovery

Before attaching the real recovery mechanism:

1. verify servo direction
2. verify stowed position
3. verify deployed position
4. verify mechanical travel is unobstructed
5. verify the deployment command only needs to occur once
6. perform controlled recovery-system tests

Do not rely solely on software simulation for the final deployment configuration.

---

# Development / SIL

The `sil/` directory contains host-side simulation and validation utilities used during software development.

These tools are **not required for normal CanSat hardware operation**.

Useful development commands include:

```powershell
python sil\simulate_flight.py
python sil\test_simulated_flight.py
python sil\test_production_sil.py
```

A development-only telemetry replay utility is also available:

```powershell
python sil\replay_to_glance.py
```

This allows the GLANCE configuration to be tested without flight hardware.

Generated simulation data under:

```text
sil/data/
```

is development output and is excluded from version control.

---

# Configuration

Hardware pins, sampling rates, recovery thresholds and other system-level constants are centralized in:

```text
src/config.h
```

Important mission parameters should be changed there rather than duplicated throughout the source.

Several values require final physical validation, particularly:

- recovery low-g threshold
- recovery confirmation count
- servo pulse limits
- servo deployment position
- MQ135 baseline
- light-sensor physical orientation
- radio configuration

---

# License

This project is licensed under the **MIT License**.

See `LICENSE` for the complete license text.

**Team Ignition, VIT Chennai**

© 2026 Team Ignition VIT Chennai  
© Copyright of Ignition Software Department
