# Atmospheric Data Acquisition — Weather Balloon

Embedded firmware and custom PCB design for an atmospheric measurement payload, developed as a school project in Luxembourg.

**My contribution:** firmware development, PCB design and technical project delivery. The project brought together sensor integration, data logging, electronics assembly and launch preparation.

## Flight outcome

The original team log records a launch from Redange on **14 September 2024**, followed by recovery near Verdun, France, roughly two hours later. The payload, camera and SD card were recovered intact. The flight dataset is not included in this repository.

## Engineering work

- Integrated environmental and motion sensors using I²C and SPI.
- Developed Arduino sketches for individual sensor checks and combined acquisition.
- Implemented timestamped SD-card logging and LED status indications.
- Designed the data-logger schematic and board layout in EAGLE.

## Start here

| Area | Files | What to review |
| --- | --- | --- |
| Combined acquisition | [`all_in_one.ino`](Prep_launch/Module_software_tests/all_in_one/all_in_one.ino) | Environmental sensors, IMU, ozone, gamma sensor, PT100, RTC and SD integration |
| Smaller logger variant | [`Final_Code.ino`](Prep_launch/Final%20Code/Final_Code.ino) | BMP280 + MPU6050 acquisition and CSV-style logging |
| Hardware | [PCB files](Prep_launch/PCB) | EAGLE `.sch` schematic and `.brd` layout |
| Component experiments | [Module tests](Prep_launch/Module_software_tests) | Isolated checks and calibration sketches |

## Hardware and dependencies

The repository contains several hardware revisions. Select dependencies for the sketch you are examining; the sketches are not interchangeable.

| Sketch | Components / libraries |
| --- | --- |
| `all_in_one.ino` | Adafruit MS8607, Adafruit Unified Sensor, DFRobot OzoneSensor, Arduino LSM9DS1, Adafruit MAX31865, RTClib, SD, Wire and SPI; GDK101 access is implemented in the sketch |
| `Final_Code.ino` | MPU6050_light, Adafruit BMP280 and its dependencies, SD, Wire and SPI |

Use the Arduino IDE and a board core compatible with the selected hardware. Board selection, library versions, wiring and calibration must be matched to the original build; a reproducible build configuration is not yet recorded.

## Working with the code

1. Start with the individual [module tests](Prep_launch/Module_software_tests) to understand each peripheral.
2. Install the libraries used by your chosen sketch.
3. Match sensor addresses, chip-select pins and LED pins to the board schematic.
4. Open the sketch in the Arduino IDE, keeping any companion `.h` and `.cpp` files together.
5. Check serial output and SD logging on the bench before combining peripherals.

The folder name `Final Code` describes a historical revision. Its sketch uses BMP280 and MPU6050 sensors; the broader acquisition sketch lives under `Module_software_tests/all_in_one`.

## Project status

This repository preserves the project firmware and PCB sources. It includes experiments as well as integrated sketches, rather than a single packaged application. Measured sensor accuracy, exact dependency versions and flight-data analysis are not documented here.

## Team and acknowledgements

- **Ben Lies:** programming, PCB design and technical project delivery.
- **Ben Kasel:** mentoring, organisation, communication and troubleshooting.
- **Christophe Mayers:** soldering, project support and the original written project log.

The original log also mentions additional participants without naming them. This list reflects the credits currently documented in the repository.

[Original project log](https://github.com/BigblenHD/Weather-balloon/blob/688e36f4c935f01a88b4a0122774435af6ce9a63/README.md) · [Ben's portfolio](https://benlies.com)
