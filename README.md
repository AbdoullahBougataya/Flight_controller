# Flight Controller

[![Build Arduino Sketch](https://github.com/AbdoullahBougataya/Flight_controller/actions/workflows/main.yml/badge.svg)](https://github.com/AbdoullahBougataya/Flight_controller/actions/workflows/main.yml)

> [!TIP]
> This codebase is compatible with all Arduino compatible boards

## Overview

This repository has the software part of a DIY flight controller. It's for people who want to build their own flight controller. This project is based on what we've learned (and what we're still learning) as Control & Automation engineering students. *The software has features for flight stabilization, sensor integration, PID control, and signal processing*. Compatible with quadcopter drones.

## Project structure
``` sh
/Flight_controller
├── /include
│     ├── BMI160.h
│     ├── RCFilter.h
│     └── README
├── /src
│     ├── BMI160.cpp
│     └── RCFilter.cpp
├── Flight_controller.ino
├── LICENSE
├── README.md
├── /lib
└── /test
```

## Features

- Real-time flight stabilization using *PID control*.
- Supports the *BMI160 IMU* (Inertial Measurement Unit) sensor.
- Supports the *BMP390 Barometric sensor* (Altimeter).
- Communication via *standard RC protocols* (PPM).
- Simple sensor communications *using I²C*.
- Integration with *GPS modules for navigation*.
- Cross-platform compatibility with *Arduino boards*.

## Getting Started

Follow these steps to set up and run the Flight Controller software.

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software)
- Required hardware:
  - Microcontroller (e.g., [Arduino UNO](https://store.arduino.cc/products/arduino-uno-rev3), [ESP32-S3](https://www.espressif.com/en/products/socs/esp32-s3)...)
  - [BMI160](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi160/) IMU
  - [BMP390](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/pressure-sensors-bmp390.html) Barometric sensor
  - GPS module (optional)
  - RC transmitter and receiver

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/AbdoullahBougataya/Flight_controller.git
   ```
2. Open the project in your preferred IDE.
3. Install the [Wire](https://docs.arduino.cc/language-reference/en/functions/communication/wire/) and the [FreeRTOS](https://docs.arduino.cc/libraries/freertos/) libraries.
4. Compile and upload the code to your microcontroller.

## Hardware Configuration

_Waiting for the image_

## Usage

1. Connect the hardware components as per the [Hardware Configuration](#Hardware Configuration).
2. Power on the flight controller and *wait 5 seconds* for the sensors to calibrate.
3. Use your RC transmitter to control the drone.

## Contributing

Contributions are welcome! Please follow these steps:

1. *Fork* the repository.
2. Create a new branch for your *feature* or *bugfix*.
3. *Submit a pull request* with a detailed description of your changes.

## License

This project is licensed under the *GPL v2.0 License*. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- *Carbon aeronautics* series on making a Quadcopter using Teensy (Arduino compatible).
- *Phil's Lab* series on DSP using STM32.

