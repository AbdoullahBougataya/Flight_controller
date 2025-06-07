<div align='center'><img src='https://github.com/AbdoullahBougataya/Flight_controller/blob/main/img/Quadimg.jpg' alt="Quadcopter" width="400" height="400" style="display: block; margin: 0 auto"/></div>

> [!TIP]
> This codebase is only compatible with dual core ESP32s

# Flight Controller
## Overview

This repository is the software part of a DIY flight controller. It's for people who want to build their own flight controller. This project is based on what we've learned (and what we're still learning) as Control & Automation engineering students. **The software has features for flight stabilization, sensor integration, PID control, and signal processing**. Compatible with ESP32 based quadcopter drones.

## Project structure
```
/Flight_controller
├── /Datasheet
│     └── Sensors
│          └── BMI160 Datasheet.pdf
├── /include
│     ├── BMI160.h
│     ├── RCFilter.h
│     └── README
├── /src
│     ├── PID.c
│     ├── BMI160.c
│     └── RCFilter.c
├── /main
│     ├── CMakeLists.txt
│     └── main.c
├── LICENSE
├── CMakeLists.txt
└── README.md --> You are in this file
```

## Features

- Real-time flight stabilization using **PID control**.
- Supports the **BMI160 IMU** (**I**nertial **M**easurement **U**nit) sensor.
- Supports the **BMP390 Barometric sensor** (Altimeter).
- Communication via **standard RC protocols** (PPM/PWM).
- Simple sensor communications **using I²C**.

## Getting Started

Follow these steps to set up and run the Flight Controller software.

### Prerequisites

- [ESP IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)
- Required hardware:
  - Microcontroller (e.g., [Arduino UNO](https://store.arduino.cc/products/arduino-uno-rev3), [ESP32-S3](https://www.espressif.com/en/products/socs/esp32-s3)...)
  - [BMI160](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi160/) IMU
  - [BMP280](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/pressure-sensors-bmp280.html) Barometric sensor
  - GPS module (optional)
  - RC transmitter and receiver

### Installation

1. Install **ESP IDF** following the [installation guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html#installation).
2. Clone this repository:
   ```bash
   git clone https://github.com/AbdoullahBougataya/Flight_controller.git
   ```
3. Open the project in your preferred IDE.
4. Build and flash the code to your microcontroller.

## Hardware Configuration

_Waiting for the image_

## Usage

1. Connect the hardware components as per the [Hardware Configuration](#Hardware-Configuration).
2. Power on the flight controller and **wait 8 seconds** for the sensors and the ESCs to calibrate.
3. Use your RC transmitter to control the drone.

## Contributing

Contributions are welcome! Please follow these steps:

1. **Fork** the repository.
2. Create a new branch for your *feature* or *bugfix*.
3. **Submit a pull request** with a detailed description of your changes.

## License

This project is licensed under the **GPL v2.0 License**. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **Carbon aeronautics** series on making a Quadcopter using Teensy (Arduino compatible).
- **Phil's Lab** series on DSP using STM32.
- **void loop Robotech & Automation** videos on ESP IDF programming.
- **DroneBot Workshop** videos on PWM communication.
