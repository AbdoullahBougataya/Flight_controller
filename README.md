# Flight Controller

[![Build Arduino Sketch](https://github.com/AbdoullahBougataya/Flight_controller/actions/workflows/main.yml/badge.svg)](https://github.com/AbdoullahBougataya/Flight_controller/actions/workflows/main.yml)

> [!TIP]
> This codebase is compatible with all Arduino compatible boards

## Overview

This repository has the software part of a DIY flight controller. It's for people who want to build their own flight controller. This project is based on what we've learned (and what we're still learning) as Control & Automation engineering students. The software has features for flight stabilization, sensor integration, PID control, and signal processing. Compatible with quadcopter drones.

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

- Real-time flight stabilization using PID control
- Supports the BMI160 IMU (Inertial Measurement Unit) sensor.
- Supports the BMP390 Barometric sensor (Altimeter).
- Communication via standard RC protocols (PPM).
- Simple sensor communications using I²C.
- Integration with GPS modules for navigation.
- Cross-platform compatibility with Arduino boards.

## Getting Started

Follow these steps to set up and run the Flight Controller software.

### Prerequisites

- Arduino IDE
- Required hardware:
  - Microcontroller (e.g., Arduino UNO, ESP32-S3...)
  - BMI160 IMU
  - GPS module (optional)
  - RC transmitter and receiver

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/AbdoullahBougataya/Flight_controller.git
   ```
2. Open the project in your preferred IDE.
3. Install the Wire and the FreeRTOS libraries.
4. Compile and upload the code to your microcontroller.

## Usage

1. Connect the hardware components as per the [hardware setup guide](#link-to-hardware-setup-guide).
2. Power on the flight controller and wait 5 seconds for the sensors to calibrate.
3. Use your RC transmitter to control the drone.
