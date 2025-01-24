# Flight Controller

> [!INFO]
> This codebase is compatible with all Arduino compatible boards

## Get started

To upload this code on your microcontroller, you should first install the [Arduino IDE](https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.4_Windows_64bit.exe), then open up the file `Flight_controller.ino`, plug your Arduino compatible board to your computer, select the COM port from the list above and select you board name from the list. After that, connect the SDA of the BMI160 to the SDA of your board and the SCL (Maybe SCK) to the corresponding SCL on the board and connect the SA0 to the Ground (GND)...After connecting all the wires to their corresponding pins. press the right arrow to upload the code to your board. After uploading the code to the board, open a serial monitor and set it up on 115200 Baud rate.

## Project structure
``` sh
/Flight_controller
├── /include
│     ├── BMI160.h
│     ├── FIRFilter.h
│     └── README
├── /src
│     ├── BMI160.cpp
│     └── FIRFilter.cpp
├── Flight_controller.ino
├── LICENSE
├── README.md
```

