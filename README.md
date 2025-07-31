# Arduino Driver for 60 GHz Radar BGT60TRxx
With this library a XENSIV™ BGT60TRxx 60 GHz radar sensor can be configured
and used with Arduino.

> [!NOTE]  
> This project is work in progress and not covering all functions
of the sensor yet.   
> If you are missing any functionality feel free to [contribute](https://github.com/Infineon/arduino-radar-bgt60tr13/fork) or [open an issue](https://github.com/Infineon/arduino-radar-bgt60tr13/issues).

## Overview
The [XENSIV™ BGT60TRxx](https://www.infineon.com/cms/en/product/sensor/radar-sensors/radar-sensors-for-iot/60ghz-radar/)
is a 60 GHz radar sensor developed by Infineon, designed for advanced sensing applications.

It supports detection ranges of up to 15 meters and features low power consumption,
making it well-suited for a wide range of IoT use cases.

Typical use cases:
 - Presence Detection/Segmentation
 - Touchless Interaction
 - Vital Sensing

## Dependencies
This module depends on the [arduinoFFT module](https://github.com/kosme/arduinoFFT),
written by kosme.

## Installation of this Module
TODO

## Usage

### Things to consider when using this Library 
- The `readFifo` function can only transmit 8192 words,
which consist of 24 data bits
    - Maximum transmission possible: 24.576 bytes
- The chip returns an error when reading while the stack is full or empty
- Data can be checked for overflow or underflow errors using the `checkData` function
