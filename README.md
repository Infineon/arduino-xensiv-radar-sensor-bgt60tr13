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

### Arduino IDE Library Manager
Use the Arduino Library Manager to install the 'bgt60trxx_radar' library.

### Arduino IDE Manual Installation
Download the desired .zip library version from the repository [releases](https://github.com/Infineon/arduino-radar-bgt60tr13/releases) section.

> [!WARNING]  
> As a general recommendation, downloading directly from the master branch should be avoided. Even though it should not, it could contain incomplete or faulty code.

## Usage

### Things to consider when using this Library 
- The `readFifo` function can only transmit 8192 words,
which consist of 24 data bits
    - Maximum transmission possible: 24.576 bytes
- The chip returns an error when reading while the stack is full or empty
- Data can be checked for overflow or underflow errors using the `checkData` function
- Currently, only one of three receiver antennas are implemented.


### Example Code
```c++
// import Modul
#include "bgt60trxx_lib.hpp"

/// configuration values for chip
const size_t no_of_chirps = 1;
const size_t samples_per_chirp = 128;
const size_t words = samples_per_chirp * no_of_chirps;
const size_t ADC_DIV = 60;
const size_t start_freq = 62500000;  // in kHz
const size_t bandwidth  =  2000000;    // in kHz

/// initialise sensor struct
bgt60trxx_struct* bgt60trxx_sensor;

void setup() 
{
    ...
    // initialize sensor
    bgt60trxx_sensor = initStruct(words, (void*)interrupt_handler);
    
    set_adc_div(bgt60trxx_sensor, ADC_DIV);
    set_chirp_len(bgt60trxx_sensor, samples_per_chirp);
    
    size_t FSU = calculateFSU(start_freq);
    size_t RTU = calculateRTU(ADC_DIV, samples_per_chirp);
    size_t RSU = calculateRSU(bandwidth, RTU);
    
    configure_chirp(bgt60trxx_sensor, FSU, RTU, RSU);
    set_vga_gain_ch1(bgt60trxx_sensor, 3);
    initSensor(bgt60trxx_sensor);
    
    // start measuring
    startFrame(bgt60trxx_sensor);
}

void loop() 
{
    // read Data from sensor
    // data is available in array bgt60trxx_sensor->vReal
    // Length of array is bgt60trxx_sensor->word_size / 2
    readDistance(bgt60trxx_sensor);

    // start next measurment
    resetFIFO(bgt60trxx_sensor); // if any error, resets chip
    startFrame(bgt60trxx_sensor);
}
```