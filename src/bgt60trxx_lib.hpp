/// Library for BGT60TRXX-Sensor on PSOC6-Boards
#ifndef BGT60TRXX_LIB
#define BGT60TRXX_LIB

#include <Arduino.h>
#include "bgt60trxx_define.hpp"
#include "arduinoFFT.h"

#define BGT_ERROR 0
#define BGT_SUCCESS 1

typedef struct
{
    size_t word_size;
    size_t frame_size;
    byte headerGSR0[DATA_SIZE];
    byte regData[DATA_SIZE];
    reg_file register_values;

    //delete has to be manually called!
    byte* data;
    float* vReal;
    float* vImag;

    voidFuncPtr interrupt_handler;

    size_t start_freq;
    size_t Clk_Per_Chirp;    
    size_t step_freq_chirp;
    size_t adc_div;
    ArduinoFFT<float> FFT;
} bgt60trxx_struct;

typedef bgt60trxx_struct const * const constBGT_ptr;
typedef bgt60trxx_struct * const BGT_ptr;

/* 
    Implementation of an BGT60-Radar sensor using one antenna.
    Creates a dynamic instance, which needs to be deleted by user!
*/
bgt60trxx_struct* initStruct(
    size_t const word_size, 
    voidFuncPtr interrupt_handler
);

/*
    Frees all Memory of Struct
    Returns BGT_ERROR on Error
*/
size_t deinitStruct(BGT_ptr sensor);

/*
    Returns meter per index.
    Is based from parameters of chirp config
*/
float get_range_resolution(constBGT_ptr sensor);

/*
    Read register of BGT60TR-Sensor
*/
size_t read_reg(BGT_ptr sensor, size_t const reg_addr);

/*
    Write register of BGT60TR-Sensor with given Data
    Returns BGT_ERROR on Error
*/
size_t write_reg(
    constBGT_ptr sensor, 
    size_t const reg_addr, 
    size_t const data
);

/*
    sets frequency divider of adc. Only enabled with InitSensor-Method
    Returns BGT_ERROR on Error
*/
size_t set_adc_div(BGT_ptr sensor, size_t const data);

/*
    sets chirp length of sensor. Only enabled with InitSensor-Method
    Returns BGT_ERROR on Error
*/
size_t set_chirp_len(BGT_ptr sensor, size_t const chirp_len);

/*
    Configures chirp parameters. 
    Only enabled with InitSensor-Method.

    N_FSU = Starting Frequency
    N_RTU = Clock cycles per chirp
    N_RSU = Frequency step per clock cycle

    for calculation of those values see Datasheet
    Returns BGT_ERROR on Error
*/ 
size_t configure_chirp(
    BGT_ptr sensor, 
    size_t const N_FSU, 
    size_t const N_RTU, 
    size_t const N_RSU
);

/*
    Amplifier for recieved signal (Channel 1) from sensor. 
    Values from 0 to 5 are allowed (see Datasheet).
    Only enabled with InitSensor-Method.
    Returns BGT_ERROR on Error
*/
size_t set_vga_gain_ch1(BGT_ptr sensor, size_t const gain);

/*
    Sets register values for sensor
    Only enabled with InitSensor-Method.
    Returns BGT_ERROR on Error
*/
size_t set_init_value(
    BGT_ptr sensor, 
    size_t const data, 
    size_t const address, 
    size_t const reset_mask, 
    size_t const offset
);


/*
    sets compare value using a read-modify-write
    Only enabled with a InitSensor call
    Returns BGT_ERROR on Error
*/
size_t setCompareValue(BGT_ptr sensor, size_t const compare_value);

/*
    Enables Test-Mode (LFSR Enable) for Sensor
    Returns BGT_ERROR on Error
*/
size_t enableTestMode(BGT_ptr sensor);

/*
    Start frame generation and leave Deep-Sleep-Mode
    Returns BGT_ERROR on Error
*/
size_t startFrame(BGT_ptr sensor);

/*
    Read n-words from FIFO-Stack.
    Checks Header-Information for Error (like Overflow/Underflow).
    Data is stored inside self.data object
    Returns BGT_ERROR on Error
*/
size_t readFifo(BGT_ptr sensor);

/*
    Reads FIFO and
    save word-wise inside self.fft.re.
    Then runs High-Pass-Filter.
    Using the FFT, the range-profile is calculated and
    stored inside self.fft.re and self.fft.im
    Returns BGT_ERROR on Error
*/
size_t readDistance(BGT_ptr sensor);

/*
    inits sensor
    and writes all registers anew.
    Returns BGT_ERROR on Error
*/
size_t initSensor(BGT_ptr sensor);

/*
    Resets fifo and fsm
    Returns BGT_ERROR on Error
*/
size_t resetFIFO(BGT_ptr sensor);

/*
    Resets fsm
    Returns BGT_ERROR on Error
*/
size_t resetFSM(BGT_ptr sensor);

/*
    Resets software, fifo and fsm
    Returns BGT_ERROR on Error
*/
size_t reset(BGT_ptr sensor);

/*
    Calculates Clock cycles per chirp.
    Needs division factor for sample rate (adc) and
    how many samples per chirp are needed.
*/
size_t calculateRTU(size_t const adc_div, size_t const samples_per_chirp);

/*
    Calculates value for Starting Frequency. 
    start_freq needs to be in kHz.
*/
size_t calculateFSU(size_t const start_freq);

/*
    Calculates Frequency step per clock cycle.
    Needs Starting Frequency as well as wanted bandwidth
*/
size_t calculateRSU(size_t const bandwidth, size_t const RTU);

/*
    Check Data for overflow and underflow.
    Returns BGT_ERROR on Error
*/
size_t checkData(float const* const data, size_t const length);

#endif