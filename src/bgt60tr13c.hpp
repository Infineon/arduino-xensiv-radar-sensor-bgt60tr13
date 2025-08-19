#ifndef BGT60TR13C_LIB_HPP
#define BGT60TR13C_LIB_HPP

#include <Arduino.h>
#include <SPI.h>
#include "bgt60tr13c_regs.hpp"
#include "arduinoFFT.h"

enum class BGT_status : uint8_t {
    BGT_error = 0,
    BGT_success = 1
};

/**
 * @brief Overloaded operator to check if BGT_status is an error.
 */
inline bool operator!(BGT_status status) {
    return status == BGT_status::BGT_error;
}

/**
 * @brief Structure representing the BGT60TRXX sensor.
 */
struct bgt60trxx_struct
{
    // sensor data
    size_t word_size;
    size_t frame_size;
    byte header_GSR0[DATA_SIZE];
    byte reg_data[DATA_SIZE];
    reg_file register_values;
    voidFuncPtr interrupt_handler;
    byte* data;

    arduino::HardwareSPI* radar_sensor_spi;

    // Chirp configuration
    size_t start_freq;
    size_t clk_per_chirp;    
    size_t step_freq_chirp;
    size_t adc_div;

    // FFT interface
    ArduinoFFT<float> FFT;
    float* vReal;
    float* vImag;
};

using BGT_const_ptr = bgt60trxx_struct const * const;
using BGT_ptr = bgt60trxx_struct * const;

/**
 * Implementation of an BGT60-Radar sensor using one antenna.
 * Creates a dynamic instance, which needs to be deleted by user!
 * 
 * @param word_size The size of one word in the sensor. 
 * @param interrupt_handler Pointer to the interrupt handler function.
 * @return Pointer to the initialized sensor structure.
 */
bgt60trxx_struct* init_struct(
    size_t const word_size, 
    voidFuncPtr interrupt_handler,
    arduino::HardwareSPI *spi_interface = &SPI
);

/**
 * Deinitializes the BGT60TRXX sensor structure.
 * Frees all allocated memory.
 * @param sensor Pointer to the sensor structure to deinitialize.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status deinit_struct(BGT_ptr sensor);

/**
 * Calculates the range resolution of the sensor.
 * The range resolution is calculated based on the sensor's chirp configuration.
 * @param sensor Pointer to the sensor structure.
 * @return The range resolution in meters per index.
 */
float get_range_resolution(BGT_const_ptr sensor);

/**
 * Reads a register from the BGT60TRXX sensor.
 * @param sensor Pointer to the sensor structure.
 * @param reg_addr The address of the register to read.
 * @return The value of the register, or BGT_error on failure.
 */
BGT_status read_reg(BGT_ptr sensor, size_t const reg_addr);

/**
 * Writes a value to a register in the BGT60TRXX sensor.
 * @param sensor Pointer to the sensor structure.
 * @param reg_addr The address of the register to write to.
 * @param data The data to write to the register.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status write_reg(
    BGT_const_ptr sensor, 
    size_t const reg_addr, 
    size_t const data
);

/*
    sets frequency divider of adc. Only enabled with InitSensor-Method
    Returns BGT_error on Error
*/
/**
 * Sets the ADC division factor for the sensor.
 * Only enabled with init_sensor method
 * @param sensor Pointer to the sensor structure.
 * @param data The ADC division factor to set.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status set_adc_div(BGT_ptr sensor, size_t const data);

/**
 * Sets the chirp length for the sensor.
 * Only enabled with init_sensor method
 * @param sensor Pointer to the sensor structure.
 * @param chirp_len The length of the chirp in samples.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status set_chirp_len(BGT_ptr sensor, size_t const chirp_len);

/**
 * Configures the chirp parameters for the sensor.
 * Only enabled with init_sensor method.
 * For calculation of parameters see Datasheet.
 * @param sensor Pointer to the sensor structure.
 * @param N_FSU The starting frequency in Hz.
 * @param N_RTU The clock cycles per chirp.
 * @param N_RSU The frequency step per clock cycle.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status configure_chirp(
    BGT_ptr sensor, 
    size_t const N_FSU, 
    size_t const N_RTU, 
    size_t const N_RSU
);

/**
 * Sets the VGA gain for channel 1 of the sensor.
 * Only enabled with init_sensor method.
 * @param sensor Pointer to the sensor structure.
 * @param gain The gain value to set (0-5).
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status set_vga_gain_ch1(BGT_ptr sensor, size_t const gain);

/**
 * Sets register values for sensor.
 * Only enabled with init_sensor method.
 * @param sensor Pointer to the sensor structure.
 * @param data The data to set in the register.
 * @param address The address of the register to set.
 * @param reset_mask The mask to reset bits in the register.
 * @param offset The offset for the data in the register.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status set_init_value(
    BGT_ptr sensor, 
    size_t const data, 
    size_t const address, 
    size_t const reset_mask, 
    size_t const offset
);

/**
 * Sets the compare value for the sensor using a read-modify-write.
 * If the compare value is greater than or equal to 100,
 * it is set to 99.
 * @param sensor Pointer to the sensor structure.
 * @param compare_value The compare value to set (0-99).
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status set_compare_value(BGT_ptr sensor, size_t const compare_value);

/**
 * Enables the test mode (LFSR Enable) for the sensor.
 * @param sensor Pointer to the sensor structure.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status enable_testmode(BGT_ptr sensor);

/**
 * Starts the frame generation for the sensor.
 * This function leaves the deep sleep mode and prepares the sensor for data acquisition.
 * @param sensor Pointer to the sensor structure.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status start_frame(BGT_ptr sensor);

/**
 * Reads data from the FIFO stack of the sensor.
 * Checks header information for errors (like overflow/underflow).
 * Data is stored inside the sensor's data object.
 * @param sensor Pointer to the sensor structure.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status read_fifo(BGT_ptr sensor);

/**
 * Reads the distance data from the sensor.
 * Unpacks the recorded ADC data into the real part of the FFT.
 * Runs a high-pass filter on the data before performing the FFT.
 * The resulting range profile is stored in the sensor's FFT structure.
 * @param sensor Pointer to the sensor structure.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status read_distance(BGT_ptr sensor);

/**
 * Initializes the sensor by writing all register values from the register file to the sensor.
 * @param sensor Pointer to the sensor structure.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status init_sensor(BGT_ptr sensor);

/**
 * Resets the FIFO and FSM of the sensor.
 * @param sensor Pointer to the sensor structure.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status reset_FIFO(BGT_ptr sensor);

/**
 * Resets the FSM of the sensor.
 * @param sensor Pointer to the sensor structure.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status reset_FSM(BGT_ptr sensor);

/**
 * This function resets the sensor's state, FIFO, and FSM.
 * @param sensor Pointer to the sensor structure.
 * @return BGT_success on success, BGT_error on failure.
 */
BGT_status reset(BGT_ptr sensor);

/**
 * Calculates the clock cycles per chirp.
 * This is based on the ADC division factor and the number of samples per chirp.
 * @param adc_div The ADC division factor.
 * @param samples_per_chirp The number of samples per chirp.
 * @return The calculated RTU value.
 */
size_t calculate_RTU(size_t const adc_div, size_t const samples_per_chirp);

/**
 * Calculates the starting frequency for the sensor.
 * This is based on the desired starting frequency in kHz.
 * @param start_freq The desired starting frequency in kHz.
 * @return The calculated FSU value.
 */
size_t calculate_FSU(size_t const start_freq);

/**
 * Calculates the frequency step per clock cycle.
 * This is based on the desired bandwidth and the RTU value.
 * @param bandwidth The desired bandwidth in Hz.
 * @param RTU The clock cycles per chirp.
 * @return The calculated RSU value.
 */
size_t calculate_RSU(size_t const bandwidth, size_t const RTU);

/**
 * Checks the data for overflow and underflow conditions.
 * This function iterates through the data array and checks for any anomalies.
 * @param data Pointer to the data array.
 * @param length The length of the data array.
 * @return BGT_success if no errors are found, BGT_error if errors are detected.
 */
BGT_status check_data(float const* const data, size_t const length);

/**
 * @brief Gets the FFT data calculated from the sensor.
 * @param sensor Pointer to the sensor structure.
 * @return Pointer to the FFT data array.
 */
float* get_fft_data(BGT_const_ptr sensor);

/**
 * @brief Gets the length of the FFT data.
 * @param sensor Pointer to the sensor structure.
 * @return The length of the FFT data.
 */
size_t get_fft_length(BGT_const_ptr sensor);

#endif // BGT60TRXX_LIB_HPP