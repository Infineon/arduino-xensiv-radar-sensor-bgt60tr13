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
 *   Redefine voidFuncPtr for
 *   Boards, which don't implement
 *   them.
 */ 
typedef void (*voidFuncPtr)();

/**
 * @brief Overloaded operator to check if BGT_status is an error.
 */
inline bool operator!(BGT_status status) {
    return status == BGT_status::BGT_error;
}

/**
 * @brief Class representing the BGT60TRXX sensor.
 */
class BGT60TRXX
{
private:
    // SPI Interface
    //==============================
    SPIClass* radar_sensor_spi;
    size_t pin_cs;
    
    // Transfer of SPI Interface
    //============================
    size_t word_size;
    size_t frame_size;
    
    // Buffers for performance
    byte header_GSR0[DATA_SIZE];
    byte reg_data[DATA_SIZE];
    byte* data;
    
    // Load default register configuration
    reg_file register_values;

    // Interrupt Handler
    voidFuncPtr interrupt_handler;

    // Chirp configuration
    size_t start_freq;
    size_t clk_per_chirp;    
    size_t step_freq_chirp;
    size_t adc_div;

    // FFT interface
    ArduinoFFT<float> FFT;
    float* vReal;
    float* vImag;

    size_t board_freq;

    // ==========================
    // private helper functions
    // ==========================
    
    /**
     * @brief Sets up the interrupt pin and handler.
     * @param pin The pin number for the interrupt.
     */
    void _setup_interrupt(size_t const pin, voidFuncPtr handler);
    
    /**
      * @brief Caches the chirp configuration from the register file.
      */
    void _cache_chirp_config();
    
    /**
    * @brief Fetches data from the RegFile using a given address.
    * @param address The address to fetch data from.
    * @return The data fetched from the RegFile, or 0 if no entry exists.
    */
    size_t fetch_from_regFile(size_t address);

    /**
    * @brief Writes data to the RegFile at a specified address.
    * @param address The address in the RegFile to write to.
    * @param data The data to write to the RegFile.
    * @return BGT_status::BGT_success on success, BGT_status::BGT_error on failure.
    */
    BGT_status write_to_regFile(size_t address, size_t data);
    
    /**
    * @brief Sets specific bits in a register of the BGT sensor.
    * @param reg_addr The address of the register to modify.
    * @param bits The bits to set in the register.
    * @return BGT_status::BGT_success on success, BGT_status::BGT_error on failure.
    */
    BGT_status set_bits(size_t const reg_addr, size_t const bits);

    /**
    * @brief Unpacks recorded ADC data into the real part of the FFT.
    * 
    *   Words (ADC) are represented in a byte array:
    *   a2 a1;  a0 b2;  b1 b0
    * @return BGT_status::BGT_success on success, BGT_status::BGT_error on failure.
    */
    BGT_status unpack_adc_data();

    /**
    * @brief Runs a high-pass filter on the real part of the FFT data.
    * @return BGT_status::BGT_success on success, BGT_status::BGT_error on failure.
    */
    BGT_status apply_highpass_filter();

    /**
    * @brief Anti Coupling filter for receiver/transmitter antenna
    * @return BGT_status::BGT_success on success, BGT_status::BGT_error on failure.
    */
    BGT_status apply_anti_coupling_filter();



public:

    /**
    * Implementation of a BGT60-Radar sensor using one antenna.
    * Creates a dynamic instance, which needs to be deleted by user!
    * 
    * @param word_size The size of one word in the sensor. 
    * @param interrupt_handler Pointer to the interrupt handler function.
    * @param pin_cs Pin to ChipSelect
    * @param pin_interrupt Pin to Interrupt Pin of Sensor
    * @param board_freq Used board frequency
    * @param spi_interface SPI-Interface to Radar-Sensor, default used SPI-Default-Interface
    */
    BGT60TRXX(
        size_t const word_size, 
        voidFuncPtr interrupt_handler,
        size_t pin_cs,
        size_t pin_interrupt,
        size_t board_freq,
        SPIClass *spi_interface = &SPI
    );
    
    /**
    * Deinitializes the BGT60TRXX sensor.
    * Frees all allocated memory.
    */
    ~BGT60TRXX();

    // remove copy and assignment
    BGT60TRXX(const BGT60TRXX&) = delete;
    BGT60TRXX& operator=(const BGT60TRXX&) = delete;

    
    /**
    * Calculates the range resolution of the sensor.
    * The range resolution is calculated based on the sensor's chirp configuration.
    * @return The range resolution in meters per index.
    */
    float get_range_resolution();

    /**
    * Reads a register from the BGT60TRXX sensor.
    * @param reg_addr The address of the register to read.
    * @return The value of the register, or BGT_error on failure.
    */
    BGT_status read_reg(size_t const reg_addr);

    /**
    * Writes a value to a register in the BGT60TRXX sensor.
    * @param reg_addr The address of the register to write to.
    * @param data The data to write to the register.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status write_reg(
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
    * @param data The ADC division factor to set.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status set_adc_div(size_t const data);

    /**
    * Sets the chirp length for the sensor.
    * Only enabled with init_sensor method
    * @param chirp_len The length of the chirp in samples.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status set_chirp_len(size_t const chirp_len);

    /**
    * Configures the chirp parameters for the sensor.
    * Only enabled with init_sensor method.
    * For calculation of parameters see Datasheet.
    * @param N_FSU The starting frequency in Hz.
    * @param N_RTU The clock cycles per chirp.
    * @param N_RSU The frequency step per clock cycle.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status configure_chirp(
        size_t const N_FSU, 
        size_t const N_RTU, 
        size_t const N_RSU
    );

    /**
    * Sets the VGA gain for a specific channel.
    * Only enabled with init_sensor method.
    * @param gain The gain value to set (0-5).
    * @param channel The channel number to set the gain for (1-3).
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status set_vga_gain(size_t const channel, size_t const gain);

    /**
    * Sets register values for sensor.
    * Only enabled with init_sensor method.
    * @param data The data to set in the register.
    * @param address The address of the register to set.
    * @param reset_mask The mask to reset bits in the register.
    * @param offset The offset for the data in the register.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status update_register_field(
        size_t const data, 
        size_t const address, 
        size_t const reset_mask, 
        size_t const offset
    );

    /**
    * Sets the compare value for the sensor using a read-modify-write.
    * If the compare value is greater than or equal to 100,
    * it is set to 99.
    * @param compare_value The compare value to set (0-99).
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status set_compare_value(size_t const compare_value);

    /**
    * Enables the test mode (LFSR Enable) for the sensor.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status enable_test_mode();

    /**
    * Starts the frame generation for the sensor.
    * This function leaves the deep sleep mode and prepares the sensor for data acquisition.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status start_frame();

    /**
    * Reads data from the FIFO stack of the sensor.
    * Checks header information for errors (like overflow/underflow).
    * Data is stored inside the sensor's data object.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status read_fifo();

    /**
    * Reads the distance data from the sensor.
    * Unpacks the recorded ADC data into the real part of the FFT.
    * Runs a high-pass filter on the data before performing the FFT.
    * The resulting range profile is stored in the sensor's FFT structure.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status read_distance();

    /**
    * Initializes the sensor by writing all register values from the register file to the sensor.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status init_sensor();

    /**
    * Resets the FIFO and FSM of the sensor.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status reset_fifo();

    /**
    * Resets the FSM of the sensor.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status reset_fsm();

    /**
    * This function resets the sensor's state, FIFO, and FSM.
    * @return BGT_success on success, BGT_error on failure.
    */
    BGT_status reset();

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
    * @brief Gets the FFT data calculated from the sensor.
    * @return Pointer to the FFT data array.
    */
    float* get_fft_data();

    /**
    * @brief Gets the length of the FFT data.
    * @return The length of the FFT data.
    */
    size_t get_fft_length();
};

/**
* @brief Converts FFT bin index to physical range.
* @param index FFT bin index.
* @param range_resolution Range resolution in meters (from get_range_resolution).
* @return Range in meters.
*/
float calculate_range_from_index(int index, float range_resolution);


#endif // BGT60TRXX_LIB_HPP