#ifndef BGT60_RADAR_HPP
#define BGT60_RADAR_HPP

#include <Arduino.h>
#include <SPI.h>
#include "bgt60tr13c.hpp"

/**
 * @brief High-level configuration for the radar sensor.
 *
 * Bundles everything the client would otherwise have to wire up by hand
 * (SPI pins, chirp timing, gain). Sensible defaults are provided; typically
 * only the pins and board frequency must be set.
 */
struct RadarConfig {
    // --- Hardware / SPI ---
    size_t pin_cs;                       ///< Chip-select pin.
    size_t pin_interrupt;                ///< Sensor interrupt / reset pin.
    size_t board_freq;                   ///< System clock of the board in Hz.
    voidFuncPtr interrupt_handler = nullptr; ///< Optional IRQ handler (may be null).
    SPIClass* spi_interface = &SPI;      ///< SPI instance wired to the sensor.

    // --- Acquisition / chirp ---
    size_t samples_per_chirp = 128;      ///< ADC samples per chirp.
    size_t zero_padding_factor = 4;      ///< FFT zero-padding factor (1, 2, 4, ...).
    size_t adc_div = 60;                 ///< ADC clock divider.
    size_t start_freq = 58000000;        ///< Chirp start frequency in kHz.
    size_t vga_gain   = 3;               ///< VGA gain for RX channel 1 (0-5).

    // --- Range ---
    /// Maximum measurable distance in meters. This is the knob the user tunes;
    /// the wrapper derives the chirp bandwidth from it in init():
    ///     bandwidth = samples_per_chirp * c / (4 * max_range)
    ///
    /// Trade-off: a SMALLER max_range raises the bandwidth and gives FINER
    /// range resolution. A LARGER max_range lowers the bandwidth, so the
    /// resolution gets COARSER - distant and weak targets, and telling close
    /// targets apart, become harder / less accurate.
    ///
    /// The value is validated in init(): it must be > 0 and produce a bandwidth
    /// within the sensor's usable sweep (roughly 1.75 m .. 48 m at 128
    /// samples/chirp); init() returns false otherwise.
    float max_range = 3.84f;             ///< In meters (~2.5 GHz sweep).

    // --- Timing ---
    /// Pause (ms) after a read before the FIFO is reset and the next frame is
    /// armed. The subsequent frame is then read promptly, matching the sensor's
    /// required read -> delay -> reset_fifo -> start_frame ordering.
    size_t frame_delay_ms = 100;
};

/**
 * @brief Processed range profile returned by the radar.
 *
 * Points at the sensor's internal buffer and is valid until the next
 * @ref BGT60Radar::read_distance call.
 */
struct DistanceData {
    const float* magnitudes = nullptr;   ///< Range profile in dB (index 0..length-1).
    size_t length = 0;                   ///< Number of usable range bins.
    float range_resolution = 0.0f;       ///< Meters per bin.
};

/**
 * @brief Simple, high-level wrapper around the BGT60TR13C sensor.
 *
 * Hides the low-level register/chirp configuration behind three calls:
 *   1. @ref init            – configure and start the sensor.
 *   2. @ref read_distance   – acquire and process one range profile.
 *   3. @ref get_distance_data – retrieve the latest processed profile.
 *
 * Typical usage:
 * @code
 *   BGT60Radar radar;
 *   RadarConfig cfg;
 *   cfg.pin_cs = RSPI_CS;
 *   cfg.pin_interrupt = RXRES_L;
 *   cfg.board_freq = 100000000;
 *   radar.init(cfg);
 *
 *   while (true) {
 *     if (radar.read_distance()) {
 *       DistanceData d = radar.get_distance_data();
 *       for (size_t i = 0; i < d.length; i++) {
 *         float range_m = i * d.range_resolution;
 *         // use d.magnitudes[i] ...
 *       }
 *     }
 *   }
 * @endcode
 */
class BGT60Radar {
public:
    BGT60Radar();
    ~BGT60Radar();

    // Non-copyable: owns a raw sensor instance.
    BGT60Radar(const BGT60Radar&) = delete;
    BGT60Radar& operator=(const BGT60Radar&) = delete;

    /**
     * @brief Configures and starts the sensor.
     *
     * Performs the full low-level bring-up (reset, ADC/chirp configuration,
     * gain, register upload) and arms the first acquisition.
     * @param config Radar configuration.
     * @return true on success, false if any step failed.
     */
    bool init(const RadarConfig& config);

    /**
     * @brief Acquires and processes one range profile.
     *
     * Reads the FIFO, runs the signal-processing chain, caches the result and
     * arms the next acquisition. Call @ref get_distance_data afterwards.
     * @return true on success, false on a read/processing error.
     */
    bool read_distance();

    /**
     * @brief Returns the most recently processed range profile.
     * @return Reference to the cached distance data (valid until next read).
     */
    const DistanceData& get_distance_data() const;

    /**
     * @brief Range resolution of the current configuration.
     * @return Meters per range bin.
     */
    float get_range_resolution() const;

    /**
     * @brief Access to the underlying low-level sensor (advanced use).
     * @return Pointer to the sensor, or nullptr if not initialised.
     */
    BGT60TR13C* raw();

private:
    BGT60TR13C* sensor;
    RadarConfig config;
    DistanceData data;
    float range_resolution;
};

#endif // BGT60_RADAR_HPP
