#include <SPI.h>
#include "bgt60tr13c.hpp"

// const values
static const size_t no_of_chirps = 1;
static const size_t samples_per_chirp = 128;
static const size_t words = samples_per_chirp * no_of_chirps;
static const size_t ADC_DIV = 60;
static const size_t start_freq = 58000000;  // in kHz
static const size_t bandwidth  =  4500000;    // in kHz

static const float threshold = 2.1;

static float range_resolution;

/*
  Define the pins for the BGT60TR13C sensor.
  The Board used is the Infineon CY8CKIT-062S2-AI.
*/
#define RSPI_MOSI 41
#define RSPI_MISO 42
#define RSPI_SCLK 43
#define RSPI_CS   44
#define RXRES_L   40

#define CHIP_FREQ 100000000

#ifdef TARGET_APP_CY8CKIT_062S2_AI
/* 
 * The CY8CKIT-062S2-AI-Board uses the
 * class SPIClassPSOC to create a new SPI-Instance.
 * This way, we can wire the radar sensor directly
 * to the spi interface.
 */
static SPIClassPSOC spi_radar_interface = SPIClassPSOC(
  RSPI_MOSI, 
  RSPI_MISO, 
  RSPI_SCLK, 
  NC, 
  false
);
static SPIClass* spi_interface = &spi_radar_interface;
#else
/*
 * When a different Board is used, the default SPI-Class
 * is used.
 * Change, when necessary.
 */
static SPIClass* spi_interface = &SPI;
#endif



/**
 * @brief Interrupt handler function.
 */
void interrupt_handler() {
  Serial.println(">Interrupt Handler called");
}

/**
 * @brief Finds the nearest peak in the signal based on a threshold function.
 * 
 * Uses this Threshold Function:
 *  if x < start -> threshold_for_lower_freq
 *  if x > end   -> threshold_for_upper_freq
 *  else         -> build linear function between first two
 * 
 * @param signal Pointer to the signal data.
 */
void detect_nearest_target(float const * const signal)
{
  for(size_t i = 1; i < words/2 - 1; i++)
  {
    if (signal[i] > threshold)
    {
      float distance_cm = calculate_range_from_index(i, range_resolution) * 100.0 / no_of_chirps;
      Serial.print(">Peak detected at: ");
      Serial.print(distance_cm);
      Serial.println("cm");
      return;
    }
  }
}

BGT60TRXX* sensor;
void setup() {
  Serial.begin(115200);
  Serial.println("> Serial Monitor enabled.");

  sensor = new BGT60TRXX(words, &interrupt_handler, RSPI_CS, RXRES_L, CHIP_FREQ, spi_interface);

  Serial.println("> Reset sensor.");
  sensor->reset();

  sensor->set_adc_div(ADC_DIV);
  sensor->set_chirp_len(samples_per_chirp);

  size_t FSU = sensor->calculate_FSU(start_freq);
  size_t RTU = sensor->calculate_RTU(ADC_DIV, samples_per_chirp);
  size_t RSU = sensor->calculate_RSU(bandwidth, RTU);

  Serial.print("> FSU = ");
  Serial.println(FSU);
  
  Serial.print("> RTU = ");
  Serial.println(RTU);
  
  Serial.print("> RSU = ");
  Serial.println(RSU);

  sensor->configure_chirp(FSU, RTU, RSU);

  sensor->set_vga_gain(1, 3);

  sensor->init_sensor();
  Serial.println("> Sensor initialised!");
  
  range_resolution = sensor->get_range_resolution();  // in meters
  Serial.print("> Range resolution is = ");
  Serial.print(range_resolution * 100);
  Serial.println(" cm");
  
  sensor->start_frame();
}

void loop() {
  sensor->read_distance();
  
  float* fft_measured_data = sensor->get_fft_data();

  detect_nearest_target(fft_measured_data);

  delay(100);
  
  sensor->reset_fifo();
  sensor->start_frame();
}
