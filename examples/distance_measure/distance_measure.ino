#include <SPI.h>
#include "bgt60tr13c.hpp"

// const values
static const size_t no_of_chirps = 1;
static const size_t samples_per_chirp = 128;
static const size_t words = samples_per_chirp * no_of_chirps;
static const size_t ADC_DIV = 60;
static const size_t start_freq = 62500000;  // in kHz
static const size_t bandwidth  =  2000000;    // in kHz

static const float threshold_for_lower_freq = 43.8;
static const float threshold_for_upper_freq = 33.5;

static const size_t threshold_start = 20;  // in cm
static const size_t threshold_end = 190;    // in cm

static bgt60trxx_struct* bgt60trxx_sensor;

static float fft_data[words / 2];

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

#ifdef TARGET_APP_CY8CKIT_062S2_AI
// The CY8CKIT-062S2-AI-Board uses the
// class SPIClassPSOC to create a new SPI-Instance.
// This way, we can wire the radar sensor directly
// to the spi interface.
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
  When a different Board is used, the default SPI-Class
  is used.
  Change, when necessary.
*/
static SPIClass* spi_interface = &SPI;
#endif

/**
 * @brief Interrupt handler function.
 */
void interrupt_handler() {
  Serial.println(">Interrupt Handler called");
}

float threshold_func[words / 2];
/**
 * @brief Finds the nearest peak in the signal based on a threshold function.
 * 
 * Uses this Threshold Function:
 *  if x < start -> threshold_for_lower_freq
 *  if x > end   -> threshold_for_upper_freq
 *  else         -> build linear function between first two
 * 
 * @param signal Pointer to the signal data.
 * @param threshold_index_start Index to start the threshold function.
 * @param threshold_index_end Index to end the threshold function.
 */
void find_nearest_peak(float const * const signal,
                      size_t const threshold_index_start,
                      size_t const threshold_index_end)
{
  for(size_t i = 0; i < words/2; i++)
  {
    if (i < threshold_index_start)
    {
      threshold_func[i] = threshold_for_lower_freq;
    }
    else if(i > threshold_index_end)
    {
      threshold_func[i] = threshold_for_upper_freq;
    }
    else
    {
      // kx + d
      threshold_func[i] = (-((threshold_for_lower_freq-threshold_for_upper_freq)
                             /(threshold_index_end-threshold_index_start))
                           *(i-threshold_index_start) 
                           + threshold_for_lower_freq);
    }
  }
  for(size_t i = 1; i < words/2 - 1; i++)
  {
    if (signal[i] > threshold_func[i])
    {
      Serial.print(">Peak detected at: ");
      Serial.print(i*range_resolution / no_of_chirps);
      Serial.println("cm");
      return;
    }
  }
}

/**
 * @brief Converts FFT data to dB scale.
 * @param fft_data Pointer to the FFT data array.
 * @param length Length of the FFT data array.
 */
void fft_to_dB(float * const fft_data, size_t const length) {
  size_t i = 1;
  while (i < length) {
    // clip signal
    if(fft_data[i] < 0.001)
      fft_data[i] = 0.001;
    // calculate to dB-Scale
    fft_data[i] = (10.0 * log10(fft_data[i]));
    i += 1;
  }
  fft_data[0] = 0;  // Remove DC-Value
}

void setup() {
  Serial.begin(115200);

  printf("> Serial Monitor enabled.");
  
  //Reset SPI-Interface
  pinMode(RXRES_L, OUTPUT);
  pinMode(RSPI_CS, OUTPUT);
  digitalWrite(RSPI_CS, HIGH);
  digitalWrite(RXRES_L, LOW);
  digitalWrite(RXRES_L, HIGH);

  bgt60trxx_sensor = init_struct(words, &interrupt_handler, spi_interface);
  if (!bgt60trxx_sensor) {
    Serial.println("Sensor initialization failed!");
    while (1);
  }

  Serial.println("> Reset Sensor...");
  reset(bgt60trxx_sensor);

  set_adc_div(bgt60trxx_sensor, ADC_DIV);
  set_chirp_len(bgt60trxx_sensor, samples_per_chirp);

  size_t FSU = calculate_FSU(start_freq);
  size_t RTU = calculate_RTU(ADC_DIV, samples_per_chirp);
  size_t RSU = calculate_RSU(bandwidth, RTU);

  Serial.print("> FSU = ");
  Serial.println(FSU);
  
  Serial.print("> RTU = ");
  Serial.println(RTU);
  
  Serial.print("> RSU = ");
  Serial.println(RSU);

  configure_chirp(bgt60trxx_sensor, FSU, RTU, RSU);

  set_vga_gain_ch1(bgt60trxx_sensor, 3);

  init_sensor(bgt60trxx_sensor);
  Serial.println("> Sensor initialised!");
  
  range_resolution = get_range_resolution(bgt60trxx_sensor) * 100;  // in cm
  Serial.print("> Range resoultion is = ");
  Serial.println(range_resolution);
  
  start_frame(bgt60trxx_sensor);
}

void loop() {
  read_distance(bgt60trxx_sensor);

  size_t const len = get_fft_length(bgt60trxx_sensor);
  float* fft_measured_data = get_fft_data(bgt60trxx_sensor);

  // divide by 2 -> removes duplication spectrum
  for (size_t i = 0; i <len / 2; i++) {
    fft_data[i] = abs(fft_measured_data[i]);
  }

  // transform data to db-scale
  fft_to_dB(fft_data, len/2);

  find_nearest_peak(
    fft_data, 
    threshold_start/(range_resolution/ no_of_chirps), 
    threshold_end/(range_resolution/ no_of_chirps)
  );

  delay(100);
  
  reset_FIFO(bgt60trxx_sensor);
  start_frame(bgt60trxx_sensor);
}
