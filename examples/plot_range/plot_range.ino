#include <SPI.h>
#include <time.h>
#include "bgt60tr13c.hpp"

// const values
static const size_t no_of_chirps = 1;
static const size_t samples_per_chirp = 128;
static const size_t words = samples_per_chirp * no_of_chirps;
static const size_t ADC_DIV = 60;
static const size_t start_freq = 62500000;  // in kHz
static const size_t bandwidth  =  2000000;    // in kHz

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
static SPIClass* spi_interface = &SPI;
#endif

/**
 * @brief Interrupt handler function.
 */
void interrupt_handler() {
  Serial.println(">Interrupt Handler called");
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
    fft_data[i] = max(0.001, fft_data[i]);
    // calculate to dB-Scale
    fft_data[i] = (10.0 * log10(fft_data[i]));
    i += 1;
  }
  fft_data[0] = 0;  // Remove DC-Value
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(USER_BUTTON, INPUT);

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
  unsigned long startTime = millis();
  read_distance(bgt60trxx_sensor);
  unsigned long elapsedTime = millis() - startTime;

  Serial.print(">Function readFIFO Time [ms] = ");
  Serial.println(elapsedTime);
  
  size_t const len = get_fft_length(bgt60trxx_sensor);
  float* fft_measured_data = get_fft_data(bgt60trxx_sensor);

  // divide by 2 -> removes duplication spectrum
  for (size_t i = 0; i < len / 2; i++) {
    fft_data[i] = fft_measured_data[i];
  }

  // transform data to db-scale
  fft_to_dB(fft_data, len / 2);

  for (size_t i = 0; i < len / 2; i++) {
    double distance = i * range_resolution / no_of_chirps;
    Serial.print(distance);
    Serial.print(",");
    Serial.print(fft_data[i]);
    Serial.print(";");
  }
  Serial.println("");

  delay(100);
  
  reset_FIFO(bgt60trxx_sensor);
  start_frame(bgt60trxx_sensor);
}
