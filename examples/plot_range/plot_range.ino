#include <SPI.h>
#include <time.h>
#include "bgt60tr13c.hpp"
#include <string.h>

// const values
static const size_t no_of_chirps = 1;
static const size_t samples_per_chirp = 128;
static const size_t words = samples_per_chirp * no_of_chirps;
static const size_t ADC_DIV = 60;
static const size_t start_freq = 58000000;  // in kHz
static const size_t bandwidth  =  4500000;    // in kHz

static float range_resolution;

static float threshold = 2.1;

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
  * @brief Handels float to string conversion for print
  */
String ftos(float const value);

/**
 * @brief Interrupt handler function.
 */
void interrupt_handler() 
{
  Serial.println(">Interrupt Handler called");
}

BGT60TRXX* sensor = nullptr;
void setup() 
{
  Serial.begin(115200);

  sensor = new BGT60TRXX(words, &interrupt_handler, RSPI_CS, RXRES_L, CHIP_FREQ, spi_interface);

  Serial.println("> Reset Sensor...");
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

void loop() 
{
  sensor->read_distance();
  
  size_t const len = sensor->get_fft_length();
  float* fft_measured_data = sensor->get_fft_data();

  // Only us length/2 -> removes duplication spectrum
  String data_output = "fft;";
  String threshold_output = "threshold;";
  for (size_t i = 0; i < len / 2; i++) {
      float distance_cm = calculate_range_from_index(i, range_resolution) * 100.0 / no_of_chirps;
      data_output += ftos(distance_cm) + "," + ftos(fft_measured_data[i]) + ";";
      threshold_output += ftos(distance_cm) + "," + ftos(threshold) + ";";
  }

  // send special string to now plot a signal
  Serial.println(data_output);

  // send special string to now plot threshold
  Serial.println(threshold_output);

  delay(100);
  
  sensor->reset_fifo();
  sensor->start_frame();
}

String ftos(float const value) 
{
    static int const buffer_size = 16;
    static char buffer[buffer_size];

    int front = (int)value;
    int back = (int)(abs(value-1.0*front)*100);
    
    // Format the string
    snprintf(buffer, buffer_size, "%d.%d", front, back);

    return String(buffer);
}