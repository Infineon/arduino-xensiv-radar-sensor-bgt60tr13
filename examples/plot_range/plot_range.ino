#include <time.h>
#include <string.h>
#include "bgt60_radar.hpp"

// const values
// Maximum measurable distance in meters. A smaller value raises the chirp
// bandwidth and gives finer resolution; a larger value lowers the bandwidth,
// so resolution gets coarser and far/weak targets are harder to measure.
static const float max_range = 3.20;    // in meters (~2.5 GHz sweep)

static float threshold = 3.8;

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
  * @brief Handles float to string conversion for print
  */
String ftos(float const value);

/**
 * @brief Interrupt handler function.
 */
void interrupt_handler() 
{
  Serial.println(">Interrupt Handler called");
}

BGT60Radar radar;
void setup() 
{
  Serial.begin(115200);

  RadarConfig config;
  config.pin_cs            = RSPI_CS;
  config.pin_interrupt     = RXRES_L;
  config.board_freq        = CHIP_FREQ;
  config.interrupt_handler = &interrupt_handler;
  config.spi_interface     = spi_interface;
  config.max_range         = max_range;
  config.vga_gain          = 3;

  if (!radar.init(config)) {
    Serial.println("> Sensor initialisation failed!");
    return;
  }
  Serial.println("> Sensor initialised!");

  Serial.print("> Range resolution is = ");
  Serial.print(radar.get_range_resolution() * 100);
  Serial.println(" cm");
}

void loop() 
{
  if (!radar.read_distance())
    return;

  DistanceData d = radar.get_distance_data();

  // Only length/2 bins carry unique range information (handled by the wrapper).
  String data_output = "fft;";
  String threshold_output = "threshold;";
  for (size_t i = 0; i < d.length; i++) {
      float distance_cm = i * d.range_resolution * 100.0;
      data_output += ftos(distance_cm) + "," + ftos(d.magnitudes[i]) + ";";
      threshold_output += ftos(distance_cm) + "," + ftos(threshold) + ";";
  }

  // send special string to now plot a signal
  Serial.println(data_output);

  // send special string to now plot threshold
  Serial.println(threshold_output);
}

String ftos(float const value) 
{
    static int const buffer_size = 7;
    static char buffer[buffer_size];

    int front = (int)value;
    int back = (int)(abs(value-(float)front)*100);
    
    // Format the string
    snprintf(buffer, buffer_size, "%d.%d", front, back);

    return String(buffer);
}