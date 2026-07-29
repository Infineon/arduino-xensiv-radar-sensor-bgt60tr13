#include "bgt60_radar.hpp"

// const values
// Maximum measurable distance in meters. A smaller value raises the chirp
// bandwidth and gives finer resolution; a larger value lowers the bandwidth,
// so resolution gets coarser and far/weak targets are harder to measure.
static const float max_range = 2.4;    // in meters (~2.5 GHz sweep)
static const float threshold = 3.7;

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

BGT60Radar radar;

void setup() {
  Serial.begin(115200);
  Serial.println("> Serial Monitor enabled.");

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

void loop() {
  if (!radar.read_distance())
    return;

  // Range to the nearest target above the threshold (in meters), or < 0 if none.
  float distance = radar.raw()->get_nearest_distance(threshold);
  if (distance >= 0.0) {
    Serial.print(">Nearest target at: ");
    Serial.print(distance * 100.0);
    Serial.println("cm");
  }
}
