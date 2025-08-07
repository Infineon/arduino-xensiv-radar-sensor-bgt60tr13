/// Main program to test the bgt60tr13c sensor in arduino.
/// 10.06.2025
/// Samuel Weissenbacher, Infinieon
/// ======================================================
#include "bgt60trxx_lib.hpp"
#include <time.h>

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

void interrupt_handler() {
  Serial.println(">Interrupt Handler called");
}

void fft_to_dB(float* fft_data, size_t length) {
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

  bgt60trxx_sensor = initStruct(words, (*interrupt_handler));

  Serial.println("> Reset Sensor...");
  reset(bgt60trxx_sensor);

  set_adc_div(bgt60trxx_sensor, ADC_DIV);
  set_chirp_len(bgt60trxx_sensor, samples_per_chirp);

  size_t FSU = calculateFSU(start_freq);
  size_t RTU = calculateRTU(ADC_DIV, samples_per_chirp);
  size_t RSU = calculateRSU(bandwidth, RTU);
  
  Serial.print("> FSU = ");
  Serial.println(FSU);
  
  Serial.print("> RTU = ");
  Serial.println(RTU);
  
  Serial.print("> RSU = ");
  Serial.println(RSU);

  configure_chirp(bgt60trxx_sensor, FSU, RTU, RSU);

  set_vga_gain_ch1(bgt60trxx_sensor, 3);

  initSensor(bgt60trxx_sensor);
  Serial.println("> Sensor initialised!");
  
  range_resolution = get_range_resolution(bgt60trxx_sensor) * 100;  // in cm
  Serial.print("> Range resoultion is = ");
  Serial.println(range_resolution);
  
  startFrame(bgt60trxx_sensor);
}

void loop() {
  unsigned long startTime = millis();
  readDistance(bgt60trxx_sensor);
  unsigned long elapsedTime = millis() - startTime;

  Serial.print(">Function readFIFO Time [ms] = ");
  Serial.println(elapsedTime);

  // divide by 2 -> removes duplication spectrum
  for (size_t i = 0; i < bgt60trxx_sensor->word_size / 2; i++) {
    fft_data[i] = bgt60trxx_sensor->vReal[i];
  }

  // transform data to db-scale
  fft_to_dB(fft_data, bgt60trxx_sensor->word_size/2);

  for (size_t i = 0; i < bgt60trxx_sensor->word_size / 2; i++) {
    double distance = i * range_resolution / no_of_chirps;
    Serial.print(distance);
    Serial.print(",");
    Serial.print(fft_data[i]);
    Serial.print(";");
  }
  Serial.println("");

  delay(100);
  
  resetFIFO(bgt60trxx_sensor);
  startFrame(bgt60trxx_sensor);
}
