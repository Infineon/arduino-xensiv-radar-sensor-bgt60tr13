/// Main program to measure distance
/// from the bgt60tr13c sensor in arduino.
/// 14.07.2025
/// Samuel Weissenbacher, Infinieon
/// ======================================================
#include "bgt60trxx_lib.hpp"

// const values
const size_t no_of_chirps = 1;
const size_t samples_per_chirp = 128;
const size_t words = samples_per_chirp * no_of_chirps;
const size_t ADC_DIV = 60;
const size_t start_freq = 62500000;  // in kHz
const size_t bandwidth  =  2000000;    // in kHz

const float threshold_for_lower_freq = 43.8;
const float threshold_for_upper_freq = 33.5;

bgt60trxx_struct* bgt60trxx_sensor;

float fft_data[words / 2];

float range_resolution;

void interrupt_handler() {
  Serial.println(">Interrupt Handler called");
}

float threshold_func[words / 2];
//detect first peak in signal.
//Uses a Threshold Function:
//  if x < start -> threshold_for_lower_freq
//  if x > end   -> threshold_for_upper_freq
//  else         -> build linear function between first two
void find_nearest_peak(float const * const signal,
                      size_t threshold_index_start,
                      size_t threshold_index_end)
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

  bgt60trxx_sensor = initStruct(words, (void*)interrupt_handler);

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
  readDistance(bgt60trxx_sensor);

  // divide by 2 -> removes duplication spectrum
  for (size_t i = 0; i < bgt60trxx_sensor->word_size / 2; i++) {
    fft_data[i] = abs(bgt60trxx_sensor->vReal[i]);
  }

  // transform data to db-scale
  fft_to_dB(fft_data, bgt60trxx_sensor->word_size/2);

  find_nearest_peak(fft_data, 20/(range_resolution/ no_of_chirps), 
                    190/(range_resolution/ no_of_chirps));

  delay(100);
  
  resetFIFO(bgt60trxx_sensor);
  startFrame(bgt60trxx_sensor);
}
