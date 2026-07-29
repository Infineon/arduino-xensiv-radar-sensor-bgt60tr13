#include "bgt60_radar.hpp"

BGT60Radar::BGT60Radar()
  : sensor(nullptr), range_resolution(0.0f)
{
}

BGT60Radar::~BGT60Radar()
{
  if(this->sensor != nullptr)
  {
    delete this->sensor;
    this->sensor = nullptr;
  }
}

bool BGT60Radar::init(const RadarConfig& config)
{
  this->config = config;

  // --- Derive the chirp bandwidth from the requested maximum range ---
  //
  // The range math (see BGT60TR13C::get_range_resolution) gives
  //     max_range = samples_per_chirp * c / (4 * bandwidth)
  // so, solving for the bandwidth in kHz:
  //     bandwidth[kHz] = samples_per_chirp * SPEED_OF_LIGHT * 250 / max_range
  // SPEED_OF_LIGHT is 300 (Mm/s); the constant 250 = 1000 / 4 converts the
  // Mm/s & MHz terms into kHz, the unit calculate_RSU() expects.
  //
  // Usable sweep bandwidth of the BGT60TR13C (approx. 58 - 63.5 GHz band).
  constexpr float BW_MIN_KHZ = 200000.0f;   //  0.2 GHz -> largest max_range
  constexpr float BW_MAX_KHZ = 5500000.0f;  //  5.5 GHz -> smallest max_range

  if(config.max_range <= 0.0f)
  {
    Serial.println("> init: max_range must be greater than 0 m.");
    return false;
  }

  float const bandwidth_f =
      ((float)config.samples_per_chirp * (float)SPEED_OF_LIGHT * 250.0f)
      / config.max_range;

  if(bandwidth_f < BW_MIN_KHZ || bandwidth_f > BW_MAX_KHZ)
  {
    Serial.print("> init: max_range = ");
    Serial.print(config.max_range);
    Serial.println(" m is outside the supported range (~1.75 m .. 48 m).");
    return false;
  }

  size_t const bandwidth = (size_t)(bandwidth_f + 0.5f);  // kHz, rounded

  if(config.zero_padding_factor == 0)
  {
    Serial.println("> init: zero_padding_factor must be >= 1.");
    return false;
  }

  size_t const word_size = config.samples_per_chirp * config.zero_padding_factor;
  // Re-initialisation: drop any previous sensor instance first.
  if(this->sensor != nullptr)
  {
    delete this->sensor;
    this->sensor = nullptr;
  }

  this->sensor = new BGT60TR13C(
    word_size,
    config.interrupt_handler,
    config.pin_cs,
    config.pin_interrupt,
    config.board_freq,
    config.spi_interface
  );

  if(this->sensor == nullptr)
    return false;

  // --- Low-level bring-up ---
  if(!this->sensor->reset())
    return false;

  if(!this->sensor->set_adc_div(config.adc_div))
    return false;

  if(!this->sensor->set_chirp_len(config.samples_per_chirp))
    return false;

  size_t const FSU = this->sensor->calculate_FSU(config.start_freq);
  size_t const RTU = this->sensor->calculate_RTU(config.adc_div, config.samples_per_chirp);
  size_t const RSU = this->sensor->calculate_RSU(bandwidth, RTU);

  if(!this->sensor->configure_chirp(FSU, RTU, RSU))
    return false;

  if(!this->sensor->set_vga_gain(1, config.vga_gain))
    return false;

  if(!this->sensor->init_sensor())
    return false;

  this->range_resolution = this->sensor->get_range_resolution();

  // Only the lower half of the spectrum carries unique range information.
  this->data.magnitudes = nullptr;
  this->data.length = word_size / 2;
  this->data.range_resolution = this->range_resolution;

  // Arm the first acquisition.
  if(!this->sensor->start_frame())
    return false;

  return true;
}

bool BGT60Radar::read_distance()
{
  if(this->sensor == nullptr)
    return false;

  if(this->config.frame_delay_ms > 0)
    delay(this->config.frame_delay_ms);

  this->sensor->reset_fifo();
  this->sensor->start_frame();

  bool const ok = (bool)this->sensor->read_distance();

  if(ok)
  {
    this->data.magnitudes = this->sensor->get_fft_data();
    this->data.length = this->sensor->get_fft_length() / 2;
    this->data.range_resolution = this->range_resolution;
  }

  return ok;
}

const DistanceData& BGT60Radar::get_distance_data() const
{
  return this->data;
}

float BGT60Radar::get_range_resolution() const
{
  return this->range_resolution;
}

BGT60TR13C* BGT60Radar::raw()
{
  return this->sensor;
}
