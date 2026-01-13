#include "bgt60tr13c.hpp"
#include "pins_arduino.h"

// local functions
//===========================

/**
 * @brief Converts a 32-bit integer to a byte array.
 * @param data The 32-bit integer to convert.
 * @param dataByte Pointer to the byte array where the result will be stored.
 * @return BGT_status::BGT_success on success, BGT_status::BGT
 */
BGT_status int_to_bytes(uint32_t data, byte* dataByte);

/**
 * @brief Converts a byte array to a 32-bit integer.
 * @param dataByte Pointer to the byte array to convert.
 * @return The 32-bit integer representation of the byte array.
 */
int bytes_to_int(byte const * const dataByte);

/*
  Because of misaligned address.
  Address of FIFO = 0x60.
  But when reading, sensor returns
  error!
  4 words (or 6 bytes) seems to
  be the minimum value where
  an SPI-Read works.
*/
/**
 * @brief Number of bytes to skip before reading the FIFO.
 * This is necessary due to misaligned addresses in the BGT60TR13C sensor.
 * The FIFO address is 0x60, but reading it directly results in an error.
 * 4 words (or 6 bytes) seems to be the minimum value where an SPI-Read works.
 */
static const size_t skipFirstValues = 6;


void BGT60TR13C::_setup_interrupt(size_t const pin, voidFuncPtr handler)
{
  this->interrupt_handler = handler;
  if(handler != 0)
  {
    // Set the IRQ handler
    attachInterrupt(digitalPinToInterrupt(pin), handler, RISING);

    // Print a message to indicate that the IRQ handler has been set
    Serial.println("IRQ handler was set.");
  }
}

void BGT60TR13C::_cache_chirp_config()
{
  this->start_freq = (fetch_from_regFile(PLL1_0_ADDR) 
                      & PLL1_0_FSU_MASK);
  this->clk_per_chirp = (fetch_from_regFile(PLL1_2_ADDR) 
                      & PLL1_2_RTU_MASK);
  this->step_freq_chirp = (fetch_from_regFile(PLL1_1_ADDR) 
                      & PLL1_1_RSU_MASK);
}

BGT60TR13C::BGT60TR13C(
  size_t const word_size, 
  voidFuncPtr interrupt_handler,
  size_t pin_cs,
  size_t pin_interrupt,
  size_t board_freq,
  SPIClass *spi_interface
)
{
  Serial.println("> Initalizing Sensor...");
  this->word_size = word_size;
  this->frame_size = (size_t)(((float)word_size)*1.5 + 0.5);

  //Check if fifo overflow would happen
  if (this->frame_size > (FIFO_SIZE_BYTE - skipFirstValues))
  {
    Serial.println("Frame size too large! FIFO Overflow of radar sensor would happen.");
    return;
  }  
  this->frame_size  += skipFirstValues;

  this->data = (byte*) malloc(sizeof(byte)*(this->frame_size));
  if(this->data == 0)
  {
    Serial.println("Could not allocate memory for data buffer!");
    return;
  }

  for(size_t i = 0; i < SIZE_REG_FILE; i++)
    (this->register_values)[i] = init_register_list[i];
    
  //Init SPI-Interface
  pinMode(pin_interrupt, OUTPUT);
  pinMode(pin_cs, OUTPUT);
  digitalWrite(pin_cs, HIGH);
  digitalWrite(pin_interrupt, LOW);
  digitalWrite(pin_interrupt, HIGH);

  _setup_interrupt(pin_interrupt, interrupt_handler);

  this->radar_sensor_spi = spi_interface;
  this->pin_cs = pin_cs;
  
  // Set SPI Interface with 50 MHz
  this->radar_sensor_spi->begin();
  this->radar_sensor_spi->beginTransaction(
    SPISettings(50000000, MSBFIRST, SPI_MODE0)
  );
  
  //Chirp Configuration
  _cache_chirp_config();
  
  this->adc_div = (fetch_from_regFile(ADC0_ADDR) 
                    & ADC0_DIV_MASK) 
                  >> ADC0_DIV_OFFSET;

  this->board_freq = board_freq;

  // FFT Config
  this->vReal = (float*) malloc(sizeof(float)*(this->word_size));
  this->vImag = (float*) malloc(sizeof(float)*(this->word_size));

  if (vReal == 0 || vImag == 0) 
  {
    Serial.println("Could not allocate memory for FFT buffers!");
    return;
  }

  this->FFT = ArduinoFFT<float>
  (
    this->vReal, 
    this->vImag, 
    this->word_size, 
    this->board_freq / this->adc_div
  );
}

BGT60TR13C::~BGT60TR13C()
{
  if(this->vReal == nullptr)
  {
    free(this->vReal); 
    this->vReal = nullptr;
  }
  if(this->vImag == nullptr)
  {
    free(this->vImag); 
    this->vImag = nullptr;
  }
  if(this->data == nullptr)
  {
    free(this->data); 
    this->data = nullptr;
  }
}
    
float BGT60TR13C::get_range_resolution()
{
  size_t RSU = this->step_freq_chirp;
  size_t RTU = this->clk_per_chirp;
  
  float delta_f_RF = STEP_CHIRP_DIVIDER 
                      * F_ADC_CLK 
                      * (((float)RSU) / pow(2,20));

  // Multiply with 8! -> See Datasheet BGT60TR13C P.56 RTU
  float bandwidth = (RTU * 8) * delta_f_RF;

  return SPEED_OF_LIGHT / (2.0f * bandwidth);
}

BGT_status BGT60TR13C::read_reg(size_t const reg_addr)
{
  // Only Address needs to be send
  // LSB = R/W = 0
  byte addr = (reg_addr << 1) & 0xFE;

  // SPI Read
  digitalWrite(this->pin_cs, LOW);

  // Send address and read GSR0-Status-Register
  this->reg_data[0] = this->radar_sensor_spi->transfer(addr); 
  for (int i = 1; i < DATA_SIZE; i++) { // Read data
    this->reg_data[i] = this->radar_sensor_spi->transfer(0x00);
  }
  digitalWrite(this->pin_cs, HIGH);

  // Check if Error Occured
  if((this->reg_data[0] & 0x0F) != 0x0 
      and (this->reg_data[0] & 0x0F) != 0x4)
  {
    Serial.print("Status Register Error! GSR0 = ");
    Serial.println(this->reg_data[0], HEX);
    return BGT_status::BGT_error;
  }
    
  // Info!
  // Only use the lower bytes (Bit 23:0) -> Data
  // Rest is Header-Data
  return BGT_status::BGT_success;
}

BGT_status BGT60TR13C::write_reg(
  size_t const reg_addr, 
  size_t const data
)
{
  //SPI Transfer Format: Addr - R/W - Data 
  int dataInt = ((reg_addr << ADDR_OFFSET) & ADDR_MASK);
  dataInt |= WRITE_EN;
  dataInt |= ((data << DATA_OFFSET) & DATA_MASK);

  static byte dataToSend[DATA_SIZE];
  int_to_bytes(dataInt, dataToSend);

  // SPI Write
  digitalWrite(this->pin_cs, LOW);
  for (int i = 0; i < DATA_SIZE; i++) {
    this->radar_sensor_spi->transfer(dataToSend[i]); // Write data
  }
  digitalWrite(this->pin_cs, HIGH);
  return BGT_status::BGT_success;
}

BGT_status BGT60TR13C::set_adc_div(size_t const data)
{
  this->adc_div = data;
  this->FFT = ArduinoFFT<float>(
    this->vReal, 
    this->vImag, 
    this->word_size, 
    this->board_freq / this->adc_div
  );
  return update_register_field(data, 
                        ADC0_ADDR, 
                        ADC0_DIV_MASK, 
                        ADC0_DIV_OFFSET);
}

BGT_status BGT60TR13C::set_chirp_len(size_t const chirp_len)
{
  return update_register_field(chirp_len, 
                        PLL1_3_ADDR, 
                        APU0_MASK, 
                        APU0_OFFSET);
}

BGT_status BGT60TR13C::configure_chirp(size_t const N_FSU, size_t const N_RTU, size_t const N_RSU)
{
    if(!update_register_field(
      N_FSU, 
      PLL1_0_ADDR, 
      PLL1_0_FSU_MASK, 
      PLL1_0_FSU_OFFSET)
    )
      return BGT_status::BGT_error;
                    
    if(!update_register_field(
      N_RSU, 
      PLL1_1_ADDR, 
      PLL1_1_RSU_MASK, 
      PLL1_1_RSU_OFFSET)
    )
      return BGT_status::BGT_error;

    if(!update_register_field(
      N_RTU, 
      PLL1_2_ADDR, 
      PLL1_2_RTU_MASK, 
      PLL1_2_RTU_OFFSET)
    )
      return BGT_status::BGT_error;

    // set values for range resoultion
    this->start_freq = N_FSU;
    this->step_freq_chirp = N_RSU;
    this->clk_per_chirp = N_RTU;

    return BGT_status::BGT_success;
}
BGT_status BGT60TR13C::set_vga_gain(size_t const channel, size_t const gain)
{
    switch(channel) {
        case 1: return update_register_field(gain, CSU1_2_ADDR, CSU1_2_VGA_GAIN1_MASK, CSU1_2_VGA_GAIN1_OFFSET);
        case 2: return update_register_field(gain, CSU1_2_ADDR, CSU1_2_VGA_GAIN2_MASK, CSU1_2_VGA_GAIN2_OFFSET);
        case 3: return update_register_field(gain, CSU1_2_ADDR, CSU1_2_VGA_GAIN3_MASK, CSU1_2_VGA_GAIN3_OFFSET);
        default:
          Serial.print("set_vga_gain: invalid channel ");
          Serial.print(channel);
          Serial.println(" (valid range is 1-3)");
          return BGT_status::BGT_error;
    }
}

BGT_status BGT60TR13C::update_register_field(
  size_t const data, 
  size_t const address, 
  size_t const reset_mask, 
  size_t const offset)
{
  size_t oldValue = fetch_from_regFile(address);
  size_t newValue = (oldValue & ~reset_mask) | (data << offset);

  if(!write_to_regFile(address, newValue))
    return BGT_status::BGT_error;

  return BGT_status::BGT_success;
}

BGT_status BGT60TR13C::set_compare_value(
  size_t const compare_value
)
{
  size_t value = (compare_value >= 100) ? (FIFO_SIZE - 1) : compare_value;
  return update_register_field(value, 
                SFCTL_ADDR, 
                SFCTL_FIFO_CREF_MASK, 
                SFCTL_FIFO_CREF_OFFSET);
}

BGT_status BGT60TR13C::enable_test_mode()
{
  if(!set_bits(SFCTL_ADDR, TEST_MODE_EN)) 
    return BGT_status::BGT_error;

  // Init RFT0 Register
  if(!set_bits(RFT0_ADDR, TEST_IF_ENABLE)) 
    return BGT_status::BGT_error;

  return reset_fsm();
}

BGT_status BGT60TR13C::start_frame()
{
  return set_bits(MAIN_ADDR, START_FRAME);
}

BGT_status BGT60TR13C::read_fifo()
{
  // SPI Read
  digitalWrite(this->pin_cs, LOW);

  for(int i = 0; i < DATA_SIZE; i++)
  {
    this->header_GSR0[i] = this->radar_sensor_spi->transfer(ENABLE_BURST_MODE[i]);
  }
  
  // Check if Error Occured
  if((this->header_GSR0[DATA_SIZE-1] & 0x0F) != 0x0 
    	and (this->header_GSR0[DATA_SIZE-1] & 0x0F) != 0x4)
  {
    Serial.print("Status Register Error! GSR0 = ");
    Serial.println(this->header_GSR0[DATA_SIZE-1], HEX);
    return BGT_status::BGT_error;
  }

  for(size_t i = 0; i < this->frame_size; i++)
  {
    this->data[i] = this->radar_sensor_spi->transfer(0x00);
  }

  digitalWrite(this->pin_cs, HIGH);
    
  return BGT_status::BGT_success;
}

/**
 * @brief Converts FFT data to dB scale.
 * @param fft_data Pointer to the FFT data array.
 * @param length Length of the FFT data array.
 */
void convert_to_db(float * const fft_data, size_t const length) {
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

BGT_status BGT60TR13C::read_distance()
{
  if(!read_fifo()) return BGT_status::BGT_error;
  
  if(!unpack_adc_data()) return BGT_status::BGT_error;
  if(!apply_highpass_filter()) return BGT_status::BGT_error;

  // run FFT
  this->FFT.compute(FFTDirection::Forward);
  this->FFT.complexToMagnitude();

  convert_to_db(this->vReal, this->word_size);
  
  if(!apply_anti_coupling_filter()) return BGT_status::BGT_error;
    
  return BGT_status::BGT_success;
}

BGT_status BGT60TR13C::init_sensor()
{
  for (int i = 0; i < SIZE_REG_FILE; i++) 
  {
    reg_pair* current_reg = &((this->register_values)[i]);
    int data = current_reg->data;
    int addr = current_reg->addr;

    if(!write_reg(addr, data)) 
    {
      return BGT_status::BGT_error;
    }
  }
  return BGT_status::BGT_success;
}

BGT_status BGT60TR13C::reset_fifo()
{
  return set_bits(MAIN_ADDR, FIFO_RESET);
}

BGT_status BGT60TR13C::reset_fsm()
{
  return set_bits(MAIN_ADDR, FSM_RESET);
}

BGT_status BGT60TR13C::reset()
{
  return write_reg(MAIN_ADDR, SOFT_RESET);
}

size_t BGT60TR13C::calculate_RTU(size_t const adc_div, size_t const samples_per_chirp)
{
  return ((size_t)((adc_div * samples_per_chirp)/8)) + T_SETUP;
}

size_t BGT60TR13C::calculate_FSU(size_t const start_freq)
{
  // 24 bit two complement needed for sensor
  return size_t(pow(2,20) * ((((float)start_freq/640000))-96)) & 0xFFFFFF;
}

size_t BGT60TR13C::calculate_RSU(size_t const bandwidth, size_t const RTU)
{
  float dRF = bandwidth/(8*RTU);
  return size_t(pow(2,20) * dRF / 640000);
}

BGT_status BGT60TR13C::set_bits(size_t const reg_addr, size_t const bits)
{
  if(!read_reg(reg_addr)) return BGT_status::BGT_error;
  int data = bytes_to_int(this->reg_data) | bits;
  if(!write_reg(reg_addr, data)) return BGT_status::BGT_error;

  return BGT_status::BGT_success;
}

BGT_status BGT60TR13C::unpack_adc_data()
{
  size_t byte_index = skipFirstValues;
  size_t fft_index = 0;
  size_t const len = this->frame_size;
  while (byte_index < len - 2)
  {
    // Extract 3 bytes from the byte array
    byte byte1 = this->data[byte_index];
    byte byte2 = this->data[byte_index + 1];
    byte byte3 = this->data[byte_index + 2];
    
    uint32_t a = ((((uint32_t)(byte1 & 0xFF)) << 4) 
              | (((uint32_t)(byte2 & 0xF0)) >> 4));
    
    uint32_t b = ((((uint32_t)(byte2 & 0x0F)) << 8) 
              | (((uint32_t)(byte3 & 0xFF)) >> 0));

    // Assign the 12-bit values to the real-register
    this->vReal[fft_index] = (float) a;
    this->vReal[fft_index + 1] = (float) b;

    // reset imag-register
    this->vImag[fft_index] = 0.0;
    this->vImag[fft_index + 1] = 0.0;

    fft_index += 2;
    byte_index += 3;
  }
  return BGT_status::BGT_success;
}

BGT_status BGT60TR13C::apply_anti_coupling_filter()
{
  // Calculation: Typical: sig - mov_avg
  // y[i] = x[i] - 1/N sum_(k = i - (N-1))^(i)(x[k])
  // y[i] = (N-1)/N * x[i] - 1/N sum_(k = i - (N-1))^(i-1)(x[k])
  // Meaning: b0 = (N-1)/N
  // all other bx = -1/N
  // For Decoupling we use a broad moving average to
  // calculate reflections out: N=10
  float x0 = 0.0, x1 = 0.0, x2 = 0.0, x3 = 0.0, x4 = 0.0, x5 = 0.0, x6 = 0.0, x7 = 0.0, x8 = 0.0, x9 = 0.0, y0 = 0.0;
  int i = 0;
  constexpr float bx = -0.1; // -1/N
  constexpr float b0 = 0.9; // (N-1)/N
  int limit = this->word_size;
  while (i < limit)
  {
    x9 = x8;
    x8 = x7;
    x7 = x6;
    x6 = x5;
    x5 = x4;
    x4 = x3;
    x3 = x2;
    x2 = x1;
    x1 = x0;
    x0 = this->vReal[i];
    y0 = x0*b0 + x1*bx + x2*bx + x3*bx + x4*bx + x5*bx + x6*bx + x7*bx + x8*bx + x9*bx;
    this->vReal[i] = y0;
    i++;
  }
    
  // Using the filter, reflections should be calculated out
  // To remove coupling between Rx and Tx decrease the values from the first 3 values
  int j = 0;
  while(j < 3)
  {
    this->vReal[j] = 0.0;
    j++;
  }

  // Nearer Targets get detected much better,
  // but we want to have one threshold level to detect
  // every value. Decreasing it using a linear function
  // helps here
  /*int n = 3; // start with 3, the other values cant be used anyways
  limit = 9;
  float k = 28.0; // Initial adjustment value
  while (n < limit)
  {
    if (k > 0.0)
    {
      this->vReal[n] -= k;
      k -= 3.0;
    }
    n++;
  }*/

  this->vReal[3] -= 28;
  this->vReal[4] -= 26;
  this->vReal[5] -= 19;
  this->vReal[6] -= 12;
  this->vReal[7] -= 11;
  this->vReal[8] -= 4;
    
  return BGT_status::BGT_success;
}


BGT_status BGT60TR13C::apply_highpass_filter()
{
  // Coefficients for Chebyshev 2nd Order High-Pass Filter
  constexpr float b0 = 0.943;
  constexpr float b1 = -1.885;
  constexpr float b2 = 0.943;
  constexpr float a1 = 1.881;
  constexpr float a2 = -0.890;

  // High-Pass Filter
  // Chebyshev 2nd Order
  float x0 = 0.0;
  float x1 = 0.0;
  float x2 = 0.0;
  float y0 = 0.0;
  float y1 = 0.0;
  float y2 = 0.0;
  size_t i = 0;
  size_t const limit = this->word_size;
  while (i < limit)
  {
    x2 = x1;
    x1 = x0;
    x0 = this->vReal[i];
    y2 = y1;
    y1 = y0;
    y0 = x0*b0 + x1*b1 + x2*b2 + y1*a1 + y2*a2; 
    this->vReal[i] = y0;
    i += 1;
  }
  return BGT_status::BGT_success;
}

BGT_status int_to_bytes(uint32_t data, byte* dataByte)
{
  uint32_t dataCopy = data;
  for (int i = DATA_SIZE - 1; i >= 0; i--) 
  {
    dataByte[i] = dataCopy & 0xFF;
    dataCopy = dataCopy >> 8;
  }
  return BGT_status::BGT_success;
}

size_t BGT60TR13C::fetch_from_regFile(size_t address)
{
  for (int i = 0; i < SIZE_REG_FILE; i++) {
    if(address == this->register_values[i].addr)
      return this->register_values[i].data;
  }
  return 0;
}

BGT_status BGT60TR13C::write_to_regFile(size_t address, size_t data)
{
  for (int i = 0; i < SIZE_REG_FILE; i++) {
    if(address == this->register_values[i].addr)
    {
      this->register_values[i].data = data;
      return BGT_status::BGT_success;
    }
  }
  return BGT_status::BGT_error;
}

int bytes_to_int(byte const * const dataByte)
{
  int data = 0;
  for (int i = 0; i < DATA_SIZE; i++) 
  {
    data = data << 8;
    data |= dataByte[i];
  }
  return data;
}

float* BGT60TR13C::get_fft_data()
{
  return this->vReal;
}

size_t BGT60TR13C::get_fft_length() 
{
  return this->word_size;
}

float calculate_range_from_index(int index, float range_resolution)
{
  return index * range_resolution;
}