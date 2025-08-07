#include "cy_syslib.h"
#include <Arduino.h>
#include <SPI.h>
#include "bgt60trxx_lib.hpp"
#include "pins_arduino.h"

// define pins for spi communication
#define RSPI_MOSI 41
#define RSPI_MISO 42
#define RSPI_SCLK 43
#define RSPI_CS   44
#define RXRES_L   40

#define CHIP_FREQ 100000000

// local functions
//===========================

/*
  sets data bits inside register to '1' using a read-modify-write
  Returns BGT_ERROR on Error
*/
size_t set_bits(BGT_ptr sensor, size_t const reg_addr, size_t const bits);

/*
  Unpacks recorded adc data
  into real-part of fft.    
  
  words (adc) are represented in a byte array:
  a2 a1;  a0 b2;  b1 b0
  
  Returns BGT_ERROR on Error
*/
size_t unpackRecData(BGT_ptr sensor);

/*
  High-Pass Filter Chebyscheff 2nd Order
  over real-part data before fft.
  
  Returns BGT_ERROR on Error
*/
size_t runHighPassFilter(BGT_ptr sensor);

/*
  returns a 4 byte array from 32 bit int.
  databyte needs to be 4 bytes long!!!
*/
size_t intToBytes(uint32_t data, byte* dataByte);

/*
  Fetches data from the RegFile
  using a address.
  If no entry exists the function returns 0
*/
size_t fetchFromRegFile(BGT_ptr sensor, size_t address);

/*
  Writes to RegFile.
  Returns BGT_ERROR on error
*/
size_t writeToRegFile(BGT_ptr sensor, size_t address, size_t data);

/*
  returns the byte value of an 4 byte array
  in integer.
*/
int BytesToInt(byte* dataByte);

/*
  Because of misaligned address.
  Address of FIFO = 0x60.
  But when reading, sensor returns
  error!
  4 words (or 6 bytes) seems to
  be the minimum value where
  an SPI-Read works.
*/
static const size_t skipFirstValues = 6;

// Create an instance of SPIClassPSOC for SPI communication as slave
static SPIClassPSOC SPI1 = SPIClassPSOC(
  RSPI_MOSI, 
  RSPI_MISO, 
  RSPI_SCLK, 
  NC, 
  false
);
static SPIClassPSOC *radar_sensor_spi = &SPI1;

bgt60trxx_struct* initStruct(
  size_t const word_size, 
  voidFuncPtr interrupt_handler
)
{
  bgt60trxx_struct* ret = (bgt60trxx_struct*) malloc(sizeof(bgt60trxx_struct));
  if(ret == 0) return 0;

  ret->word_size = word_size;
  ret->frame_size = (size_t)(((float)word_size)*1.5 + 0.5);

  //Check if fifo overflow would happen
  if (ret->frame_size > (FIFO_SIZE_BYTE - skipFirstValues))
  {
    free(ret);
    return 0;
  }  
  ret->frame_size  += skipFirstValues;

  ret->data = (byte*) malloc(sizeof(byte)*(ret->frame_size));
  if(ret->data == 0)
  {
    free(ret);
    return 0;
  }

  for(size_t i = 0; i < SIZE_REG_FILE; i++)
    (ret->register_values)[i] = init_register_list[i];

  ret->interrupt_handler = interrupt_handler;
  if(interrupt_handler != 0)
  {
    // Set the IRQ handler
    attachInterrupt(digitalPinToInterrupt(RXRES_L), interrupt_handler, RISING);

    // Print a message to indicate that the IRQ handler has been set
    Serial.println("IRQ handler was set.");
  }

  //Reset Device
  pinMode(RXRES_L, OUTPUT);
  pinMode(RSPI_CS, OUTPUT);
  digitalWrite(RSPI_CS, HIGH);
  digitalWrite(RXRES_L, LOW);
  digitalWrite(RXRES_L, HIGH);
  
  // Set SPI Interface with 50 MHz
  radar_sensor_spi->begin();
  radar_sensor_spi->beginTransaction(
    SPISettings(50000000, MSBFIRST, SPI_MODE0)
  );
  
  //Chirp Configuration
  ret->start_freq = (fetchFromRegFile(ret, PLL1_0_ADDR) 
                      & PLL1_0_FSU_MASK);
  ret->Clk_Per_Chirp = (fetchFromRegFile(ret, PLL1_2_ADDR) 
                      & PLL1_2_RTU_MASK);
  ret->step_freq_chirp = (fetchFromRegFile(ret, PLL1_1_ADDR) 
                      & PLL1_1_RSU_MASK);

  ret->adc_div = (fetchFromRegFile(ret, ADC0_ADDR) 
                    & ADC0_DIV_MASK) 
                  >> ADC0_DIV_OFFSET;

  // FFT Config
  ret->vReal = (float*) malloc(sizeof(float)*(ret->word_size));
  ret->vImag = (float*) malloc(sizeof(float)*(ret->word_size));
  ret->FFT = ArduinoFFT<float>(
    ret->vReal, 
    ret->vImag, 
    ret->word_size, 
    CHIP_FREQ / ret->adc_div
  );

  return ret;
}

size_t deinitStruct(BGT_ptr sensor)
{
  free(sensor->data);
  free(sensor);
  return BGT_SUCCESS;
}

float get_range_resolution(constBGT_ptr sensor)
{
  static size_t f_adc_clk = 80; // in MHz
  static size_t step_chirp_divider = 8;
  
  size_t RSU = sensor->step_freq_chirp;
  size_t RTU = sensor->Clk_Per_Chirp;
  
  float delta_f_RF = step_chirp_divider 
                      * f_adc_clk 
                      * (((float)RSU) / pow(2,20));

  static size_t c0 = 300; // in Mm/s

  // Multiply with 8! -> See Datasheet BGT60TRXX P.56 RTU
  float bandwidth = (RTU * 8) * delta_f_RF;

  return c0 / (2 * bandwidth);
}

size_t read_reg(BGT_ptr sensor, size_t const reg_addr)
{
  // Only Address needs to be send
  // LSB = R/W = 0
  byte addr = (reg_addr << 1) & 0xFE;

  // SPI Read
  digitalWrite(RSPI_CS, LOW);

  // Send address and read GSR0-Status-Register
  sensor->regData[0] = radar_sensor_spi->transfer(addr); 
  for (int i = 1; i < DATA_SIZE; i++) { // Read data
    sensor->regData[i] = radar_sensor_spi->transfer(0x00);
  }
  digitalWrite(RSPI_CS, HIGH);

  // Check if Error Occured
  if((sensor->regData[0] & 0x0F) != 0x0 
      and (sensor->regData[0] & 0x0F) != 0x4)
  {
    Serial.print("Status Register Error! GSR0 = ");
    Serial.println(sensor->regData[0], HEX);
    return BGT_ERROR;
  }
    
  // Info!
  // Only use the lower bytes (Bit 23:0) -> Data
  // Rest is Header-Data
  return BGT_SUCCESS;
}

size_t write_reg(
  constBGT_ptr sensor, 
  size_t const reg_addr, 
  size_t const data
)
{
  //SPI Transfer Format: Addr - R/W - Data 
  int dataInt = ((reg_addr << ADDR_OFFSET) & ADDR_MASK);
  dataInt |= WRITE_EN;
  dataInt |= ((data << DATA_OFFSET) & DATA_MASK);

  static byte dataToSend[DATA_SIZE];
  intToBytes(dataInt, dataToSend);

  // SPI Write
  digitalWrite(RSPI_CS, LOW);
  for (int i = 0; i < DATA_SIZE; i++) {
    radar_sensor_spi->transfer(dataToSend[i]); // Write data
  }
  digitalWrite(RSPI_CS, HIGH);
  return BGT_SUCCESS;
}

size_t set_adc_div(BGT_ptr sensor, size_t const data)
{
  sensor->adc_div = data;
  sensor->FFT = ArduinoFFT<float>(
    sensor->vReal, 
    sensor->vImag, 
    sensor->word_size, 
    CHIP_FREQ / sensor->adc_div
  );
  return set_init_value(sensor,
                        data, 
                        ADC0_ADDR, 
                        ADC0_DIV_MASK, 
                        ADC0_DIV_OFFSET);
}

size_t set_chirp_len(BGT_ptr sensor, size_t const chirp_len)
{
  return set_init_value(sensor,
                        chirp_len, 
                        PLL1_3_ADDR, 
                        APU0_MASK, 
                        APU0_OFFSET);
}

size_t configure_chirp(BGT_ptr sensor, size_t const N_FSU, size_t const N_RTU, size_t const N_RSU)
{
    if(!set_init_value(
      sensor,
      N_FSU, 
      PLL1_0_ADDR, 
      PLL1_0_FSU_MASK, 
      PLL1_0_FSU_OFFSET)
    )
      return BGT_ERROR;
                    
    if(!set_init_value(
      sensor,
      N_RSU, 
      PLL1_1_ADDR, 
      PLL1_1_RSU_MASK, 
      PLL1_1_RSU_OFFSET)
    )
      return BGT_ERROR;

    if(!set_init_value(
      sensor,
      N_RTU, 
      PLL1_2_ADDR, 
      PLL1_2_RTU_MASK, 
      PLL1_2_RTU_OFFSET)
    )
      return BGT_ERROR;

    // set values for range resoultion
    sensor->start_freq = N_FSU;
    sensor->step_freq_chirp = N_RSU;
    sensor->Clk_Per_Chirp = N_RTU;

    return BGT_SUCCESS;
}

size_t set_vga_gain_ch1(BGT_ptr sensor, size_t const gain)
{
  return set_init_value(sensor,
                  gain, 
                  CSU1_2_ADDR, 
                  CSU1_2_VGA_GAIN1_MASK, 
                  CSU1_2_VGA_GAIN1_OFFSET);
}

size_t set_init_value(
  BGT_ptr sensor, 
  size_t const data, 
  size_t const address, 
  size_t const reset_mask, 
  size_t const offset)
{
  size_t oldValue = fetchFromRegFile(sensor, address);
  size_t newValue = (oldValue & ~reset_mask) | (data << offset);
  if(!writeToRegFile(sensor, address, newValue))
    return BGT_ERROR;
  return BGT_SUCCESS;
}

size_t setCompareValue(
  BGT_ptr sensor, 
  size_t const compare_value
)
{
  if(compare_value >= 100)
  {
    return set_init_value(sensor,
                  FIFO_SIZE - 1, 
                  SFCTL_ADDR, 
                  SFCTL_FIFO_CREF_MASK, 
                  SFCTL_FIFO_CREF_OFFSET);
  }
  else
  {
    return set_init_value(sensor,
                  compare_value, 
                  SFCTL_ADDR, 
                  SFCTL_FIFO_CREF_MASK, 
                  SFCTL_FIFO_CREF_OFFSET);
  }
  return BGT_SUCCESS;
}

size_t enableTestMode(BGT_ptr sensor)
{
  if(!set_bits(sensor, SFCTL_ADDR, TEST_MODE_EN)) 
    return BGT_ERROR;

  // Init RFT0 Register
  if(!set_bits(sensor, RFT0_ADDR, TEST_IF_ENABLE)) 
    return BGT_ERROR;

  return resetFSM(sensor);
}

size_t startFrame(BGT_ptr sensor)
{
  return set_bits(sensor, MAIN_ADDR, START_FRAME);
}

size_t readFifo(BGT_ptr sensor)
{
  // SPI Read
  digitalWrite(RSPI_CS, LOW);

  for(int i = 0; i < DATA_SIZE; i++)
  {
    sensor->headerGSR0[i] = radar_sensor_spi->transfer(ENABLE_BURST_MODE[i]);
  }
  
  // Check if Error Occured
  if((sensor->headerGSR0[DATA_SIZE-1] & 0x0F) != 0x0 
    	and (sensor->headerGSR0[DATA_SIZE-1] & 0x0F) != 0x4)
  {
    Serial.print("Status Register Error! GSR0 = ");
    Serial.println(sensor->headerGSR0[DATA_SIZE-1], HEX);
    return BGT_ERROR;
  }

  for(size_t i = 0; i < sensor->frame_size; i++)
  {
    sensor->data[i] = radar_sensor_spi->transfer(0x00);
  }

  digitalWrite(RSPI_CS, HIGH);
    
  return BGT_SUCCESS;
}

size_t readDistance(BGT_ptr sensor)
{
  if(!readFifo(sensor)) return BGT_ERROR;
  
  if(!unpackRecData(sensor)) return BGT_ERROR;
  if(!runHighPassFilter(sensor)) return BGT_ERROR;

  // run FFT
  sensor->FFT.compute(FFTDirection::Forward);
  sensor->FFT.complexToMagnitude();
    
  return BGT_SUCCESS;
}

size_t initSensor(BGT_ptr sensor)
{
  for (int i = 0; i < SIZE_REG_FILE; i++) 
  {
    reg_pair* current_reg = &((sensor->register_values)[i]);
    int data = current_reg->data;
    int addr = current_reg->addr;

    if(!write_reg(sensor, addr, data)) 
    {
      return BGT_ERROR;
    }
  }
  return BGT_SUCCESS;
}

size_t resetFIFO(BGT_ptr sensor)
{
  return set_bits(sensor, MAIN_ADDR, FIFO_RESET);
}

size_t resetFSM(BGT_ptr sensor)
{
  return set_bits(sensor, MAIN_ADDR, FSM_RESET);
}

size_t reset(BGT_ptr sensor)
{
  if(!write_reg(sensor, MAIN_ADDR, SOFT_RESET))
    return BGT_ERROR;

  return BGT_SUCCESS;
}

size_t calculateRTU(size_t const adc_div, size_t const samples_per_chirp)
{
  return ((size_t)((adc_div * samples_per_chirp)/8)) + T_SETUP;
}

size_t calculateFSU(size_t const start_freq)
{
  // 24 bit two complement needed for sensor
  return size_t(pow(2,20) * ((((float)start_freq/640000))-96)) & 0xFFFFFF;
}

size_t calculateRSU(size_t const bandwidth, size_t const RTU)
{
  float dRF = bandwidth/(8*RTU);
  return size_t(pow(2,20) * dRF / 640000);
}

size_t checkData(float const* const data, size_t const length)
{
  size_t i = 0;
  while (i < length)
  {
    if(data[i] == 0x00)
    {
      Serial.print("Underflow detected! At index");
      Serial.println(i);
      return BGT_ERROR;
    }
    else if(data[i] == ~0x00)
    {
      Serial.print("Overflow detected! At index");
      Serial.println(i);
      return BGT_ERROR;
    }
    i += 1;
  }
  return BGT_SUCCESS;
}

size_t set_bits(BGT_ptr sensor, size_t const reg_addr, size_t const bits)
{
  if(!read_reg(sensor, reg_addr)) return BGT_ERROR;
  int data = BytesToInt(sensor->regData) | bits;
  if(!write_reg(sensor, reg_addr, data)) return BGT_ERROR;

  return BGT_SUCCESS;
}

size_t unpackRecData(BGT_ptr sensor)
{
  size_t byte_index = skipFirstValues;
  size_t fft_index = 0;
  size_t const len = sensor->frame_size;
  while (byte_index < len - 2)
  {
    // Extract 3 bytes from the byte array
    byte byte1 = sensor->data[byte_index];
    byte byte2 = sensor->data[byte_index + 1];
    byte byte3 = sensor->data[byte_index + 2];
    
    uint32_t a = ((((uint32_t)(byte1 & 0xFF)) << 4) 
              | (((uint32_t)(byte2 & 0xF0)) >> 4));
    
    uint32_t b = ((((uint32_t)(byte2 & 0x0F)) << 8) 
              | (((uint32_t)(byte3 & 0xFF)) >> 0));

    // Assign the 12-bit values to the real-register
    sensor->vReal[fft_index] = (float) a;
    sensor->vReal[fft_index + 1] = (float) b;

    // reset imag-register
    sensor->vImag[fft_index] = 0.0;
    sensor->vImag[fft_index + 1] = 0.0;

    fft_index += 2;
    byte_index += 3;
  }
  return BGT_SUCCESS;
}

size_t runHighPassFilter(BGT_ptr sensor)
{
  // High-Pass Filter
  // Chebyshev 2nd Order
  float x0 = 0.0;
  float x1 = 0.0;
  float x2 = 0.0;
  float y0 = 0.0;
  float y1 = 0.0;
  float y2 = 0.0;
  size_t i = 0;
  size_t const limit = sensor->word_size;
  while (i < limit)
  {
    x2 = x1;
    x1 = x0;
    x0 = sensor->vReal[i];
    y2 = y1;
    y1 = y0;
    y0 = x0*0.943 - x1*1.885 + x2*0.943 + y1*1.881 - y2*0.890; 
    sensor->vReal[i] = y0;
    i += 1;
  }
  return BGT_SUCCESS;
}

size_t intToBytes(uint32_t data, byte* dataByte)
{
  uint32_t dataCopy = data;
  for (int i = DATA_SIZE - 1; i >= 0; i--) 
  {
    dataByte[i] = dataCopy & 0xFF;
    dataCopy = dataCopy >> 8;
  }
  return BGT_SUCCESS;
}

size_t fetchFromRegFile(BGT_ptr sensor, size_t address)
{
  for (int i = 0; i < SIZE_REG_FILE; i++) {
    if(address == sensor->register_values[i].addr)
      return sensor->register_values[i].data;
  }
  return 0;
}

size_t writeToRegFile(BGT_ptr sensor, size_t address, size_t data)
{
  for (int i = 0; i < SIZE_REG_FILE; i++) {
    if(address == sensor->register_values[i].addr)
    {
      sensor->register_values[i].data = data;
      return BGT_SUCCESS;
    }
  }
  return BGT_ERROR;
}

int BytesToInt(byte* dataByte)
{
  int data = 0;
  for (int i = 0; i < DATA_SIZE; i++) 
  {
    data = data << 8;
    data |= dataByte[i];
  }
  return data;
}