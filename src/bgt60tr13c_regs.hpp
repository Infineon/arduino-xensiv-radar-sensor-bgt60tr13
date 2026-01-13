// ================================================
// Defines const values needed for BGT60TR13C Sensor
// Samuel Weissenbacher, 08.2025
// ================================================

#ifndef BGT60TR13C_DEFINE_HPP
#define BGT60TR13C_DEFINE_HPP

// ===========================
// Register Mask and Offsets
// ===========================
#define ADDR_MASK                (0xFE000000)
#define ADDR_OFFSET              (25)
#define DATA_MASK                (0x00FFFFFF)
#define DATA_OFFSET              (0)

#define BURST_MODE               (0xFF)
#define BURST_MODE_MASK          (0xFF000000)
#define BURST_MODE_OFFSET        (17)

#define GSR0_STATUS_FLAG_MASK    (0x0F)

// SFCTL Register
#define SFCTL_FIFO_CREF_MASK     (0x001FFF)
#define SFCTL_FIFO_CREF_OFFSET   (0 << 0)

// ADC0 Register
#define ADC0_DIV_OFFSET          (14)
#define ADC0_DIV_MASK            (0xFF << ADC0_DIV_OFFSET)

// APU0 Register
#define APU0_OFFSET              (0)
#define APU0_MASK                (0xFFF << APU0_OFFSET)

// PLL1_0 Register
#define PLL1_0_FSU_OFFSET        (0)
#define PLL1_0_FSU_MASK          (0xFFFFFF << PLL1_0_FSU_OFFSET)

#define PLL1_1_RSU_OFFSET        (0)
#define PLL1_1_RSU_MASK          (0xFFFFFF << PLL1_1_RSU_OFFSET)

#define PLL1_2_RTU_OFFSET        (0)
#define PLL1_2_RTU_MASK          (0x3FFF << PLL1_2_RTU_OFFSET)

// CSU1_2 VGA Gain Registers
#define CSU1_2_VGA_GAIN1_OFFSET  (2)
#define CSU1_2_VGA_GAIN1_MASK    (0x7 << CSU1_2_VGA_GAIN1_OFFSET)

#define CSU1_2_VGA_GAIN2_OFFSET  (7)
#define CSU1_2_VGA_GAIN2_MASK    (0x7 << CSU1_2_VGA_GAIN2_OFFSET)

#define CSU1_2_VGA_GAIN3_OFFSET  (12)
#define CSU1_2_VGA_GAIN3_MASK    (0x7 << CSU1_2_VGA_GAIN3_OFFSET)

// ===========================
// Control Flags
// ===========================
#define SFCTL_MISO_HS_READ       (1 << 16)
#define SFCTL_FIFO_LP_MODE       (1 << 13)
#define WRITE_EN                 (0x01000000)
#define START_FRAME              (0x01)
#define TEST_MODE_EN             (0x1 << 17)
#define FIFO_RESET               (1 << 3)
#define FSM_RESET                (1 << 2)
#define SOFT_RESET               (1 << 1)
#define TEST_IF_ENABLE           (7 << 18)
#define BG_EN                    (1 << 7)
#define MADC_ISOPD               (1 << 8)
#define CS_EN                    (1 << 4)
#define MADC_EN                  (1 << 10)

// ===========================
// Register Addresses
// ===========================
#define MAIN_ADDR                (0x00)
#define ADC0_ADDR                (0x01)
#define CHIP_ID_ADDR             (0x02)
#define STAT1_ADDR               (0x03)
#define PACR1_ADDR               (0x04)
#define PACR2_ADDR               (0x05)
#define SFCTL_ADDR               (0x06)
#define SADC_CTRL_ADDR           (0x07)
#define CSI_0_ADDR               (0x08)
#define CSI_1_ADDR               (0x09)
#define CSI_2_ADDR               (0x0A)
#define CSCI_ADDR                (0x0B)
#define CSDS_0_ADDR              (0x0C)
#define CSDS_1_ADDR              (0x0D)
#define CSDS_2_ADDR              (0x0E)
#define CSCDS_ADDR               (0x0F)
#define CSU1_0_ADDR              (0x10)
#define CSU1_1_ADDR              (0x11)
#define CSU1_2_ADDR              (0x12)
#define CSD1_0_ADDR              (0x13)
#define CSD1_1_ADDR              (0x14)
#define CSD1_2_ADDR              (0x15)
#define CSC1_ADDR                (0x16)
#define CSU2_0_ADDR              (0x17)
#define CSU2_1_ADDR              (0x18)
#define CSU2_2_ADDR              (0x19)
#define CSD2_0_ADDR              (0x1A)
#define CSD2_1_ADDR              (0x1B)
#define CSD2_2_ADDR              (0x1C)
#define CSC2_ADDR                (0x1D)
#define CSU3_0_ADDR              (0x1E)
#define CSU3_1_ADDR              (0x1F)
#define CSU3_2_ADDR              (0x20)
#define CSD3_0_ADDR              (0x21)
#define CSD3_1_ADDR              (0x22)
#define CSD3_2_ADDR              (0x23)
#define CSC3_ADDR                (0x24)
#define CSU4_0_ADDR              (0x25)
#define CSU4_1_ADDR              (0x26)
#define CSU4_2_ADDR              (0x27)
#define CSD4_0_ADDR              (0x28)
#define CSD4_1_ADDR              (0x29)
#define CSD4_2_ADDR              (0x2A)
#define CSC4_ADDR                (0x2B)
#define CCR0_ADDR                (0x2C)
#define CCR1_ADDR                (0x2D)
#define CCR2_ADDR                (0x2E)
#define CCR3_ADDR                (0x2F)
#define PLL1_0_ADDR              (0x30)
#define PLL1_1_ADDR              (0x31)
#define PLL1_2_ADDR              (0x32)
#define PLL1_3_ADDR              (0x33)
#define PLL1_4_ADDR              (0x34)
#define PLL1_5_ADDR              (0x35)
#define PLL1_6_ADDR              (0x36)
#define PLL1_7_ADDR              (0x37)
#define PLL2_0_ADDR              (0x38)
#define PLL2_1_ADDR              (0x39)
#define PLL2_2_ADDR              (0x3A)
#define PLL2_3_ADDR              (0x3B)
#define PLL2_4_ADDR              (0x3C)
#define PLL2_5_ADDR              (0x3D)
#define PLL2_6_ADDR              (0x3E)
#define PLL2_7_ADDR              (0x3F)
#define PLL3_0_ADDR              (0x40)
#define PLL3_1_ADDR              (0x41)
#define PLL3_2_ADDR              (0x42)
#define PLL3_3_ADDR              (0x43)
#define PLL3_4_ADDR              (0x44)
#define PLL3_5_ADDR              (0x45)
#define PLL3_6_ADDR              (0x46)
#define PLL3_7_ADDR              (0x47)
#define PLL4_0_ADDR              (0x48)
#define PLL4_1_ADDR              (0x49)
#define PLL4_2_ADDR              (0x4A)
#define PLL4_3_ADDR              (0x4B)
#define PLL4_4_ADDR              (0x4C)
#define PLL4_5_ADDR              (0x4D)
#define PLL4_6_ADDR              (0x4E)
#define PLL4_7_ADDR              (0x4F)
#define RFT0_ADDR                (0x55)
#define RFT1_ADDR                (0x56)
#define PLL_DFT0_ADDR            (0x59)
#define STAT0_ADDR               (0x5D)
#define NONE_ADDR                (0x5B)
#define SADC_RESULT_ADDR         (0x5E)
#define FIFO_FSTAT_ADDR          (0x5F)
#define FIFO_ADDR                (0x60)

// ===========================
// FIFO Configuration
// ===========================
#define FIFO_SIZE                (8192)
#define FIFO_SIZE_BYTE           (12288)
#define DATA_SIZE                (4)

// FIFO burst mode enable command
// ===========================
// 0xFF = Adress; 
// FIFO-Address is shifted by (<< 1)
// Address 0x60 << 1 = 0xC0
// But when accessing this address, spi returns error!
// Solution (optimizable!) is to read 3 addresses before
// and skip those values later
const byte ENABLE_BURST_MODE[4] = {0xFF, 0xBD, 0x00, 0x00};

// ===========================
// Timing Configuration
// ===========================
// See Setup Saw-Tooth p.24 of BGT60TRxx Datasheet
// T_SETUP > T_PAEN + T_SSTART - T_START
constexpr size_t T_SETUP = 60; // [6us * 8/t_sys]


// ===========================
// Physical Constants
// ===========================
constexpr size_t F_ADC_CLK = 80;  // 80 MHz
constexpr size_t STEP_CHIRP_DIVIDER = 8;
constexpr size_t SPEED_OF_LIGHT = 300;  // in Mm/s

typedef struct{
    size_t addr;
    size_t data;
} reg_pair;

// Init Register Values for sensor
constexpr int SIZE_REG_FILE = 38;
const reg_pair init_register_list[SIZE_REG_FILE] = {
    {MAIN_ADDR, 0x011e8270},
    {ADC0_ADDR, 0x03088210},
    {PACR1_ADDR, 0x09e967fd},
    {PACR2_ADDR, 0x0b0805b4},
    {SFCTL_ADDR, 0x0d1027ff},
    {SADC_CTRL_ADDR, 0x0f010700},
    {CSI_0_ADDR, 0x11000000},
    {CSI_1_ADDR, 0x13000000},
    {CSI_2_ADDR, 0x15000000},
    {CSCI_ADDR, 0x17000be0},
    {CSDS_0_ADDR, 0x19000000},
    {CSDS_1_ADDR, 0x1b000000},
    {CSDS_2_ADDR, 0x1d000000},
    {CSCDS_ADDR, 0x1f000b60},
    {CSU1_0_ADDR, 0x21103c51},
    {CSU1_1_ADDR, 0x231ff41f},
    {CSU1_2_ADDR, 0x25006f73},
    {CSC1_ADDR, 0x2d000490},
    {CSC2_ADDR, 0x3b000480},
    {CSC3_ADDR, 0x49000480},
    {CSC4_ADDR, 0x57000480},
    {CCR0_ADDR, 0x5911be0e},
    {CCR1_ADDR, 0x5b44c40a}, // T_START = (0x0a * 8 + 10 * t_sys = 1.125us
    {CCR2_ADDR, 0x5d000000},
    {CCR3_ADDR, 0x5f787e1e}, // T_PAEN = 0x1e * 8 * t_sys = 3us; 
                             //T_SSTART = (0x1e * 8 + 1 * t_sys = 3.0125us
    {PLL1_0_ADDR, 0x61f5208a},
    {PLL1_1_ADDR, 0x630000a4},
    {PLL1_2_ADDR, 0x65000252},
    {PLL1_3_ADDR, 0x67000080},
    {PLL1_4_ADDR, 0x69000000},
    {PLL1_5_ADDR, 0x6b000000},
    {PLL1_6_ADDR, 0x6d000000},
    {PLL1_7_ADDR, 0x6f093910},
    {PLL2_7_ADDR, 0x7f000100},
    {PLL3_7_ADDR, 0x8f000100},
    {PLL4_7_ADDR, 0x9f000100},
    {RFT1_ADDR, 0xad000000},
    {NONE_ADDR, 0xb7000000}
};

typedef reg_pair reg_file[SIZE_REG_FILE];

#endif // BGT60TRXX_DEFINE_HPP