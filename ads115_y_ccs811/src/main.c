/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $
   programa para lectura del módulo ccs811- Ultra-Low Power Digital Gas Sensor for
Monitoring Indoor Air Quality . Adaptado de la librería de Adafruit
   stm32f103c8t6       ccs811
i2c2, pin B10          (SCL)
          B11          (SDA)
          GND          WKE
                       +3.3 voltios.

programa para manejar el módulo ads1115, convertidor ADC con ganancia programable.
 El resultado se ve en pantalla usando uart a 9600bps.
 maneja i2c2,
    stm32f103c8t6      ads115
 pin B10               (SCL)
     B11               (SDA)
                       +3.3 voltios.
**********************************************************************/
#include "stm32f10x_conf.h"

//==============================================================================================================
/// lo siguiente es solo para el ads115

#define ADS1115_I2C                  I2C2
#define ADS1115_I2C_RCC_Periph       RCC_APB1Periph_I2C2
#define ADS1115_I2C_Port             GPIOB
#define ADS1115_I2C_SCL_Pin          GPIO_Pin_10
#define ADS1115_I2C_SDA_Pin          GPIO_Pin_11
#define ADS1115_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define ADS1115_I2C_Speed            100000

#define ADS1115_GND_ADDR 0x90
#define ADS1115_POW_ADDR 0x92
#define ADS1115_SCL_ADDR 0x96
#define ADS1115_SDA_ADDR 0x94

#define CONVER_REG_ADDR 0x00
#define CONFIG_REG_ADDR 0x01
#define LO_TRESH_REG_ADDR 0x02
#define HI_TRESH_REG_ADDR 0x03

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define ADS1015_ADDRESS (0x48) ///< 1001 000 (ADDR = GND)
/*=========================================================================*/

/*=========================================================================
    CONVERSION DELAY (in mS)
    -----------------------------------------------------------------------*/
#define ADS1015_CONVERSIONDELAY (1) ///< Conversion delay
#define ADS1115_CONVERSIONDELAY (9) ///< Conversion delay
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
#define ADS1015_REG_POINTER_MASK (0x03)      ///< Point mask
#define ADS1015_REG_POINTER_CONVERT (0x00)   ///< Conversion
#define ADS1015_REG_POINTER_CONFIG (0x01)    ///< Configuration
#define ADS1015_REG_POINTER_LOWTHRESH (0x02) ///< Low threshold
#define ADS1015_REG_POINTER_HITHRESH (0x03)  ///< High threshold
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
#define ADS1015_REG_CONFIG_OS_MASK (0x8000) ///< OS Mask
#define ADS1015_REG_CONFIG_OS_SINGLE                                           \
  (0x8000) ///< Write: Set to start a single-conversion
#define ADS1015_REG_CONFIG_OS_BUSY                                             \
  (0x0000) ///< Read: Bit = 0 when conversion is in progress
#define ADS1015_REG_CONFIG_OS_NOTBUSY                                          \
  (0x8000) ///< Read: Bit = 1 when device is not performing a conversion

#define ADS1015_REG_CONFIG_MUX_MASK (0x7000) ///< Mux Mask
#define ADS1015_REG_CONFIG_MUX_DIFF_0_1                                        \
  (0x0000) ///< Differential P = AIN0, N = AIN1 (default)
#define ADS1015_REG_CONFIG_MUX_DIFF_0_3                                        \
  (0x1000) ///< Differential P = AIN0, N = AIN3
#define ADS1015_REG_CONFIG_MUX_DIFF_1_3                                        \
  (0x2000) ///< Differential P = AIN1, N = AIN3
#define ADS1015_REG_CONFIG_MUX_DIFF_2_3                                        \
  (0x3000) ///< Differential P = AIN2, N = AIN3
#define ADS1015_REG_CONFIG_MUX_SINGLE_0 (0x4000) ///< Single-ended AIN0
#define ADS1015_REG_CONFIG_MUX_SINGLE_1 (0x5000) ///< Single-ended AIN1
#define ADS1015_REG_CONFIG_MUX_SINGLE_2 (0x6000) ///< Single-ended AIN2
#define ADS1015_REG_CONFIG_MUX_SINGLE_3 (0x7000) ///< Single-ended AIN3

#define ADS1015_REG_CONFIG_PGA_MASK (0x0E00)   ///< PGA Mask
#define ADS1015_REG_CONFIG_PGA_6_144V (0x0000) ///< +/-6.144V range = Gain 2/3
#define ADS1015_REG_CONFIG_PGA_4_096V (0x0200) ///< +/-4.096V range = Gain 1
#define ADS1015_REG_CONFIG_PGA_2_048V                                          \
  (0x0400) ///< +/-2.048V range = Gain 2 (default)
#define ADS1015_REG_CONFIG_PGA_1_024V (0x0600) ///< +/-1.024V range = Gain 4
#define ADS1015_REG_CONFIG_PGA_0_512V (0x0800) ///< +/-0.512V range = Gain 8
#define ADS1015_REG_CONFIG_PGA_0_256V (0x0A00) ///< +/-0.256V range = Gain 16

#define ADS1015_REG_CONFIG_MODE_MASK (0x0100)   ///< Mode Mask
#define ADS1015_REG_CONFIG_MODE_CONTIN (0x0000) ///< Continuous conversion mode
#define ADS1015_REG_CONFIG_MODE_SINGLE                                         \
  (0x0100) ///< Power-down single-shot mode (default)

#define ADS1015_REG_CONFIG_DR_MASK (0x00E0)   ///< Data Rate Mask
#define ADS1015_REG_CONFIG_DR_128SPS (0x0000) ///< 128 samples per second
#define ADS1015_REG_CONFIG_DR_250SPS (0x0020) ///< 250 samples per second
#define ADS1015_REG_CONFIG_DR_490SPS (0x0040) ///< 490 samples per second
#define ADS1015_REG_CONFIG_DR_920SPS (0x0060) ///< 920 samples per second
#define ADS1015_REG_CONFIG_DR_1600SPS                                          \
  (0x0080) ///< 1600 samples per second (default)
#define ADS1015_REG_CONFIG_DR_2400SPS (0x00A0) ///< 2400 samples per second
#define ADS1015_REG_CONFIG_DR_3300SPS (0x00C0) ///< 3300 samples per second

#define ADS1015_REG_CONFIG_CMODE_MASK (0x0010) ///< CMode Mask
#define ADS1015_REG_CONFIG_CMODE_TRAD                                          \
  (0x0000) ///< Traditional comparator with hysteresis (default)
#define ADS1015_REG_CONFIG_CMODE_WINDOW (0x0010) ///< Window comparator

#define ADS1015_REG_CONFIG_CPOL_MASK (0x0008) ///< CPol Mask
#define ADS1015_REG_CONFIG_CPOL_ACTVLOW                                        \
  (0x0000) ///< ALERT/RDY pin is low when active (default)
#define ADS1015_REG_CONFIG_CPOL_ACTVHI                                         \
  (0x0008) ///< ALERT/RDY pin is high when active

#define ADS1015_REG_CONFIG_CLAT_MASK                                           \
  (0x0004) ///< Determines if ALERT/RDY pin latches once asserted
#define ADS1015_REG_CONFIG_CLAT_NONLAT                                         \
  (0x0000) ///< Non-latching comparator (default)
#define ADS1015_REG_CONFIG_CLAT_LATCH (0x0004) ///< Latching comparator

#define ADS1015_REG_CONFIG_CQUE_MASK (0x0003) ///< CQue Mask
#define ADS1015_REG_CONFIG_CQUE_1CONV                                          \
  (0x0000) ///< Assert ALERT/RDY after one conversions
#define ADS1015_REG_CONFIG_CQUE_2CONV                                          \
  (0x0001) ///< Assert ALERT/RDY after two conversions
#define ADS1015_REG_CONFIG_CQUE_4CONV                                          \
  (0x0002) ///< Assert ALERT/RDY after four conversions
#define ADS1015_REG_CONFIG_CQUE_NONE                                           \
  (0x0003) ///< Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/

/** Gain settings */
typedef enum {
  GAIN_TWOTHIRDS = ADS1015_REG_CONFIG_PGA_6_144V,
  GAIN_ONE = ADS1015_REG_CONFIG_PGA_4_096V,
  GAIN_TWO = ADS1015_REG_CONFIG_PGA_2_048V,
  GAIN_FOUR = ADS1015_REG_CONFIG_PGA_1_024V,
  GAIN_EIGHT = ADS1015_REG_CONFIG_PGA_0_512V,
  GAIN_SIXTEEN = ADS1015_REG_CONFIG_PGA_0_256V
} adsGain_t;

uint8_t m_i2cAddress;      ///< the I2C address
uint8_t m_conversionDelay; ///< conversion deay
uint8_t m_bitShift;        ///< bit shift amount
uint16_t m_gain;          ///< ADC gain

void ADS1115_start(void);
void writeRegister(uint8_t slaveAddr, uint8_t regAddr, uint16_t value);
uint16_t readRegister(uint8_t slaveAddr, uint8_t regAddr);
void setGain(uint16_t gain);
uint16_t getGain(void);
uint16_t readADC_SingleEnded(uint8_t channel);
int16_t readADC_Differential_0_1(void);
int16_t readADC_Differential_2_3();
void startComparator_SingleEnded(uint8_t channel, int16_t threshold);
int16_t getLastConversionResults(void);
//==============================================================================================================
/// lo siguiente es solo para el ccs811

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define CCS811_ADDRESS (0x5A)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum {
  CCS811_STATUS = 0x00,
  CCS811_MEAS_MODE = 0x01,
  CCS811_ALG_RESULT_DATA = 0x02,
  CCS811_RAW_DATA = 0x03,
  CCS811_ENV_DATA = 0x05,
  CCS811_NTC = 0x06,
  CCS811_THRESHOLDS = 0x10,
  CCS811_BASELINE = 0x11,
  CCS811_HW_ID = 0x20,
  CCS811_HW_VERSION = 0x21,
  CCS811_FW_BOOT_VERSION = 0x23,
  CCS811_FW_APP_VERSION = 0x24,
  CCS811_ERROR_ID = 0xE0,
  CCS811_SW_RESET = 0xFF,
};

// bootloader registers
enum {
  CCS811_BOOTLOADER_APP_ERASE = 0xF1,
  CCS811_BOOTLOADER_APP_DATA = 0xF2,
  CCS811_BOOTLOADER_APP_VERIFY = 0xF3,
  CCS811_BOOTLOADER_APP_START = 0xF4
};

enum {
  CCS811_DRIVE_MODE_IDLE = 0x00,
  CCS811_DRIVE_MODE_1SEC = 0x01,
  CCS811_DRIVE_MODE_10SEC = 0x02,
  CCS811_DRIVE_MODE_60SEC = 0x03,
  CCS811_DRIVE_MODE_250MS = 0x04,
};

/*=========================================================================*/

#define CCS811_HW_ID_CODE 0x81

#define CCS811_REF_RESISTOR 100000

uint8_t _i2caddr;
float _tempOffset;

uint16_t _TVOC;
uint16_t _eCO2;

typedef struct
{
    /* 0: no error
     *  1: error has occurred
     */
    uint8_t ERROR : 1;
    // reserved : 2
    /* 0: no samples are ready
     *  1: samples are ready
     */
    uint8_t DATA_READY : 1;
    uint8_t APP_VALID : 1;
    // reserved : 2
    /* 0: boot mode, new firmware can be loaded
     *  1: application mode, can take measurements
     */
    uint8_t FW_MODE : 1;
} status;
status _status;

// measurement and conditions register
typedef struct
{
    // reserved : 2
    /* 0: interrupt mode operates normally
*  1: Interrupt mode (if enabled) only asserts the nINT signal (driven low) if
the new ALG_RESULT_DATA crosses one of the thresholds set in the THRESHOLDS
register by more than the hysteresis value (also in the THRESHOLDS register)
*/
    uint8_t INT_THRESH : 1;
    /* 0: int disabled
*  1: The nINT signal is asserted (driven low) when a new sample is ready in
                    ALG_RESULT_DATA. The nINT signal will stop being driven low
when ALG_RESULT_DATA is read on the I²C interface.
*/
    uint8_t INT_DATARDY : 1;
    uint8_t DRIVE_MODE : 3;
  } meas_mode;
meas_mode _meas_mode;

typedef struct
{
    /* The CCS811 received an I²C write request addressed to this station but
       with invalid register address ID */
    uint8_t WRITE_REG_INVALID : 1;
    /* The CCS811 received an I²C read request to a mailbox ID that is invalid
     */
    uint8_t READ_REG_INVALID : 1;
    /* The CCS811 received an I²C request to write an unsupported mode to
            MEAS_MODE */
    uint8_t MEASMODE_INVALID : 1;
    /* The sensor resistance measurement has reached or exceeded the maximum
            range */
    uint8_t MAX_RESISTANCE : 1;
    /* The Heater current in the CCS811 is not in range */
    uint8_t HEATER_FAULT : 1;
    /*  The Heater voltage is not being applied correctly */
    uint8_t HEATER_SUPPLY : 1;
  } error_id;
error_id _error_id;

float loge(float y);
unsigned int get_msb(int v);
void error_id_set(uint8_t data);
uint16_t getTVOC(void);
uint16_t geteCO2(void);
void setTempOffset(float offset);
uint8_t ccs811begin(uint8_t addr);
void setDriveMode(uint8_t mode);
void enableInterrupt(void);
void disableInterrupt(void);
uint8_t meas_mode_get(void);
uint8_t available(void);
uint8_t readData(void);
void setEnvironmentalData(uint8_t humidity, double temperature);
double calculateTemperature(void);
void setThresholds(uint16_t low_med, uint16_t med_high, uint8_t hysteresis);
void SWReset(void);
uint8_t checkError(void);
void status_set(uint8_t data);
//==============================================================================================================
/// funciones comunes para los dos sensores
void  delay(uint16_t retardo);
void write8(uint8_t direccion, uint8_t reg, uint8_t value);
uint8_t read8(uint8_t direccion, uint8_t reg);
void read(uint8_t direccion, uint8_t reg, uint8_t *buf, uint8_t num);
void write(uint8_t direccion, uint8_t reg, uint8_t *buf, uint8_t num);
void i2c_start();
void i2c_stop();
void i2c_address_direction(uint8_t address, uint8_t direction);
void i2c_transmit(uint8_t byte);
uint8_t i2c_receive_ack();
uint8_t i2c_receive_nack();
void configuracion_i2c2(void);
void writeRegister(uint8_t slaveAddr, uint8_t regAddr, uint16_t value);
uint16_t readRegister(uint8_t slaveAddr, uint8_t regAddr);
void UART_Init(void);
void UART_numero(uint32_t adc_value);
void UART_mensaje(char* mensaje, uint8_t largo);

int main(void)
{
    UART_Init();
    configuracion_i2c2();

    ADS1115_start();

    //UART_mensaje("1234567890123456789012345678901234567890123456789012345678901234567890\n", uint8_t largo);
    UART_mensaje("Getting single-ended readings from AIN0..3\n", 43);
    UART_mensaje("ADC Range: +/- 4.096V (1 bit = 2mV/ADS1015, 0.125mV/ADS1115)\n", 62);

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    setGain(ADS1015_REG_CONFIG_PGA_4_096V);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

    UART_mensaje("CCS811 test\n", 12);

    if(!ccs811begin(CCS811_ADDRESS))
    {
        UART_mensaje("Falla en test\n", 14);
        while(1);
    }

    // Wait for the sensor to be ready
    while(!available());
    int16_t adc0, adc1, adc2, adc3;
  while(1)
  {
      adc0 = readADC_SingleEnded(0);
      adc1 = readADC_SingleEnded(1);
      adc2 = readADC_SingleEnded(2);
      adc3 = readADC_SingleEnded(3);
      UART_mensaje("AIN0: ", 6); UART_numero(adc0);
      UART_mensaje("AIN1: ", 6); UART_numero(adc1);
      UART_mensaje("AIN2: ", 6); UART_numero(adc2);
      UART_mensaje("AIN3: ", 6); UART_numero(adc3);
      UART_mensaje(" \n: ", 2);

      if(available())
      {
          if(!readData())
          {
              UART_mensaje("CO2: ", 5);
              UART_numero(geteCO2());
              UART_mensaje("ppm, TVOC: ", 11);
              UART_numero(getTVOC());
          }
          else
          {
              UART_mensaje("ERROR! \n", 8);
              while(1);
          }
      }
  delay(500);
  }
}

//==============================================================================================================
/// las siguientes son funciones específicas del ads115

void ADS1115_start()
{
    m_conversionDelay = ADS1115_CONVERSIONDELAY;
    m_bitShift = 0;
    m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
    m_i2cAddress= ADS1015_ADDRESS<<1;
}

void setGain(uint16_t gain)
{
    m_gain = gain;
}

uint16_t getGain(void)
{
    return m_gain;
}

uint16_t readADC_SingleEnded(uint8_t channel)
{
  if (channel > 3) {
    return 0;
  }

  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set single-ended input channel
  switch (channel) {
  case (0):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
    break;
  case (1):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
    break;
  case (2):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
    break;
  case (3):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
    break;
  }

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
  return readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
}

int16_t readADC_Differential_0_1(void)
{
  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_0_1; // AIN0 = P, AIN1 = N

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res = readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

int16_t readADC_Differential_2_3()
{
  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_2_3; // AIN2 = P, AIN3 = N

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res = readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

void startComparator_SingleEnded(uint8_t channel, int16_t threshold)
{
  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_1CONV |   // Comparator enabled and asserts on 1
                                        // match
      ADS1015_REG_CONFIG_CLAT_LATCH |   // Latching mode
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_CONTIN |  // Continuous conversion mode
      ADS1015_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= m_gain;

  // Set single-ended input channel
  switch (channel) {
  case (0):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
    break;
  case (1):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
    break;
  case (2):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
    break;
  case (3):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
    break;
  }

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1015
  uint8_t buf[]= {(uint8_t) ((threshold << m_bitShift)>>8)& 0xff, (uint8_t) ((threshold << m_bitShift)& 0xff)};
  write(m_i2cAddress, ADS1015_REG_POINTER_HITHRESH, buf, 2);
  //writeRegister(m_i2cAddress, ADS1015_REG_POINTER_HITHRESH, threshold << m_bitShift);

  // Write config register to the ADC
  buf[0]=(uint8_t) (config>>8)& 0xff;
  buf[1]= (uint8_t) (config & 0xff);
  write(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, buf, 2);
  //writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);
}

int16_t getLastConversionResults(void)
{
  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint8_t buf[3];
  read(m_i2cAddress, ADS1015_REG_POINTER_CONVERT, buf, 2);
  uint16_t res= (uint16_t) (buf[0]<<8 + buf[1]);
  res= res >> m_bitShift;
  //uint16_t res = readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

//==============================================================================================================
/// las siguientes son funciones específicas del ccs811

void error_id_set(uint8_t data)
{
    _error_id.WRITE_REG_INVALID = data & 0x01;
    _error_id.READ_REG_INVALID = (data & 0x02) >> 1;
    _error_id.MEASMODE_INVALID = (data & 0x04) >> 2;
    _error_id.MAX_RESISTANCE = (data & 0x08) >> 3;
    _error_id.HEATER_FAULT = (data & 0x10) >> 4;
    _error_id.HEATER_SUPPLY = (data & 0x20) >> 5;
}

/**************************************************************************/
/*!
  @brief  returns the stored total volatile organic compounds measurement.
 This does does not read the sensor. To do so, call readData()
  @returns TVOC measurement as 16 bit integer
*/
/**************************************************************************/
uint16_t getTVOC(void)
{
    return _TVOC;
}

/**************************************************************************/
/*!
  @brief  returns the stored estimated carbon dioxide measurement. This does
 does not read the sensor. To do so, call readData()
  @returns eCO2 measurement as 16 bit integer
*/
/**************************************************************************/
uint16_t geteCO2(void)
{
    return _eCO2;
}

/**************************************************************************/
/*!
  @brief  set the temperature compensation offset for the device. This is
 needed to offset errors in NTC measurements.
  @param offset the offset to be added to temperature measurements.
*/
/**************************************************************************/
void setTempOffset(float offset)
{
    _tempOffset = offset;
}

/**************************************************************************/
/*!
    @brief  Setups the I2C interface and hardware and checks for communication.
    @param  addr Optional I2C address the sensor can be found on. Default is
   0x5A
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/
uint8_t ccs811begin(uint8_t addr)
{
    _i2caddr = addr<<1;

    SWReset();
    delay(100);

    // check that the HW id is correct
    if (read8(_i2caddr, CCS811_HW_ID) != CCS811_HW_ID_CODE)
    return 0;
    uint8_t buf[]= {0, 0};
    // try to start the app
    write(_i2caddr, CCS811_BOOTLOADER_APP_START, buf, 0);
    delay(100);

    // make sure there are no errors and we have entered application mode
    if (checkError())
    return 0;
    if (!_status.FW_MODE)
    return 0;
    disableInterrupt();
    // default to read every second
    setDriveMode(CCS811_DRIVE_MODE_1SEC);
    return 1;
}

/**************************************************************************/
/*!
    @brief  sample rate of the sensor.
    @param  mode one of CCS811_DRIVE_MODE_IDLE, CCS811_DRIVE_MODE_1SEC,
   CCS811_DRIVE_MODE_10SEC, CCS811_DRIVE_MODE_60SEC, CCS811_DRIVE_MODE_250MS.
*/
void setDriveMode(uint8_t mode)
{
  _meas_mode.DRIVE_MODE = mode;
  write8(_i2caddr, CCS811_MEAS_MODE, meas_mode_get());
}

/**************************************************************************/
/*!
    @brief  enable the data ready interrupt pin on the device.
*/
/**************************************************************************/
void enableInterrupt(void)
{
  _meas_mode.INT_DATARDY = 1;
  write8(_i2caddr, CCS811_MEAS_MODE, meas_mode_get());
}

/**************************************************************************/
/*!
    @brief  disable the data ready interrupt pin on the device
*/
/**************************************************************************/
void disableInterrupt(void)
{
  _meas_mode.INT_DATARDY = 0;
  write8(_i2caddr, CCS811_MEAS_MODE, meas_mode_get());
}

uint8_t meas_mode_get(void)
{
    return (_meas_mode.INT_THRESH << 2) | (_meas_mode.INT_DATARDY << 3) | (_meas_mode.DRIVE_MODE << 4);
}

/**************************************************************************/
/*!
    @brief  checks if data is available to be read.
    @returns True if data is ready, false otherwise.
*/
/**************************************************************************/
uint8_t available(void)
{
  status_set(read8(_i2caddr, CCS811_STATUS));
  if (!_status.DATA_READY)
    return 0;
  else
    return 1;
}

/**************************************************************************/
/*!
    @brief  read and store the sensor data. This data can be accessed with
   getTVOC() and geteCO2()
    @returns 0 if no error, error code otherwise.
*/
/**************************************************************************/
uint8_t readData(void)
{
  if (!available())
    return 0;
  else
  {
    uint8_t buf[8];
    read(_i2caddr, CCS811_ALG_RESULT_DATA, buf, 8);

    _eCO2 = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
    _TVOC = ((uint16_t)buf[2] << 8) | ((uint16_t)buf[3]);

    if (_status.ERROR)
      return buf[5];

    else
      return 0;
  }
}

/**************************************************************************/
/*!
    @brief  set the humidity and temperature compensation for the sensor.
    @param humidity the humidity data as a percentage. For 55% humidity, pass in
   integer 55.
    @param temperature the temperature in degrees C as a decimal number.
   For 25.5 degrees C, pass in 25.5
*/
/**************************************************************************/
void setEnvironmentalData(uint8_t humidity, double temperature)
{
  /* Humidity is stored as an unsigned 16 bits in 1/512%RH. The
  default value is 50% = 0x64, 0x00. As an example 48.5%
  humidity would be 0x61, 0x00.*/

  /* Temperature is stored as an unsigned 16 bits integer in 1/512
  degrees; there is an offset: 0 maps to -25°C. The default value is
  25°C = 0x64, 0x00. As an example 23.5% temperature would be
  0x61, 0x00.
  The internal algorithm uses these values (or default values if
  not set by the application) to compensate for changes in
  relative humidity and ambient temperature.*/

  uint8_t hum_perc = humidity << 1;

  float fractional = 0;//modf(temperature, &temperature);
  uint16_t temp_high = (((uint16_t)temperature + 25) << 9);
  uint16_t temp_low = ((uint16_t)(fractional / 0.001953125) & 0x1FF);

  uint16_t temp_conv = (temp_high | temp_low);

  uint8_t buf[] = {hum_perc, 0x00, (uint8_t)((temp_conv >> 8) & 0xFF),
                   (uint8_t)(temp_conv & 0xFF)};

  write(_i2caddr, CCS811_ENV_DATA, buf, 4);
}

/**************************************************************************/
/*!
    @deprecated hardware support removed by vendor
    @brief  calculate the temperature using the onboard NTC resistor.
    @returns temperature as a double.
*/
/**************************************************************************/
double calculateTemperature(void)
{
  uint8_t buf[4];
  read(_i2caddr, CCS811_NTC, buf, 4);

  uint32_t vref = ((uint32_t)buf[0] << 8) | buf[1];
  uint32_t vntc = ((uint32_t)buf[2] << 8) | buf[3];

  // from ams ccs811 app note
  uint32_t rntc = vntc * CCS811_REF_RESISTOR / vref;

  double ntc_temp;
  ntc_temp = (double) loge((float)rntc / CCS811_REF_RESISTOR); // 1 ¿como consigo el logaritmo aquí?
  ntc_temp /= 3380;
  ntc_temp /= 3380;                                   // 2
  ntc_temp += 1.0 / (25 + 273.15);                    // 3
  ntc_temp = 1.0 / ntc_temp;                          // 4
  ntc_temp -= 273.15;                                 // 5
  return ntc_temp - _tempOffset;
}

float loge(float y) {
    int log2;
    float divisor, x, result;

    log2 = get_msb((int)y); // See: https://stackoverflow.com/a/4970859/6630230
    divisor = (float)(1 << log2);
    x = y / divisor;    // normalized value between [1.0, 2.0]

    result = -1.7417939 + (2.8212026 + (-1.4699568 + (0.44717955 - 0.056570851 * x) * x) * x) * x;
    result += ((float)log2) * 0.69314718; // ln(2) = 0.69314718

    return result;
}

unsigned int get_msb(int v)
{
    int r = 31;                         // maximum number of iteration until integer has been totally left shifted out, considering that first bit is index 0. Also we could use (sizeof(int)) << 3 - 1 instead of 31 to make it work on any platform.

    while (!(v & 0x80000000) && r--) {   // mask of the highest bit
        v <<= 1;                        // multiply integer by 2.
    }
    return r;                           // will even return -1 if no bit was set, allowing error catch
}
/**************************************************************************/
/*!
    @brief  set interrupt thresholds
    @param low_med the level below which an interrupt will be triggered.
    @param med_high the level above which the interrupt will ge triggered.
    @param hysteresis optional histeresis level. Defaults to 50
*/
/**************************************************************************/
void setThresholds(uint16_t low_med, uint16_t med_high, uint8_t hysteresis)
{
  uint8_t buf[] = {(uint8_t)((low_med >> 8) & 0xF), (uint8_t)(low_med & 0xF), (uint8_t)((med_high >> 8) & 0xF), (uint8_t)(med_high & 0xF), hysteresis};

  write(_i2caddr, CCS811_THRESHOLDS, buf, 5);
}

/**************************************************************************/
/*!
    @brief  trigger a software reset of the device
*/
/**************************************************************************/
void SWReset(void)
{
  // reset sequence from the datasheet
  uint8_t seq[] = {0x11, 0xE5, 0x72, 0x8A};
  write(_i2caddr, CCS811_SW_RESET, seq, 4);
}

/**************************************************************************/
/*!
    @brief   read the status register and store any errors.
    @returns the error bits from the status register of the device.
*/
/**************************************************************************/
uint8_t checkError(void)
{
  status_set(read8(_i2caddr, CCS811_STATUS));
  return _status.ERROR;
}

void status_set(uint8_t data)
{
    _status.ERROR = data & 0x01;
    _status.DATA_READY = (data >> 3) & 0x01;
    _status.APP_VALID = (data >> 4) & 0x01;
    _status.FW_MODE = (data >> 7) & 0x01;
}

//==============================================================================================================

void  delay(uint16_t retardo)
{
    for(uint32_t i= 0; i< (retardo*30000); i++)
    {
        asm("nop");
    }
}

// escribe solo un byte
void write8(uint8_t direccion, uint8_t reg, uint8_t value)
{
  uint8_t buf[]= {value, 0};
  write(direccion, reg, buf, 1);
}

// lee solo un byte
uint8_t read8(uint8_t direccion, uint8_t reg)
{
  uint8_t ret;
  read(direccion, reg, &ret, 1);

  return ret;
}

// lee num bytes y los pone en buf. Necesita el registro reg
void read(uint8_t direccion, uint8_t reg, uint8_t *buf, uint8_t num)
{
  uint8_t i;

  i2c_start();
  i2c_address_direction(direccion, I2C_Direction_Transmitter);
  i2c_transmit(reg);
  i2c_stop();
  i2c_start();
  i2c_address_direction(direccion, I2C_Direction_Receiver);
  num--;
  for(i= 0; i< num; i++)
  {
      buf[i] = i2c_receive_ack();
  }
  buf[i] = i2c_receive_nack();
  i2c_stop();
}

// escribe num bytes de buf. Necesita el registro reg
void write(uint8_t direccion, uint8_t reg, uint8_t *buf, uint8_t num)
{
  i2c_start();
  i2c_address_direction(direccion, I2C_Direction_Transmitter);
  i2c_transmit((uint8_t)reg);
  for(uint8_t i= 0; i< num; i++)
  {
      i2c_transmit(buf[i]);
  }
  i2c_stop();
}

void i2c_start()
{
    // Wait until I2Cx is not busy anymore
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));

    // Generate start condition
    I2C_GenerateSTART(I2C2, ENABLE);

    // Wait for I2C EV5.
    // It means that the start condition has been correctly released
    // on the I2C bus (the bus is free, no other devices is communicating))
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
}

void i2c_stop()
{
    // Generate I2C stop condition
    I2C_GenerateSTOP(I2C2, ENABLE);
    // Wait until I2C stop condition is finished
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF));
}

void i2c_address_direction(uint8_t address, uint8_t direction)
{
    // Send slave address
    I2C_Send7bitAddress(I2C2, address, direction);

    // Wait for I2C EV6
    // It means that a slave acknowledges his address
    if (direction == I2C_Direction_Transmitter)
    {
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    }
    else if (direction == I2C_Direction_Receiver)
    {
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}

void i2c_transmit(uint8_t byte)
{
    // Send data byte
    I2C_SendData(I2C2, byte);
    // Wait for I2C EV8_2.
    // It means that the data has been physically shifted out and
    // output on the bus)
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
}

uint8_t i2c_receive_ack()
{
    // Enable ACK of received data
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));

    // Read and return data byte from I2C data register
    return I2C_ReceiveData(I2C2);
}

uint8_t i2c_receive_nack()
{
    // Disable ACK of received data
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));

    // Read and return data byte from I2C data register
    return I2C_ReceiveData(I2C2);
}

void configuracion_i2c2(void)
{
    I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable I2C and GPIO clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* Configure I2C pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;

    I2C_Cmd(I2C2, ENABLE);
    I2C_Init(I2C2, &I2C_InitStructure);
}

void writeRegister(uint8_t slaveAddr, uint8_t regAddr, uint16_t value)
//void i2c_write(uint8_t address, uint8_t data)
{
    i2c_start();
    //UART_mensaje("w01\n", 4);
    i2c_address_direction(slaveAddr, I2C_Direction_Transmitter);
    //UART_mensaje("w02\n", 4);
    i2c_transmit(regAddr);
    //UART_mensaje("w03\n", 4);
    i2c_transmit((uint8_t)((value & 0xFF00) >> 8));
    //UART_mensaje("w04\n", 4);
    i2c_transmit((uint8_t)(value & 0x00FF));
    //UART_mensaje("w05\n", 4);
    i2c_stop();
    //UART_mensaje("w06\n", 4);
}

uint16_t readRegister(uint8_t slaveAddr, uint8_t regAddr)
//void i2c_read(uint8_t address, uint8_t* data)
{
    uint16_t lectura;
    i2c_start();
    //UART_mensaje("r01\n", 4);
    i2c_address_direction(slaveAddr, I2C_Direction_Transmitter);
    //UART_mensaje("r02\n", 4);
    i2c_transmit(regAddr);
    //UART_mensaje("r03\n", 4);
    i2c_stop();
    //UART_mensaje("r04\n", 4);
    i2c_start();
    //UART_mensaje("r05\n", 4);
    i2c_address_direction(slaveAddr, I2C_Direction_Receiver);
    //UART_mensaje("1st: ", 5);
    lectura= i2c_receive_ack()<<8;
    //UART_numero(lectura);
    //UART_mensaje("2nd: ", 5);
    lectura= lectura+ i2c_receive_nack();
    //UART_numero(lectura);
    i2c_stop();
    return lectura;
    //UART_mensaje("r09\n", 4);
}

void UART_Init(void)
{
    GPIO_InitTypeDef GPIO_Struct;
    // Inicializa parametros de Uart 9600 bps, 8 bits, 1 stop, no paridad
    USART_InitTypeDef UART_Struct;
    USART_StructInit(&UART_Struct);
    UART_Struct.USART_BaudRate= 9600;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO |
    RCC_APB2Periph_GPIOA, ENABLE);
    // GPIOA PIN9 funcion alterna
    GPIO_Struct.GPIO_Pin = GPIO_Pin_9;
    GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_Struct);
    // GPIOA PIN9 funcion alterna
    GPIO_Struct.GPIO_Pin = GPIO_Pin_10;
    GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_Struct);
    USART_Cmd(USART1, ENABLE);
    USART_Init(USART1, &UART_Struct);
}

void UART_numero(uint32_t adc_value)
{
    uint16_t arreglo[]={'0', '0', '0', '0', '0', '0', '0', '0', '0', '0'};
    uint32_t a= adc_value;
    uint32_t b, c;
    uint8_t apuntador= 0;
    /// sacar los caracteres
    if(a== 0)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET);
        USART_SendData(USART1, '0');
    }
    while(a>0)
    {
        b= a/10;
        c= (uint32_t) (a- b*10);
        arreglo[apuntador]= (uint16_t) c + 48;
        apuntador++;
        a= b;
    }
    /// ahora mostrar en pantalla
    for(uint8_t i=0; i<apuntador; i++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET);
        USART_SendData(USART1, arreglo[apuntador-i-1]);
    }
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET);
    USART_SendData(USART1, '\n');
}

void UART_mensaje(char* mensaje, uint8_t largo)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, '\r');

    for(uint8_t i= 0; i<largo; i++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, mensaje[i]);
    }
    return;
}

