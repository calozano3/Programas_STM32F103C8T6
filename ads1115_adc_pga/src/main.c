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
 programa para manejar el módulo ads1115, convertidor ADC con ganancia programable.
 El resultado se ve en pantalla usando uart a 9600bps.
 maneja i2c2,
    stm32f103c8t6      ads115
 pin B10               (SCL)
     B11               (SDA)
                       +3.3 voltios.
**********************************************************************/
#include "stm32f10x_conf.h"

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

void ADS1115_Restart(void);
//=======================================
// funciones de i2c
void i2c_start();
void i2c_stop();
void i2c_address_direction(uint8_t address, uint8_t direction);
void i2c_transmit(uint8_t byte);
uint8_t i2c_receive_ack();
uint8_t i2c_receive_nack();
//=======================================
void writeRegister(uint8_t slaveAddr, uint8_t regAddr, uint16_t value);
uint16_t readRegister(uint8_t slaveAddr, uint8_t regAddr);
void setGain(uint16_t gain);
uint16_t getGain(void);
uint16_t readADC_SingleEnded(uint8_t channel);
void delay(uint16_t retardo);
int16_t readADC_Differential_0_1(void);
int16_t readADC_Differential_2_3();
void startComparator_SingleEnded(uint8_t channel, int16_t threshold);
int16_t getLastConversionResults(void);
void UART_Init(void);
void UART_numero(uint32_t adc_value);
void UART_mensaje(char* mensaje, uint8_t largo);

int main(void)
{
    UART_Init();
    ADS1115_Restart();
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

  while(1)
  {
      int16_t adc0, adc1, adc2, adc3;

      adc0 = readADC_SingleEnded(0);
      adc1 = readADC_SingleEnded(1);
      adc2 = readADC_SingleEnded(2);
      adc3 = readADC_SingleEnded(3);
      UART_mensaje("AIN0: ", 6); UART_numero(adc0);
      UART_mensaje("AIN1: ", 6); UART_numero(adc1);
      UART_mensaje("AIN2: ", 6); UART_numero(adc2);
      UART_mensaje("AIN3: ", 6); UART_numero(adc3);
      UART_mensaje(" \n: ", 2);

      delay(1000);
  }
}


void ADS1115_Restart()
{
    I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable I2C and GPIO clocks */
    RCC_APB1PeriphClockCmd(ADS1115_I2C_RCC_Periph, ENABLE);
    RCC_APB2PeriphClockCmd(ADS1115_I2C_RCC_Port | RCC_APB2Periph_AFIO, ENABLE);

    /* Configure I2C pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin = ADS1115_I2C_SCL_Pin | ADS1115_I2C_SDA_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(ADS1115_I2C_Port, &GPIO_InitStructure);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = ADS1115_I2C_Speed;

    I2C_Cmd(ADS1115_I2C, ENABLE);
    I2C_Init(ADS1115_I2C, &I2C_InitStructure);
    /* I2C Peripheral Enable */

    m_conversionDelay = ADS1115_CONVERSIONDELAY;
    m_bitShift = 0;
    m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
    m_i2cAddress= ADS1015_ADDRESS<<1;
};
//==============================================================================================================
void i2c_start()
{
    // Wait until I2Cx is not busy anymore
    while (I2C_GetFlagStatus(ADS1115_I2C, I2C_FLAG_BUSY));

    // Generate start condition
    I2C_GenerateSTART(ADS1115_I2C, ENABLE);

    // Wait for I2C EV5.
    // It means that the start condition has been correctly released
    // on the I2C bus (the bus is free, no other devices is communicating))
    while (!I2C_CheckEvent(ADS1115_I2C, I2C_EVENT_MASTER_MODE_SELECT));
}

void i2c_stop()
{
    // Generate I2C stop condition
    I2C_GenerateSTOP(ADS1115_I2C, ENABLE);
    // Wait until I2C stop condition is finished
    while (I2C_GetFlagStatus(ADS1115_I2C, I2C_FLAG_STOPF));
}

void i2c_address_direction(uint8_t address, uint8_t direction)
{
    // Send slave address
    I2C_Send7bitAddress(ADS1115_I2C, address, direction);

    // Wait for I2C EV6
    // It means that a slave acknowledges his address
    if (direction == I2C_Direction_Transmitter)
    {
        while (!I2C_CheckEvent(ADS1115_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    }
    else if (direction == I2C_Direction_Receiver)
    {
        while (!I2C_CheckEvent(ADS1115_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    }
}

void i2c_transmit(uint8_t byte)
{
    // Send data byte
    I2C_SendData(ADS1115_I2C, byte);
    // Wait for I2C EV8_2.
    // It means that the data has been physically shifted out and
    // output on the bus)
    while (!I2C_CheckEvent(ADS1115_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
}

uint8_t i2c_receive_ack()
{
    // Enable ACK of received data
    I2C_AcknowledgeConfig(ADS1115_I2C, ENABLE);
    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(ADS1115_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));

    // Read and return data byte from I2C data register
    return I2C_ReceiveData(ADS1115_I2C);
}

uint8_t i2c_receive_nack()
{
    // Disable ACK of received data
    I2C_AcknowledgeConfig(ADS1115_I2C, DISABLE);
    // Wait for I2C EV7
    // It means that the data has been received in I2C data register
    while (!I2C_CheckEvent(ADS1115_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));

    // Read and return data byte from I2C data register
    return I2C_ReceiveData(ADS1115_I2C);
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

void  delay(uint16_t retardo)
{
    for(uint32_t i= 0; i< (retardo*30000); i++)
    {
        asm("nop");
    }
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
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_HITHRESH,
                threshold << m_bitShift);

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);
}

int16_t getLastConversionResults(void)
{
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
