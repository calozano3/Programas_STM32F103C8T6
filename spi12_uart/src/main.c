/*
**
**                   Comunicación por SPI
** Se usa SPI1 como maestro y SPI2 como esclavo y Uart1 para controlar la comunicación
** Lo que se escriba en uart1, va para spi1.
** Lo que llegue a spi1, va para spi2.
** las minúsculas las convierte en mayúsculas
** Lo que llegue a spi2, va para uart1
**
** SPI1_NSS (PA4)   -->  No conectar
   SPI1_SCK (PA5)   -->  SPI2_SCK (PB13)
   SPI1_MISO (PA6)  <--  SPI2_MISO (PB14)
   SPI1_MOSI (PA7)  -->  SPI2_MOSI (PB15)
   SPI2_NSS (PB12)  -->  GND
**
**********************************************************************/

#include "stm32f10x_conf.h"

void Sysclk_56M(void);
void UART_Init(void);
void Spi1_Maestro_Conf(void);
void Spi2_Esclavo_Conf(void);

int main(void)
{
    Sysclk_56M();
    UART_Init();
    Spi1_Maestro_Conf();
    Spi2_Esclavo_Conf();

    uint16_t dato;

  while(1)
  {
      // espera por caracter desde terminal uart
      while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
      dato= USART_ReceiveData(USART1);
      // el dato que lea del uart, lo manda a spi1
      SPI_I2S_SendData(SPI1, dato);
      // espera por dato que llegue a spi2
      while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)== RESET);
      dato= SPI_I2S_ReceiveData(SPI2);
      // conversión de los caracteres a mayúsculas
      if((dato> 96) && (dato< 123))
      {
          dato= dato- 32;
      }

      // dato leido de spi2 lo envia de nuevo al uart
      USART_SendData(USART1, dato);
  }
}


/***********************************************
 * Inicializacion del reloj Sysclk en 72 Mhz, igual en HCLK y PCLK2.
 * PCLK1 en la mitad
 ***********************************************/
void Sysclk_56M(void)
{
    // usa el reloj interno mientras se configura
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
    // deshabilita el pll para poder configurarlo
    RCC_PLLCmd(DISABLE);
    //8Mhz/2*14= 56Mhz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_14);
    RCC_PLLCmd(ENABLE);
    // espera que se estabilice
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)== RESET)
    {
    }
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    // HCLK = SYSCLK
    RCC_HCLKConfig( RCC_SYSCLK_Div1);
    // PCLK2 = HCLK (56Mhz)
    RCC_PCLK2Config( RCC_HCLK_Div1);
    // PCLK1 = HCLK/2 (28Mhz)
    RCC_PCLK1Config( RCC_HCLK_Div2);
}

/***********************************************
 * Inicializacion del puerto serial Serial1
 ***********************************************/
void UART_Init(void)
{
    GPIO_InitTypeDef GPIO_Struct;
    // Inicializa parametros de Uart 9600 bps, 8 bits, 1 stop, no paridad
    USART_InitTypeDef UART_Struct;
    USART_StructInit(&UART_Struct);
    UART_Struct.USART_BaudRate= 9600;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);

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

void Spi1_Maestro_Conf(void)
{
    //habilitación del reloj para spi1, funciones alternas y puerto A
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
    // Configuración de los pines del spi1
    GPIO_InitTypeDef GPIO_Struct;
    // GPIOA PIN5 (SPI1_SCK), PIN7(SPI1_MOSI) Y PIN4 (SPI1_NSS) función alterna- salidas
    GPIO_Struct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_4;
    GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_Struct);

    // GPIOA PIN6 (SPI1_MISO) funcion alterna- entrada
    GPIO_Struct.GPIO_Pin = GPIO_Pin_6;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_Struct);

    // configuración de SPI1 como maestro
    SPI_InitTypeDef SPI_InitStruct;
    // configuración de SPI1 como maestro con una frecuencia de 7 Mhz, maestro y NSS por software
    SPI_StructInit(&SPI_InitStruct);
    SPI_InitStruct.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8; // 56Mz/ 8= 7 Mbps
    SPI_InitStruct.SPI_Mode= SPI_Mode_Master;
    SPI_InitStruct.SPI_NSS= SPI_NSS_Soft;
    SPI_Init(SPI1, &SPI_InitStruct);

    // habilitación spi1
    SPI_Cmd(SPI1, ENABLE);

    return;
}

void Spi2_Esclavo_Conf(void)
{
    //habilitación del reloj para spi2, funciones alternas y puerto B
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    // Configuración de los pines del spi2
    GPIO_InitTypeDef GPIO_Struct;
    // GPIOB PIN14 (SPI2_MISO) función alterna- salida
    GPIO_Struct.GPIO_Pin = GPIO_Pin_14;
    GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_Struct);

    // GPIOB PIN12 (SPI2_NSS), PIN13 (SPI2_SCK) y PIN15 (SPI2_MOSI) funcion alterna- entradas
    GPIO_Struct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_Struct);

    // configuración del SPI2 como esclavo
    SPI_InitTypeDef SPI_InitStruct;
    // configuración de SPI2 como esclavo con una frecuencia de 7 Mhz, maestro y NSS por software
    SPI_StructInit(&SPI_InitStruct);
    SPI_InitStruct.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_4; // 28Mz/ 4= 7 Mbps
    SPI_InitStruct.SPI_Mode= SPI_Mode_Slave;
    SPI_InitStruct.SPI_NSS= SPI_NSS_Soft;
    SPI_Init(SPI2, &SPI_InitStruct);

    // habilitación SPIs
    SPI_Cmd(SPI1, ENABLE);
    SPI_Cmd(SPI2, ENABLE);

    return;
}


