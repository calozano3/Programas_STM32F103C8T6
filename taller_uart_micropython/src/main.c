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

**********************************************************************/
#include "stm32f10x_conf.h"

void Sysclk_56M(void);
void UART1_Init(void);
void UART2_Init(void);
void UART_numero(uint32_t adc_value);
void UART_mensaje(char* mensaje, uint8_t largo);

int main(void)
{
    Sysclk_56M();
    UART1_Init();
    UART2_Init();
    uint32_t ii;
    char caracter_wifi, caracter_hercules;

    // configuración del pin A1 como salida para conectarlo a RST del wifi
    GPIO_InitTypeDef GPIO_Struct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // GPIOA PIN1 como salida
    GPIO_Struct.GPIO_Pin = GPIO_Pin_1;
    GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_Struct);
    // poner el pin de RST del wifi en cero
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    for(ii= 0; ii< 200000; ii++)
    {
        asm("nop");
    }
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
    UART_mensaje("Iniciando...\n", 13);


  while(1)
  {
      // si hay caracteres desde hercules, se envían al wifi
      if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)== SET)
      {
          caracter_hercules= (char) USART_ReceiveData(USART1);
          while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)== RESET);
          USART_SendData(USART2, caracter_hercules);
      }
      // si hay carecteres desde wifi, se envían a hercules
      if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE)== SET)
      {
          caracter_wifi= (char) USART_ReceiveData(USART2);
          while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)== RESET);
          USART_SendData(USART1, caracter_wifi);
      }
  }
}

/***********************************************
 * Inicializacion del puerto serial Serial1
 ***********************************************/
void UART1_Init(void)
{
    GPIO_InitTypeDef GPIO_Struct;
    // Inicializa parametros de Uart 9600 bps, 8 bits, 1 stop, no paridad
    USART_InitTypeDef UART_Struct;
    USART_StructInit(&UART_Struct);
    UART_Struct.USART_BaudRate= 115200;
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

/***********************************************
 * Inicializacion del puerto serial Serial1
 ***********************************************/
void UART2_Init(void)
{
    GPIO_InitTypeDef GPIO_Struct;
    // Inicializa parametros de Uart 9600 bps, 8 bits, 1 stop, no paridad
    USART_InitTypeDef UART_Struct;
    USART_StructInit(&UART_Struct);
    UART_Struct.USART_BaudRate= 115200;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
    // GPIOA PIN2 funcion alterna
    GPIO_Struct.GPIO_Pin = GPIO_Pin_2;
    GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_Struct);
    // GPIOA PIN3 funcion alterna
    GPIO_Struct.GPIO_Pin = GPIO_Pin_3;
    GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_Struct);
    USART_Init(USART2, &UART_Struct);
    USART_Cmd(USART2, ENABLE);
}

/***********************************************
 * Inicializacion del reloj Sysclk en 56 Mhz, igual en HCLK y PCLK2.
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
    // PCLK2 = HCLK
    RCC_PCLK2Config( RCC_HCLK_Div1);
    // PCLK1 = HCLK/2
}

// esta función es para enviar el número de cualquier tamaño a uart ya convertido a ascii
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

/***********************************************
 * envia un mensaje por el puerto serial
 ***********************************************/
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
