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
// ejemplo de dma aplicado al usart1 en tx, canal 4
// lleva el contenido de memoria de 10 datos para ser transferidos por uart a una pc
**********************************************************************/
#include "stm32f10x_conf.h"

#define TARJETA 1 // 0= tarjeta negra, 1= tarjeta azul

void LED_Init_tarjeta(void);
void apagar_led(void);
void encender_led(void);
uint8_t leer_led(void);

void UART_Init(void);

int main(void)
{
    LED_Init_tarjeta();
    UART_Init();

    // memoria que tiene los datos para ser transferidos
    uint32_t memoria[]={'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    uint16_t num_datos= 10;

    // configuración del dma
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel4);
    DMA_InitTypeDef DMA_InitStruct;
    DMA_StructInit(&DMA_InitStruct);
    // inicialización de la dirección del periférico
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &USART1->DR;
    // dirección de la memoria que contiene los datos
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&memoria;
    // desde donde provienen los datos y hacia donde van
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
    // tamaño del buffer
    DMA_InitStruct.DMA_BufferSize = 10;
    // incrementar la posición del periférico?
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // incrementar la posición de la memoria?
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // cuantos bits son los datos del periférico
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    // bits de la memoria
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    // ciclico o normal para los datos en memoria
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    // prioridad
    DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
    // es transferencia entre memorias?
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStruct);
    // habilitación del dma en usart1
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Channel4, ENABLE);

    // espera que se haga toda la transferencia de datos
    while(DMA_GetFlagStatus(DMA1_FLAG_TC4)== RESET);
    // cambiar los datos iniciales
    for(uint16_t i= 0; i<num_datos; i++)
    {
        memoria[i]= num_datos-1-i+48;// 48 es para convertir el valor en ascii
    }
    // inicializa el contador de dma con el valor de datos
    DMA_Cmd(DMA1_Channel4, DISABLE);// primero se deshabilita el dma para hacer el cambio
    DMA_SetCurrDataCounter(DMA1_Channel4, num_datos);
    DMA_Cmd(DMA1_Channel4, ENABLE);

    // esperar la finalización de la transferencia
    while(DMA_GetFlagStatus(DMA1_FLAG_TC4)== RESET);
    // encender el led
    encender_led();

  while(1)
  {

  }
}

// configuracipon del led de la tarjeta negra o azul
void LED_Init_tarjeta(void)
{
     GPIO_InitTypeDef GPIOB_Struct;
     GPIOB_Struct.GPIO_Speed = GPIO_Speed_2MHz;
     GPIOB_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
     if(TARJETA== 1)
     {
         RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
         GPIOB_Struct.GPIO_Pin = GPIO_Pin_13;
         GPIO_Init(GPIOC, &GPIOB_Struct);
         GPIO_SetBits(GPIOC, GPIO_Pin_13);// apagar led tarjeta azul
     }
     else
     {
         RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
         GPIOB_Struct.GPIO_Pin = GPIO_Pin_12;
         GPIO_Init(GPIOB, &GPIOB_Struct);
         GPIO_SetBits(GPIOB, GPIO_Pin_12);// apagar led tarjeta negra
     }
}

// apaga el led de la tarjeta negra o azul
void apagar_led(void)
{
     if(TARJETA== 1)
     {
         GPIO_SetBits(GPIOC, GPIO_Pin_13);// apagar led tarjeta azul
     }
     else
     {
         GPIO_SetBits(GPIOB, GPIO_Pin_12);// apagar led tarjeta negra
     }
}

// enciende el led de la tarjeta negra o azul
void encender_led(void)
{
     if(TARJETA== 1)
     {
         GPIO_ResetBits(GPIOC, GPIO_Pin_13);// apagar led tarjeta azul
     }
     else
     {
         GPIO_ResetBits(GPIOB, GPIO_Pin_12);// apagar led tarjeta negra
     }
}

// lee el estado del pin donde está conectado el led
uint8_t leer_led(void)
{
     if(TARJETA== 1)
     {
         return GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13);
     }
     else
     {
         return GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12);
     }
}

/***********************************************
 * Inicializacion del puerto serial Serial1
 ***********************************************/
void UART_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_GPIOA, ENABLE);

    // configuracion de pines del uart1
    GPIO_InitTypeDef GPIO_Struct;
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

    // Inicializa parametros de Uart 9600 bps, 8 bits, 1 stop, no paridad
    USART_InitTypeDef UART_Struct;
    USART_StructInit(&UART_Struct);
    UART_Struct.USART_BaudRate= 9600;
    USART_Init(USART1, &UART_Struct);
    USART_Cmd(USART1, ENABLE);
}
