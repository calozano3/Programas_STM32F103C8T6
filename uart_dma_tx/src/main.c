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
// ejemplo de dma aplicado al usart1 en tx y rx, canal 4 y 5, respectivamente
// hace la siguiente secuencia:
// memoria a perif�rico:
// 1. lleva el contenido de memoria de 10 datos para ser transferidos por uart a una pc
// 2. se modifica el contenido de la memoria y se env�an los datos.
// perif�rico a memoria:
// 3. se espera por 10 datos enviados desde la pc
// memoria a perif�rico:
// 4. se modifica el contenido de los datos llegados a memoria y se env�an a la pc
//    usando dma con interrupci�n por finalizaci�n de transferencia.
**********************************************************************/
#include "stm32f10x_conf.h"

#define TARJETA 1 // 0= tarjeta negra, 1= tarjeta azul

void LED_Init_tarjeta(void);
void apagar_led(void);
void encender_led(void);
uint8_t leer_led(void);

void UART_Init(void);
void Sysclk_36M(void);

int main(void)
{
    LED_Init_tarjeta();
    UART_Init();

    // 1. memoria que tiene los datos para ser transferidos
    uint32_t memoria[]={'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    uint16_t num_datos= 10;

    // configuraci�n del dma
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel4);
    DMA_InitTypeDef DMA_InitStruct;
    DMA_StructInit(&DMA_InitStruct);
    // inicializaci�n de la direcci�n del perif�rico
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &USART1->DR;
    // direcci�n de la memoria que contiene los datos
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&memoria;
    // desde donde provienen los datos y hacia donde van
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
    // tama�o del buffer
    DMA_InitStruct.DMA_BufferSize = 10;
    // incrementar la posici�n del perif�rico?
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // incrementar la posici�n de la memoria?
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // cuantos bits son los datos del perif�rico
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
    // habilitaci�n del dma en usart1
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Channel4, ENABLE);

    // espera que se haga toda la transferencia de datos
    DMA_ClearFlag(DMA1_FLAG_TC4);
    while(DMA_GetFlagStatus(DMA1_FLAG_TC4)== RESET);
    //==================================================
    // 2. cambiar los datos iniciales
    for(uint16_t i= 0; i<num_datos; i++)
    {
        memoria[i]= num_datos-1-i+48;// 48 es para convertir el valor en ascii
    }
    // inicializa el contador de dma con el valor de datos
    DMA_Cmd(DMA1_Channel4, DISABLE);// primero se deshabilita el dma para hacer el cambio
    DMA_SetCurrDataCounter(DMA1_Channel4, num_datos);
    DMA_Cmd(DMA1_Channel4, ENABLE);
    DMA_ClearFlag(DMA1_FLAG_TC4);
    // esperar la finalizaci�n de la transferencia
    while(DMA_GetFlagStatus(DMA1_FLAG_TC4)== RESET);
    // encender el led

    //==================================================
    // 3. ahora se cambia la configuraci�n para enviar datos desde la pc
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA_DeInit(DMA1_Channel4);
    // desde donde provienen los datos y hacia donde van
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_Init(DMA1_Channel5, &DMA_InitStruct);
    // habilitaci�n del dma en usart1
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA1_Channel5, ENABLE);
    DMA_ClearFlag(DMA1_FLAG_TC5);
    // espera que se haga toda la transferencia de datos
    while(DMA_GetFlagStatus(DMA1_FLAG_TC5)== RESET);

    //==================================================
    // 4. ahora vuelve y env�a lo que hay en el buffer modificado
    for(uint16_t i= 0; i<num_datos; i++)
    {
        memoria[i]= memoria[i]-32;// 48 es para convertir el valor en ascii
    }
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_DeInit(DMA1_Channel5);
    // desde donde provienen los datos y hacia donde van
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_Init(DMA1_Channel4, &DMA_InitStruct);
    // habilitaci�n del dma en usart1
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    // configuraci�n de interrupci�n por terminaci�n de transferencia
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    NVIC_InitTypeDef NVIC_Struct;
    NVIC_Struct.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_Struct);

    DMA_Cmd(DMA1_Channel4, ENABLE);

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

// lee el estado del pin donde est� conectado el led
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

/***********************************************
 * Inicializacion del reloj Sysclk en 72 Mhz, igual en HCLK y PCLK2.
 * PCLK1 en la mitad
 ***********************************************/
void Sysclk_36M(void)
{
    // usa el reloj interno mientras se configura
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
    // deshabilita el pll para poder configurarlo
    RCC_PLLCmd(DISABLE);
    //8Mhz/2*14= 56Mhz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);// 4Mx9= 36M
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
    RCC_PCLK1Config( RCC_HCLK_Div2);
}

void DMA1_Channel4_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC4))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC4);
        encender_led();
    }

}
