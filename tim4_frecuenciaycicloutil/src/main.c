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
 genera una señal pwm con una frecuencia de 30khz y cuatro salidas con la
 misma frecuencia y con ciclo útil de 10, 20, 50 y 80% en los pines
 PA6, PA7, PB0 y PB1, respectivamente.

 Ejemplo de medición de lectura de frecuencia y ciclo útil con el tim4, canal1 (PB6)
**********************************************************************/
#include "stm32f10x_conf.h"

void UART_Init(void);
void UART_numero(uint32_t adc_value);
void tim3_pwm(void);

int main(void)
{
    UART_Init();
    tim3_pwm();

    ///configuración del pin 6 del puerto B, canal 1 del tim4
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin= GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Speed= GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode= GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    ///configuración de la frecuencia base del tim4
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

    /// configuración del pwm de entrada
    TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_PWMIConfig(TIM4, &TIM_ICInitStruct);

    ///configuración del flanco para periodo y ciclo útil
    TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
    TIM_Cmd(TIM4, ENABLE);

    ///configuración de interrupción tim4, canal1
    TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel= TIM4_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority= 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority= 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd= ENABLE;
    NVIC_Init(&NVIC_InitStruct);

  while(1)
  {

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

void tim3_pwm(void)
{
    // configuracion de los pines de salida de los cuatro canales del tim3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_7;// canales 1 y 2
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1;// canales 3 y 4
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    // configuración del tiempo base del contador
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInitStruct.TIM_Period = 799; // 800 pulsos de 24mhz representa el periodo
    TIM_TimeBaseInitStruct.TIM_Prescaler = 2;// 72Mhz se divide en 3 y da 24Mhz.
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    // configuración del pwm de cada canal- canal 1
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 80; //ciclo util del 10%
    TIM_OC1Init(TIM3, &TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    // canal 2
    TIM_OCInitStruct.TIM_Pulse = 160; // 20%
    TIM_OC2Init(TIM3, &TIM_OCInitStruct);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    // canal 3
    TIM_OCInitStruct.TIM_Pulse = 400; // 50%
    TIM_OC3Init(TIM3, &TIM_OCInitStruct);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    // canal 4
    TIM_OCInitStruct.TIM_Pulse = 640; // 80%
    TIM_OC4Init(TIM3, &TIM_OCInitStruct);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    // empieza el contador a correr
    TIM_Cmd(TIM3, ENABLE);
}

void TIM4_IRQHandler(void)
{
    uint16_t periodo, ciclo_util;
    uint32_t frecuencia;

    if(TIM_GetITStatus(TIM4, TIM_IT_CC1)== SET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
        periodo= TIM_GetCapture1(TIM4);
        if(periodo > 0)
        {
            frecuencia= (uint32_t) (SystemCoreClock/ periodo);
            UART_numero(frecuencia);
            ciclo_util= (uint16_t) (TIM_GetCapture2(TIM4)* 100/ periodo);
            UART_numero(ciclo_util);
        }
        else
        {
            frecuencia= 0;
            ciclo_util= 0;
        }
    }
}
