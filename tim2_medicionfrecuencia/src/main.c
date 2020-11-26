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
  ejemplo de medición de frecuencia con tim2 y canal1 (A0)
  también tiene un generador de pwm en los pines A6, A7, PB1 y PB1
  de 30kz a 10%, 20%, 50% y 80% de ciclo útil.
**********************************************************************/
#include "stm32f10x_conf.h"

void UART_Init(void);
void UART_numero(uint32_t adc_value);
void init_timer3pwm(void);

uint8_t flanco;
uint16_t conteo1, conteo2;
uint32_t frecuencia;

int main(void)
{
    UART_Init();
    init_timer3pwm();

    /// configuración del pin de entrada del canal 1, tim2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin= GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode= GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed= GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    /// configurar el tiempo base de tim2
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

    ///configuración de la captura de la señal de entrada IC
    TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInit(TIM2, &TIM_ICInitStruct);

    /// configuración de la interrupción por flanco de subida de la señal de entrada
    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel= TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority= 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority= 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd= ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    TIM_Cmd(TIM2, ENABLE);

    flanco= 0;
    conteo1= 0;
    conteo2= 0;

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

void init_timer3pwm(void)
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

void TIM2_IRQHandler(void)
{
    uint16_t pulsos;
    if(TIM_GetITStatus(TIM2, TIM_IT_CC1)== SET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        if(flanco== 0)
        {
            conteo1= TIM_GetCapture1(TIM2);
            flanco= 1;
        }
        else
        {
            flanco= 0;
            conteo2= TIM_GetCapture1(TIM2);
            if(conteo2 > conteo1)
            {
                pulsos= conteo2- conteo1;
            }
            else
            {
                pulsos= (65535- conteo1)+ conteo2;
            }
            frecuencia= (uint32_t) (SystemCoreClock/ pulsos);
            UART_numero(frecuencia);
        }
    }
}
