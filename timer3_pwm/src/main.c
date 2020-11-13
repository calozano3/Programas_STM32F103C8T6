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
Generación de una señal PWM con diferentes ciclo útil.
Se usa TIM3 con los cuatro canales de salida en los pines
GPIOA 6 y 7 y GPIOB 0 y 1.
Todos con 30khz a 10%, 20%, 50% y 80% de ciclo útil
**********************************************************************/
#include "stm32f10x_conf.h"

int main(void)
{
    // configuración de los pines del contador 3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);
    // GPIOA pin6 es canal 1, pin7 es canal 2
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // GPIOB pin0 es canal 3, pin1 es canal 4
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // configuración base del temporizador 3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    // frecuencia del sysclock es de 72Mhz. El prescaler es 2, o sea 72Mhz/ (2+1)= 24Mhz
    // 24Mhz entra an contador. Para tener 30khz a la salida, el contador debe contar por 799:
    // 30khz= 24Mhz/(799+1).
    // O sea, el contador cuenta 800 pulsos de 24Mhz en un periodo de una señal de 30khz.
    TIM_TimeBaseInitStruct.TIM_Period = 799;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 2;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0x0000;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

    // cofiguración del registro de comparación 1
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 80; // 10% de ciclo útil= 10*800/100
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    // configuración del registro de comparación 2
    TIM_OCInitStruct.TIM_Pulse = 160; // 20% de ciclo útil
    TIM_OC2Init(TIM3, &TIM_OCInitStruct);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    // configuración del registro de comparación 3
    TIM_OCInitStruct.TIM_Pulse = 400; // 50% de ciclo útil
    TIM_OC3Init(TIM3, &TIM_OCInitStruct);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    // configuración del registro de comparación 4
    TIM_OCInitStruct.TIM_Pulse = 640; // 80% de ciclo útil
    TIM_OC4Init(TIM3, &TIM_OCInitStruct);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

  while(1)
  {

  }
}

