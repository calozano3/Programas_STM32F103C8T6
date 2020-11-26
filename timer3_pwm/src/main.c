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

   Ejemplo de generación de PWM con Tim3 usando los cuatro canales
   canal 1 (GPIOA, pin6) con 10% de ciclo útil
   canal 2 (GPIOA, pin7) con 20% de ciclo útil
   canal 3 (GPIOB, pin0) con 50% de ciclo útil
   canal 4 (GPIOB, pin1) con 80% de ciclo útil
   Frecuencia de la señal: 30khz.
   Frecuencia de entrada: 72Mhz
   Frecuencia de entrada al contador: 24Mhz
   Periodo: 799+1 pulsos
   prescaler: 2

**********************************************************************/
#include "stm32f10x_conf.h"

int main(void)
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

  while(1)
  {

  }
}
