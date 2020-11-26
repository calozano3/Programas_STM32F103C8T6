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

int main(void)
{
    /// pin de entrada para la señal de activación del pulso: Tim4, canal 2
    /// pin de salida del pulso: Tim4, canal 1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode= GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin= GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Speed= GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin= GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode= GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /// configuración de la frecuencia base
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInitStruct.TIM_Prescaler= 10;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

    /// configuración del pulso de salida
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode= TIM_OCMode_PWM2;
    TIM_OCInitStruct.TIM_OutputState= TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse= 5958;
    TIM_OC1Init(TIM4, &TIM_OCInitStruct);

    /// configuración del pulso de entrada
    TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel= TIM_Channel_2;
    TIM_ICInit(TIM4, &TIM_ICInitStruct);

    /// configuración del modo de un pulso
    TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Single);
    TIM_SelectInputTrigger(TIM4, TIM_TS_ETRF);
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Trigger);
    TIM_Cmd(TIM4, ENABLE);

  while(1)
  {

  }
}
