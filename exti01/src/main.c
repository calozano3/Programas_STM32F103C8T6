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
   ejemplo de interrrupción por línea externa para encender y apagar el led

**********************************************************************/
#include "stm32f10x_conf.h"

#define TARJETA 1 // 0= tarjeta negra, 1= tarjeta azul

void LED_Init_tarjeta(void);
void apagar_led(void);
void encender_led(void);
uint8_t leer_led(void);

uint8_t estado;


int main(void)
{
    LED_Init_tarjeta();

    // configuración de la linea B4 como interrupción externa
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);// funciones alternas
    GPIO_InitTypeDef GPIOB_Struct;
    GPIOB_Struct.GPIO_Pin = GPIO_Pin_4;
    GPIOB_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOB_Struct.GPIO_Mode = GPIO_Mode_IPU;// resistencia a vcc, lectura de entrada en "1"
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_Init(GPIOB, &GPIOB_Struct);


    // configuracion de interrupcion del pin GPIOB04:
     GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);
     EXTI_InitTypeDef EXTI_InitStruct;

     EXTI_InitStruct.EXTI_Line = EXTI_Line4;
     EXTI_InitStruct.EXTI_LineCmd = ENABLE;
     EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
     EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
     EXTI_Init(&EXTI_InitStruct);

     NVIC_InitTypeDef NVIC_Struct;
     NVIC_Struct.NVIC_IRQChannel = EXTI4_IRQn;
     NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
     NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
     NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_Struct);

     estado= 0;

  while(1)
  {
      if(estado== 1)
      {
          if(leer_led() == 0)
          {
              apagar_led();
          }
          else
          {
              encender_led();
          }
          estado= 0;
      }
      for(uint32_t i= 0; i<2000000; i++)
      {
          asm("nop");
      }
  }
}

// funcion de tratamiento a interrupción
void EXTI4_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        estado= 1;
        EXTI_ClearITPendingBit(EXTI_Line4);
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
         GPIO_SetBits(GPIOC, GPIO_Pin_13);// apagar led
     }
     else
     {
         RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
         GPIOB_Struct.GPIO_Pin = GPIO_Pin_12;
         GPIO_Init(GPIOB, &GPIOB_Struct);
         GPIO_SetBits(GPIOB, GPIO_Pin_12);// apagar led
     }
}

// apaga el led de la tarjeta negra o azul
void apagar_led(void)
{
     if(TARJETA== 1)
     {
         GPIO_SetBits(GPIOC, GPIO_Pin_13);// apagar led
     }
     else
     {
         GPIO_SetBits(GPIOB, GPIO_Pin_12);// apagar led
     }
}

// enciende el led de la tarjeta negra o azul
void encender_led(void)
{
     if(TARJETA== 1)
     {
         GPIO_ResetBits(GPIOC, GPIO_Pin_13);// apagar led
     }
     else
     {
         GPIO_ResetBits(GPIOB, GPIO_Pin_12);// apagar led
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

