/*
**
**                           Main.c
** ejemplo de contador TIM1 contando pulsos internos
** se fija el conteo para que de un segundo.
** se usa interrupción de un segundo para encender y apagar el led.
**********************************************************************/

#include "stm32f10x_conf.h"

#define TARJETA 1 // 0= tarjeta negra, 1= tarjeta azul

void LED_Init_tarjeta(void);
void apagar_led(void);
void encender_led(void);
uint8_t leer_led(void);

void Sysclk_56M(void);
void Tim1Init(void);

int main(void)
{
    Sysclk_56M();
    LED_Init_tarjeta();
    Tim1Init();

    while(1)
    {

    }
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

// función para configurar el Tim1 como contador básico que de una interrupción cada segundo
void Tim1Init(void)
{
    // poner el reloj al contador Tim1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    // se escoge el reloj interno, el que sale de APB2, o sea, 56 Mhz, como entrada al contador Tim1
    TIM_InternalClockConfig(TIM1);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct; // creacion de estructura
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct); // inicialización de datos de la estructura
	// son 56M de entrada, dividido en 56000, queda 1000
	TIM_TimeBaseInitStruct.TIM_Prescaler = 56000; // con este valor se divide el reloj de entrada
	TIM_TimeBaseInitStruct.TIM_Period = 1000; // 56.000x1.000= 56M. O sea, queda 1 hz saliendo del contador
	//TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // conteo ascendente.
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct); // configuración con datos
	TIM_Cmd(TIM1, ENABLE);// habilitación

	// configuración de la interrupción cada segundo
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); // la bandera de interrupción es la que actualiza el contador
	NVIC_InitTypeDef NVIC_Struct;
    NVIC_Struct.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_Struct);
    return;
}

void TIM1_UP_IRQHandler(void)
{
    // verificar que la bandera de interrupción es la de actualización del contador
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        // cambia el estado del led
        if(leer_led()== 0)
            apagar_led();
        else
            encender_led();
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);// borra la bandera que hizo interrumpir
    }

}
