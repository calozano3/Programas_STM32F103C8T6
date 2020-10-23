/*
**
**                           Main.c
** ejemplo de contador TIM3 como generador de señal en un pin
canal 1, pin A6

**********************************************************************/

#include "stm32f10x_conf.h"

void Sysclk_56M(void);
void Tim3Init(void);
uint32_t get_timer_clock_frequency(void);

int main(void)
{
    Sysclk_56M();
    Tim3Init();

    // configuración de la salida en el pin A6
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 1000;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC1Init(TIM3, &TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);// la comparación es la misma siempre

    // configuración del pin de salida del tim3, canal 1, GPIOA6
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIOB_Struct;
    GPIOB_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOB_Struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIOB_Struct.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIOB_Struct);

    TIM_Cmd(TIM3, ENABLE);// habilitación

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
    RCC_PCLK1Config(RCC_HCLK_Div2);
}

// función para configurar el Tim3
void Tim3Init(void)
{
    /// poner el reloj al contador Tim3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    /// se escoge el reloj interno, el que sale de APB1, o sea, 28 Mhz, como entrada al contador Tim3
    TIM_InternalClockConfig(TIM3);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct; // creacion de estructura
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct); // inicialización de datos de la estructura
	// son 28M de entrada, dividido en 28000, queda 1000
	TIM_TimeBaseInitStruct.TIM_Prescaler = (uint16_t) (get_timer_clock_frequency()/ 1000)-1; // asegura 1 ms a la entrada
	TIM_TimeBaseInitStruct.TIM_Period = 2000-1; // el contador cuenta hasta 2000 y retorna a cero
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // conteo ascendente.
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct); // configuración con datos

    return;
}

uint32_t get_timer_clock_frequency(void)
{
    RCC_ClocksTypeDef RCC_CLocks;
    RCC_GetClocksFreq(&RCC_CLocks);
    uint32_t multiplier;
    if(RCC_CLocks.PCLK1_Frequency == RCC_CLocks.SYSCLK_Frequency)
    {
        multiplier= 1;
    }
    else
    {
        multiplier= 2;
    }
    return multiplier*RCC_CLocks.PCLK1_Frequency;
}



