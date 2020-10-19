/*
**
**                           Main.c
** ejemplo de contador TIM3 como generador de señal en un pin

**********************************************************************/

#include "stm32f10x_conf.h"

#define TARJETA 1 // 0= tarjeta negra, 1= tarjeta azul

void LED_Init_tarjeta(void);
void apagar_led(void);
void encender_led(void);
uint8_t leer_led(void);

void UART_Init(void);
void UART_numero(uint32_t numero);

void Sysclk_56M(void);
void Tim3Init(void);
uint32_t get_timer_clock_frequency(void);

int main(void)
{
    Sysclk_56M();
    LED_Init_tarjeta();
    UART_Init();
    Tim3Init();
    UART_numero(get_timer_clock_frequency());

    // configuración de la salida
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 1000;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC1Init(TIM3, &TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

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
	TIM_TimeBaseInitStruct.TIM_Period = 2000-1; // se deja que el contador cuente hasta el límite
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // conteo ascendente.
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct); // configuración con datos
	//TIM_Cmd(TIM3, ENABLE);// habilitación

	// configuración de la interrupción cada segundo
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); // la bandera de interrupción es la que actualiza el contador
	NVIC_InitTypeDef NVIC_Struct;
    NVIC_Struct.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Struct.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_Struct);
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
 * envia un numero por el puerto serial
 ***********************************************/
void UART_numero(uint32_t numero)
{
    unsigned char tosend[]= {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0'};
    uint32_t valor1;
    uint8_t valor2;
    uint8_t contador= 0;

    if(numero== 0)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, numero+0x30);
    }

    while(numero>0)
    {
        valor1= (uint32_t) (numero/ 10);
        valor2= (uint8_t) (numero- valor1*10);
        tosend[contador]= valor2+ 0x30;
        numero= valor1;
        contador++;
    }

    for(uint8_t i= 0; i<contador; i++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, tosend[contador- i- 1]);
    }
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1,',');
    return;
}

void TIM2_IRQHandler(void)
{
    // verificar que la bandera de interrupción es la de actualización del contador
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        // cambia el estado del led
        if(leer_led()== 0)
            apagar_led();
        else
            encender_led();
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);// borra la bandera que hizo interrumpir
    }

}
