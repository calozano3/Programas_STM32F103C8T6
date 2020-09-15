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
// ejemplo de encender y apagar el led de la tarjeta
// cada segundo usando RTC con interrupción por segundo y por alarma.
// Inicialmente interrumpe por segundos hasta los 5 segundos. Luego deshabilita
// la interrupción por segundos y habilita la de la alarma a los 10 segundos.
// en adelante, deshabilita la interrupción por alarma y sigue interrumpiendo por segundos.
// la información de interrupción y del contador rtc se muestra en el puerto serial,
**********************************************************************/

#include "stm32f10x_conf.h"


#define TARJETA 1 // 0= tarjeta negra, 1= tarjeta azul

void LED_Init_tarjeta(void);
void apagar_led(void);
void encender_led(void);
uint8_t leer_led(void);

void UART_Init(void);
void UART_numero(uint32_t numero);
void enviarpalabra(char *arreglo, uint16_t longitud);

uint8_t bandera;

int main(void)
{
    LED_Init_tarjeta();
    UART_Init();
    enviarpalabra("Ejemplo RTC con interrupción \n", 30);

    // habilitacion del reloj
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    // habilita acceso al rtc
    PWR_BackupAccessCmd(ENABLE);

    RCC_BackupResetCmd(ENABLE);
    RCC_BackupResetCmd(DISABLE);

    // habilita el reloj externo
    RCC_LSEConfig(RCC_LSE_ON);
    // espera hasta que el reloj se estabilice
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
    // selecciona el reloj externo como fuente
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    RTC_SetPrescaler(32767); // divisor del reloj externo
    // habilita el rtc
    RCC_RTCCLKCmd(ENABLE);
    // espera por sincronización
    RTC_WaitForSynchro();
    RTC_WaitForLastTask();
    // interrupción por segundos y alarma
    RTC_ITConfig(RTC_IT_ALR | RTC_IT_SEC, ENABLE);

    RTC_ClearITPendingBit(RTC_IT_SEC | RTC_IT_ALR | RTC_IT_OW);

    RTC_SetCounter(0);

    // configuración de la interrupción por rtc
    NVIC_InitTypeDef NVIC_Struct;
    NVIC_Struct.NVIC_IRQChannel = RTC_IRQn;
    NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_Struct);

    bandera= 0;

    while(1)
    {
        if(bandera== 1)
        {
            if(leer_led()== 0)
            {
                apagar_led();
                enviarpalabra("apaga led \n", 11);
            }
            else
            {
                encender_led();
                enviarpalabra("enciende led \n", 14);
            }
            enviarpalabra("contador= ", 10);
            UART_numero(RTC_GetCounter());
            enviarpalabra("\n",1);
            bandera= 0;
        }
        asm("nop");// necesario para no dejar solo el if dentro del loop.
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

void RTC_IRQHandler(void)
{
    uint32_t contador= RTC_GetCounter();
    if(RTC_GetITStatus(RTC_IT_ALR)== SET)
    {
        RTC_ITConfig(RTC_IT_SEC, ENABLE);
        enviarpalabra("ALR \n", 5);
        RTC_ClearITPendingBit(RTC_IT_ALR);
    }
    if(RTC_GetITStatus(RTC_IT_OW)== SET)
    {
        enviarpalabra("OW  \n", 5);
        RTC_ClearITPendingBit(RTC_IT_OW);
    }
    if(RTC_GetITStatus(RTC_IT_SEC)== SET)
    {
        enviarpalabra("SEC \n", 5);
        RTC_ClearITPendingBit(RTC_IT_SEC);
        if(contador== 5)
        {
            RTC_ITConfig(RTC_IT_SEC, DISABLE);
            RTC_ITConfig(RTC_IT_ALR, ENABLE);
            RTC_SetAlarm(10);
        }
    }

    bandera= 1;
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
    return;
}

void enviarpalabra(char *arreglo, uint16_t longitud)
{
    for(uint16_t i= 0; i< longitud; i++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, arreglo[i]);
    }
}
