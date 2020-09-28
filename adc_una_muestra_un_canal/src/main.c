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

   Programa ejemplo para convertir la señal conectada a PA0 a digital
   Toma una sola muestra y usa el software de conversión
   Envía el resultado de la conversión por Uart

**********************************************************************/
#include "stm32f10x_conf.h"

void Sysclk_56M(void);
void UART_Init(void);
void UART_numero(uint32_t adc_value);
void adc1_continuo_un_canal(uint8_t canal);
void adc1_unamuestra_un_canal(uint8_t canal);

int main(void)
{
    Sysclk_56M();// configurar el sysclock para calcular la frecuencia del adc
    UART_Init();// antes de configurar el uart, debe configurar el sysclock

    uint16_t adc_value;// variable para leer la conversión

    adc1_unamuestra_un_canal(ADC_Channel_5);

  while(1)
  {
      while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)== RESET);
      adc_value = ADC_GetConversionValue(ADC1);
      ADC_SoftwareStartConvCmd(ADC1, ENABLE);
      UART_numero(adc_value);// envía el valor de la convesión a uart
      for (int i = 0; i < 2000000; ++i) asm("nop");// retardo
  }
}

/***********************************************
 * Inicializacion del puerto serial Serial1
 ***********************************************/
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

// esta función es para enviar el número de cualquier tamaño a uart ya convertido a ascii
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

void adc1_continuo_un_canal(uint8_t canal)
{
    // configuracion del PIN0 del puerto GPIOA
    GPIO_InitTypeDef GPIOA_Struct;
    GPIOA_Struct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIOA_Struct.GPIO_Mode = GPIO_Mode_AIN;

    switch(canal)
    {
    case ADC_Channel_0:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_0;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_1:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_1;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_2:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_2;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_3:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_3;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_4:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_4;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_5:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_5;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_6:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_6;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_7:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_7;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_8:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_0;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
            GPIO_Init(GPIOB, &GPIOA_Struct);
            break;
    case ADC_Channel_9:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_1;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
            GPIO_Init(GPIOB, &GPIOA_Struct);
            break;
    default:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_0;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    }
    // configuración del ADC
    // reloj para ADC (max 14MHz --> 56Mhz/4=14MHz)
    RCC_ADCCLKConfig (RCC_PCLK2_Div4);
    // habilita el reloj del ADC1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_InitTypeDef ADC_InitStruct;
    ADC_StructInit(&ADC_InitStruct);
    // convierte continuamente
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;// muestras contínuas
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;// inicio de conversión por software
    ADC_Init(ADC1, &ADC_InitStruct); //ADC1
    // ADC1, canal 0: Entrada al pin PA0
    ADC_RegularChannelConfig(ADC1, canal, 1, ADC_SampleTime_13Cycles5);
    // habilita ADC1
    ADC_Cmd(ADC1, ENABLE);
    // calibracion del ADC
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    // habilita ADC1
    ADC_Cmd(ADC1, ENABLE);
    // inicia conversion por software
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);// inicia la conversión en modo continuo
}

void adc1_unamuestra_un_canal(uint8_t canal)
{
    // configuracion del PIN0 del puerto GPIOA
    GPIO_InitTypeDef GPIOA_Struct;
    GPIOA_Struct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIOA_Struct.GPIO_Mode = GPIO_Mode_AIN;

    switch(canal)
    {
    case ADC_Channel_0:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_0;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_1:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_1;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_2:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_2;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_3:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_3;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_4:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_4;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_5:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_5;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_6:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_6;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_7:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_7;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    case ADC_Channel_8:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_0;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
            GPIO_Init(GPIOB, &GPIOA_Struct);
            break;
    case ADC_Channel_9:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_1;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
            GPIO_Init(GPIOB, &GPIOA_Struct);
            break;
    default:
            GPIOA_Struct.GPIO_Pin = GPIO_Pin_0;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIOA_Struct);
            break;
    }
    // configuración del ADC
    // reloj para ADC (max 14MHz --> 56Mhz/4=14MHz)
    RCC_ADCCLKConfig (RCC_PCLK2_Div4);
    // habilita el reloj del ADC1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitTypeDef ADC_InitStruct;
    // llenar información de la estructura del adc
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
    // initialize the ADC_ScanConvMode member
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    // Initialize the ADC_ContinuousConvMode member
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
    // Initialize the ADC_ExternalTrigConv member
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    // Initialize the ADC_DataAlign member
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    // Initialize the ADC_NbrOfChannel member
    ADC_InitStruct.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStruct); //ADC1

    // ADC1, canal 0: Entrada al pin PA0
    ADC_RegularChannelConfig(ADC1, canal, 1, ADC_SampleTime_239Cycles5);
    // habilita ADC1
    ADC_Cmd(ADC1, ENABLE);
    // calibracion del ADC
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    // habilita ADC1
    ADC_Cmd(ADC1, ENABLE);
    // inicia conversion por software
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);// inicia la conversión en modo continuo
}


