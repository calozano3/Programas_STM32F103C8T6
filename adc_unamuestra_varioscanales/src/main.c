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

   programa ejemplo que convierte varios canales de entrada cada dos segundos,
   usa el ADC1.
   Se usa el modo de rastreo de canales con DMA en forma de una
   muestra en vez de forma continua.
   Usa rtc para conteo de los dos segundos

**********************************************************************/
#include "stm32f10x_conf.h"

void Sysclk_56M(void);
void UART_Init(void);
void UART_numero(uint32_t adc_value);
void UART_mensaje(char* mensaje, uint8_t largo);
void adc1_continuo_un_canal(uint8_t canal);
void adc1_unamuestra_un_canal(uint8_t canal);
void adc1_unamuestra_varioscanales(uint8_t* canales, uint8_t cantidad);
void rtc_1segundo_config(void);

uint16_t destination[10];// arreglo donde van a quedar los datos de los canales convertidos
uint8_t bandera;// indicaci�n de retardo de un segundo con rtc

int main(void)
{
    Sysclk_56M();// configurar el sysclock para calcular la frecuencia del adc
    UART_Init();// antes de configurar el uart, debe configurar el sysclock
    rtc_1segundo_config();// configuraci�n rtc para un segundo con interrupci�n

    uint8_t canales[]= {ADC_Channel_0, ADC_Channel_1, ADC_Channel_8, ADC_Channel_9};

    adc1_unamuestra_varioscanales(canales, 4);

    bandera= 0;

  while(1)
  {
      if(bandera> 1)
      {
          UART_mensaje("\nA0=", 3);
          UART_numero(destination[0]);
          UART_mensaje("A1=", 3);
          UART_numero(destination[1]);
          UART_mensaje("B0=", 3);
          UART_numero(destination[2]);
          UART_mensaje("B1=", 3);
          UART_numero(destination[3]);
          ADC_SoftwareStartConvCmd(ADC1 , ENABLE);// inicia conversi�n
          bandera= 0;
      }
      asm("nop");// necesario para no dejar solo el if dentro del loop.
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

// esta funci�n es para enviar el n�mero de cualquier tama�o a uart ya convertido a ascii
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

/***********************************************
 * envia un mensaje por el puerto serial
 ***********************************************/
void UART_mensaje(char* mensaje, uint8_t largo)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, '\r');

    for(uint8_t i= 0; i<largo; i++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, mensaje[i]);
    }
    return;
}

void adc1_continuo_un_canal(uint8_t canal)
{
    // configuracion del PIN0 del puerto GPIOA
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;

    if(canal<8)
    {
        GPIO_InitStruct.GPIO_Pin = 1<<canal;// el canal se convierte en el # del pin
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else
    {
        GPIO_InitStruct.GPIO_Pin = 1<<(canal-8);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        GPIO_Init(GPIOB, &GPIO_InitStruct);
    }

    // configuraci�n del ADC
    // reloj para ADC (max 14MHz --> 56Mhz/4=14MHz)
    RCC_ADCCLKConfig (RCC_PCLK2_Div4);
    // habilita el reloj del ADC1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_InitTypeDef ADC_InitStruct;
    ADC_StructInit(&ADC_InitStruct);
    // convierte continuamente
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;// muestras cont�nuas
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;// inicio de conversi�n por software
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
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);// inicia la conversi�n en modo continuo
}

void adc1_unamuestra_un_canal(uint8_t canal)
{
    // configuracion del PIN0 del puerto GPIOA
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;

    if(canal<8)
    {
        GPIO_InitStruct.GPIO_Pin = 1<<canal;// el canal se convierte en el # del pin
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else
    {
        GPIO_InitStruct.GPIO_Pin = 1<<(canal-8);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        GPIO_Init(GPIOB, &GPIO_InitStruct);
    }

    // configuraci�n del ADC
    // reloj para ADC (max 14MHz --> 56Mhz/4=14MHz)
    RCC_ADCCLKConfig (RCC_PCLK2_Div4);
    // habilita el reloj del ADC1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitTypeDef ADC_InitStruct;
    // llenar informaci�n de la estructura del adc
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
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);// inicia la conversi�n en modo continuo
}

void adc1_unamuestra_varioscanales(uint8_t* canales, uint8_t cantidad)
{
    uint8_t ii;
    // configuracion de los pines de los canales del adc
    // habilitaci�n del ADC1 y GPIOA
    RCC_ADCCLKConfig(RCC_PCLK2_Div4);// 14M
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct; // estrutura para configurar los pines
    // configuraci�n de los pines del ADC (PA0 -> canal 0 a PA7 -> canal 7) como entradas analo�gicas

    GPIO_StructInit(&GPIO_InitStruct); // inicializaci�n de la estructura
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    // canales del puerto A
    for(ii= 0; ii< cantidad; ii++)
    {
        if(canales[ii]<8)
        {
            GPIO_InitStruct.GPIO_Pin = 1<<canales[ii];// el canal se convierte en el # del pin
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            GPIO_Init(GPIOA, &GPIO_InitStruct);
        }
    }
    // canales para el puerto B
    for(ii= 0; ii< cantidad; ii++)
    {
        if((canales[ii]>7) && (canales[ii] < 10))
        {
            GPIO_InitStruct.GPIO_Pin = 1<<(canales[ii]-8);// el canal se convierte en el # del pin
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
            GPIO_Init(GPIOB, &GPIO_InitStruct);
        }
    }

    ADC_InitTypeDef ADC_InitStruct;
    // configuraci�n del ADC1
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
    // multiples canales
    ADC_InitStruct.ADC_ScanConvMode = ENABLE;
    // modo de conversi�n cont�nuo
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
    // sin inicio de conversi�n externo
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    // alineamiento de presentaci�n de datos hacia la derecha
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    // 8 canales de conversi�n
    ADC_InitStruct.ADC_NbrOfChannel = cantidad;
    // carga informaci�n de configuraci�n
    ADC_Init(ADC1, &ADC_InitStruct);

    // configuraci�n de cada canal
    for(ii= 0; ii< cantidad; ii++)
    {
        ADC_RegularChannelConfig(ADC1, canales[ii], ii+1, ADC_SampleTime_239Cycles5);
    }

    // habilitaci�n de ADC1
    ADC_Cmd(ADC1, ENABLE);

    // calibraci�n
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    // activaci�n del reloj del DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // crea una estructura para DMA
    DMA_InitTypeDef DMA_InitStruct;
    //reset el canal channel1 del DMA1 a sus valores de inicio
    DMA_DeInit(DMA1_Channel1);
    // este canal se va a usar para transferencia desde perif�rico a memoria
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    //selecci�n de modo circular
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    //prioridad media
    DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
    //tama�o del dato de la fuente y el destino= 16bit
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    //habilitaci�n de incremento autom�tico en destino
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //La posici�n asignada al registro perif�rico ser� la fuente
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    //tama�o de los datos que se transfieren
    DMA_InitStruct.DMA_BufferSize = cantidad;
    //direcci�n de inicio de la fuente y el destino
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)destination;
    //programa registros del DMA
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);
    //habilita transferencia en el canal de DMA1
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // habilitaci�n de DMA para ADC
    ADC_DMACmd(ADC1, ENABLE);


    ADC_SoftwareStartConvCmd(ADC1 , ENABLE);// inicia conversi�n
}

void rtc_1segundo_config(void)
{
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
    // espera por sincronizaci�n
    RTC_WaitForSynchro();
    RTC_WaitForLastTask();
    // interrupci�n por segundos y alarma
    RTC_ITConfig(RTC_IT_ALR | RTC_IT_SEC, ENABLE);

    RTC_ClearITPendingBit(RTC_IT_SEC | RTC_IT_ALR | RTC_IT_OW);

    RTC_SetCounter(0);

    // configuraci�n de la interrupci�n por rtc
    NVIC_InitTypeDef NVIC_Struct;
    NVIC_Struct.NVIC_IRQChannel = RTC_IRQn;
    NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_Struct);
}

void RTC_IRQHandler(void)
{
    if(RTC_GetITStatus(RTC_IT_SEC)== SET)
    {
        bandera++;
        RTC_ClearITPendingBit(RTC_IT_SEC);
    }
    asm("nop");
}

