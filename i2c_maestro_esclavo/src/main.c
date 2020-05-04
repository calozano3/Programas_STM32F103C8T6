/*********************************************************************

                         Ejemplo de i2c maestro y esclavo
 Usa i2c2 como maestro y i2c1 como esclavo.
 usa uart1 para ver la comunicación y rtc para temporización

 ? se escribe en uart1 para ver el contenido de 10 posiciones de memoria
 ¿ para escribir cada una de las 10 posiciones de memoria

 El esclavo maneja una memoria de 10 posiciones. La dirección de entrada apunta
 a las posiciones de la memoria.

**********************************************************************/
#include "stm32f10x_conf.h"

#define I2CMASTER_ADDR 25
#define I2CSLAVE_ADDR  36
#define I2C1_CLOCK_FRQ 100000

// estados de la comunicación con el esclavo
#define I2C1_MODE_WAITING 0
#define I2C1_MODE_SLAVE_ADR_WR  1
#define I2C1_MODE_ADR_BYTE  2
#define I2C1_MODE_DATA_BYTE_WR  3
#define I2C1_MODE_SLAVE_ADR_RD  4
#define I2C1_MODE_DATA_BYTE_RD  5

#define LONGITUD_MEMORIA 10

void Sysclk_72M(void);
void UART_Init(void);
void UART_mensaje(char* mensaje, uint8_t largo);

void estado(I2C_TypeDef* i2c_cual);
void I2C1_Esclavo_init(void);
void I2C1_ClearFlag(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C2_Maestro_init(void);
void WriteByte(uint8_t address, uint8_t data);
uint8_t mem_read(uint8_t reg_addr);
void RTCConfig(void);
void UART_numero(uint32_t numero);

uint8_t i2c1_mode;
uint8_t i2c1_ram_adr;
uint8_t memoria[LONGITUD_MEMORIA];


int main(void)
{

    Sysclk_72M();
    UART_Init();
    RTCConfig();

    I2C1_Esclavo_init();
    I2C2_Maestro_init();

    UART_mensaje("\r\n>>", 4);

    i2c1_mode = I2C1_MODE_WAITING;
    i2c1_ram_adr= 0;
    uint8_t i= 0;
    uint16_t leer_uart;

    // llena la memoria
    for(i= 0; i< LONGITUD_MEMORIA; i++)
    {
      memoria[i]= '-';
    }
    uint32_t reloj= RTC_GetCounter();
    uint8_t caracter;

  while(1)
  {
      // si entra un caracter ? o ¿
      if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)== SET)
      {
          leer_uart= USART_ReceiveData(USART1);
          // ? es para mostrar el contenido de la memoria
          if(leer_uart== '?')
          {
              UART_mensaje("?\r\n", 3);
              for(i= 0; i< LONGITUD_MEMORIA; i++)
              {
                  UART_mensaje("M(", 2);
                  UART_numero(i);
                  UART_mensaje(")= ", 3);
                  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
                  USART_SendData(USART1, mem_read(i));// aquí lee la memoria por medio de i2c como esclavo
                  UART_mensaje("\n", 1);
              }
              UART_mensaje("\r\n>>", 4);
          }

          // ¿ es para escribir en la memoria
          if(leer_uart== '¿')
          {
              UART_mensaje("¿\r\n", 3);
              i= 0;
              reloj= RTC_GetCounter(); // si al cabo de 10 segundos no hay entrada, retorna al comienzo
              while((i<LONGITUD_MEMORIA) && ((RTC_GetCounter()- reloj)< 10))
              {
                  if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)== SET)
                  {
                      caracter= (uint8_t) (USART_ReceiveData(USART1));
                      WriteByte(i, caracter); // aquí escribe en memoria por medio del i2c esclavo
                      UART_mensaje("M(", 2);
                      UART_numero(i);
                      UART_mensaje(")<- ", 4);
                      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
                      USART_SendData(USART1, caracter);
                      UART_mensaje("\n", 1);
                      i++;
                      reloj= RTC_GetCounter();
                  }
              }
              UART_mensaje("\r\n>>", 4);
          }
      }
  }
}

/***********************************************
 * Inicializacion del reloj Sysclk en 72 Mhz, igual en HCLK y PCLK2.
 * PCLK1 en la mitad
 ***********************************************/
void Sysclk_72M(void)
{
    // usa el reloj interno mientras se configura
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
    // deshabilita el pll para poder configurarlo
    RCC_PLLCmd(DISABLE);
    //8Mhz/2*14= 56Mhz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
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
    RCC_PCLK1Config( RCC_HCLK_Div2);
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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);

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

//============================================================

// I2C1 como  esclavo- usa interrupción
void I2C1_Esclavo_init(void)
{
    I2C_DeInit(I2C1);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    // Configura pines I2C2: SCL y SDA
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // configuración I2C
    I2C_InitTypeDef  I2C_InitStructure;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2CSLAVE_ADDR;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C1_CLOCK_FRQ;

    // habilita I2C1
    I2C_Cmd(I2C1, ENABLE);
    // aplica configuración I2C1
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_StretchClockCmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    // configuración de interrupción
    NVIC_InitTypeDef NVIC_InitStructure;
     NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //Configura interrupción en caso de error en I2C
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    // habilitación de banderas de interrupción
    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);

    return;
}

void I2C1_ClearFlag(void)
{
    // ADDR-Flag clear
    while((I2C1->SR1 & I2C_SR1_ADDR) == I2C_SR1_ADDR)
    {
        I2C1->SR1;
        I2C1->SR2;
    }

    // STOPF Flag clear
    while((I2C1->SR1&I2C_SR1_STOPF) == I2C_SR1_STOPF)
    {
        I2C1->SR1;
        I2C1->CR1 |= 0x1;
    }
    return;
}

// esta función para mostrar el estado de las banderas del i2c.
// i2c_cual puede ser I2C1 o I2C2.
void estado(I2C_TypeDef* i2c_cual)
{
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_TRA)== SET)
        UART_mensaje("Transmitter/Receiver flag\r\n",27);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_BUSY)== SET)
        UART_mensaje("Bus busy flag\r\n",15);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_MSL)== SET)
        UART_mensaje("Master/Slave flag\r\n",19);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_TIMEOUT)== SET)
        UART_mensaje("Timeout or Tlow error flag\r\n",28);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_PECERR)== SET)
        UART_mensaje("PEC error in reception flag\r\n",29);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_OVR)== SET)
        UART_mensaje("Overrun/Underrun flag (Slave mode)\r\n",36);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_AF)== SET)
        UART_mensaje("Acknowledge failure flag\r\n",26);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_ARLO)== SET)
        UART_mensaje("Arbitration lost flag (Master mode)\r\n",37);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_BERR)== SET)
        UART_mensaje("Bus error flag\r\n",16);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_TXE)== SET)
        UART_mensaje("Data register empty flag (Transmitter)\r\n",40);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_RXNE)== SET)
        UART_mensaje("Data register not empty (Receiver) flag\r\n",41);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_STOPF)== SET)
        UART_mensaje("Stop detection flag (Slave mode)\r\n",34);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_BTF)== SET)
        UART_mensaje("Byte transfer finished flag\r\n",29);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_ADDR)== SET)
        UART_mensaje("Address sent flag\r\n",19);
    if(I2C_GetFlagStatus(i2c_cual, I2C_FLAG_SB)== SET)
        UART_mensaje("Start bit flag (Master mode)\r\n",30);
}

// subrutina de tratamiento a interrupción por eventos del i2c1 como esclavo
void I2C1_EV_IRQHandler(void)
{
    uint32_t event;
    uint8_t wert;

    // lee el último evento
    event = I2C_GetLastEvent(I2C1);

    switch(event)
    {
        case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
        {
            // el maestro ha enviado la dirección del esclavo para envío de datos al esclavo
            //UART_mensaje("esclavo: I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED\r\n",51);
            i2c1_mode = I2C1_MODE_SLAVE_ADR_WR;
            break;
        }

        case I2C_EVENT_SLAVE_BYTE_RECEIVED:
        {
            // el maestro ha enviado un byte al esclavo
            if(I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF)== SET)
            {
                (void)(I2C1->SR1);
//                (void)(I2C1->SR2);
            }
            wert = I2C_ReceiveData(I2C1);

            // revisa dirección
            if(i2c1_mode == I2C1_MODE_SLAVE_ADR_WR)
            {
                i2c1_mode = I2C1_MODE_ADR_BYTE;
                // pone la dirección de memoria
                if(wert > 9) wert = 9;
                i2c1_ram_adr = wert;
            }
            else
            {
                i2c1_mode = I2C1_MODE_DATA_BYTE_WR;
                // guarda byte en memoria
                memoria[i2c1_ram_adr]= wert;
                //próxima posición
                i2c1_ram_adr++;
                if(i2c1_ram_adr > 9) i2c1_ram_adr= 9;
            }
            //UART_mensaje("esclavo: I2C_EVENT_SLAVE_BYTE_RECEIVED\r\n",40);
            break;
        }

        case I2C_EVENT_SLAVE_ACK_FAILURE:
        {
            //UART_mensaje("esclavo: I2C_EVENT_SLAVE_ACK_FAILURE\r\n",38);
            I2C1->SR1 &= 0x00FF;
            break;
        }

        case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
        {
            // el maestro ha enviado la dirección del esclavo para leer un byte del esclavo
            //UART_mensaje("esclavo: I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED\r\n",54);
            i2c1_mode = I2C1_MODE_SLAVE_ADR_RD;
            // lee un byte de memoria
            wert = memoria[i2c1_ram_adr];
            // envía byte al maestro
            I2C_SendData(I2C1, wert);
            //próxima posición
            i2c1_ram_adr++;
            if(i2c1_ram_adr > 9) i2c1_ram_adr= 9;
            break;
        }

        case I2C_EVENT_SLAVE_BYTE_TRANSMITTING:
        {
            // en caso el maestro vaya a leer otro byte del esclavo
            i2c1_mode = I2C1_MODE_DATA_BYTE_RD;
            // lee byte de memoria
            wert =  memoria[i2c1_ram_adr];
            // envía byte al maestro
            I2C_SendData(I2C1, wert);
            // próxima posición
            i2c1_ram_adr++;
            if(i2c1_ram_adr > 9) i2c1_ram_adr= 9;
            //UART_mensaje("esclavo: I2C_EVENT_SLAVE_BYTE_TRANSMITTED\r\n",43);
            break;
        }

        case I2C_EVENT_SLAVE_STOP_DETECTED:
        {
            //el maestro ha parado la comunicación
            //UART_mensaje("esclavo: I2C_EVENT_SLAVE_STOP_DETECTED\r\n",40);
            I2C1_ClearFlag();
            i2c1_mode = I2C1_MODE_WAITING;
            break;
        }

        default:
        {
            break;
        }
    }
}

// función para error
void I2C1_ER_IRQHandler(void)
{
    if (I2C_GetITStatus(I2C1, I2C_IT_AF))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
    }
}


// I2C2 como maestro
void I2C2_Maestro_init(void)
{
    I2C_DeInit(I2C2);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    // Configura pines I2C2: SCL y SDA
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // configuracion I2C2 como maestro.
    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2CMASTER_ADDR;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C1_CLOCK_FRQ;

    // habilita I2C2
    I2C_Cmd(I2C2, ENABLE);
    // aplica configuración I2C2
    I2C_Init(I2C2, &I2C_InitStructure);
}

// manda un dato data a la dirección address por i2c como maestro
void WriteByte(uint8_t address, uint8_t data)
{
  I2C_GenerateSTART(I2C2,ENABLE);
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C2, I2CSLAVE_ADDR, I2C_Direction_Transmitter);
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2C2,address);
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_SendData(I2C2,data);
  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTOP(I2C2,ENABLE);

  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  return;
}

// lee un byte de memoria en la posición reg_addr
uint8_t mem_read(uint8_t reg_addr)
{
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) == SET); // mientras esté ocupado

    I2C_GenerateSTART(I2C2, ENABLE); // envía condición de inicio
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    //UART_mensaje("maestro: I2C_EVENT_MASTER_MODE_SELECT\r\n",39);
    I2C_Send7bitAddress(I2C2, I2CSLAVE_ADDR, I2C_Direction_Transmitter); // envía dirección esclavo para escribir

    while(!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    //UART_mensaje("maestro: I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED\r\n",53);
    I2C_SendData(I2C2, reg_addr); // envía dirección registro interno

    while(!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    //UART_mensaje("maestro: I2C_EVENT_MASTER_BYTE_TRANSMITTED\r\n",44);
    I2C_GenerateSTART(I2C2, ENABLE); // repite condición de inicio

    while(!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    //UART_mensaje("maestro: I2C_EVENT_MASTER_MODE_SELECT\r\n",39);
    I2C_Send7bitAddress(I2C2, I2CSLAVE_ADDR, I2C_Direction_Receiver); // envía dirección esclavo para lectura
    //UART_mensaje("maestro: \r\n",11);
    //estado(I2C2);
    I2C_AcknowledgeConfig (I2C2, DISABLE);

    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    //UART_mensaje("maestro: I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED\r\n",50);

    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    (void)(I2C2->SR1);
    uint8_t read_reg = I2C_ReceiveData (I2C2);
    I2C_NACKPositionConfig(I2C2, I2C_NACKPosition_Current);
    for(uint32_t t= 0; t<100000; t++);
    I2C_GenerateSTOP (I2C2, ENABLE);
    //UART_mensaje("maestro: I2C_EVENT_MASTER_BYTE_RECEIVED\r\n",41);

    return read_reg;
}

//============================================================
void RTCConfig(void)
{
    // habilitacion del reloj
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    // habilita acceso al rtc
    PWR_BackupAccessCmd(ENABLE);
    // habilita el reloj externo
    RCC_LSEConfig(RCC_LSE_ON);
    // espera hasta que el reloj se estabilice
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
    // selecciona el reloj externo como fuente
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    // habilita el rtc
    RCC_RTCCLKCmd(ENABLE);
    // espera por sincronizacion
    RTC_WaitForSynchro();
    while(RTC_GetFlagStatus(RTC_FLAG_RTOFF)== RESET);
    RTC_SetPrescaler(32768); // divisor del reloj externo
    RTC_SetCounter(0);

    return;
}

// convierte un número en caracteres ascii
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
//    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
//    USART_SendData(USART1, ' ');
    return;
}
