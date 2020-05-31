/*
**
**                           Main.c
** ejemplo de comunicación i2c para medir la temperatura con el módulo MLX90614
** usa el display TM1637

   Modulo           STM32F103C8T6
     VIN                +3.3v
     GND                 GND
     SCL                 PB10
     SDA                 PB11
**
**********************************************************************/
#include "stm32f10x_conf.h"

#define MLX90614_I2CADDR 1

// RAM
#define MLX90614_RAWIR1 0x04
#define MLX90614_RAWIR2 0x05
#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MLX90614_TOBJ2 0x08
// EEPROM
#define MLX90614_TOMAX 0x20
#define MLX90614_TOMIN 0x21
#define MLX90614_PWMCTRL 0x22
#define MLX90614_TARANGE 0x23
#define MLX90614_EMISS 0x24
#define MLX90614_CONFIG 0x25
#define MLX90614_ADDR 0x2E
#define MLX90614_ID1 0x3C
#define MLX90614_ID2 0x3D
#define MLX90614_ID3 0x3E
#define MLX90614_ID4 0x3F

//==============================================
#define MLX90614_REGISTER_TA      0x06
#define MLX90614_REGISTER_TOBJ1	  0x07
#define MLX90614_REGISTER_TOBJ2	  0x08
#define MLX90614_REGISTER_TOMAX   0x20
#define MLX90614_REGISTER_TOMIN   0x21
#define MLX90614_REGISTER_PWMCTRL 0x22
#define MLX90614_REGISTER_TARANGE 0x23
#define MLX90614_REGISTER_KE      0x24
#define MLX90614_REGISTER_CONFIG  0x25
#define MLX90614_REGISTER_ADDRESS 0x2E
#define MLX90614_REGISTER_ID0     0x3C
#define MLX90614_REGISTER_ID1     0x3D
#define MLX90614_REGISTER_ID2     0x3E
#define MLX90614_REGISTER_ID3     0x3F
#define MLX90614_REGISTER_SLEEP   0xFF // Not really a register, but close enough

#define I2C_READ_TIMEOUT 1000
//==============================================

#define SMBus_NAME             I2C2
#define SMBus_RCC_Periph       RCC_APB1Periph_I2C2
#define SMBus_Port             GPIOB
#define SMBus_SCL_Pin          GPIO_Pin_10
#define SMBus_SDA_Pin          GPIO_Pin_11
#define SMBus_SCL_PinSource    GPIO_PinSource10
#define SMBus_SDA_PinSource    GPIO_PinSource11
#define SMBus_RCC_Port         RCC_APB2Periph_GPIOB
#define SMBus_Speed            11000
#define SMBus_GPIO_AF          GPIO_AF_I2C2
#define SMBus_Max_Delay_Cycles 10000

void Sysclk_56M(void);
void UART_Init(void);
void UART_numero(uint32_t numero, uint8_t decimal);
void UART_mensaje(char *dato, uint8_t tamano);
void RTCConfig(void);

void delayMicroseconds(uint16_t retardo);
void checkaddress(void);

uint16_t mma_read(uint8_t dev_addr, uint8_t reg_addr);
uint8_t dir_esclavo(void);
uint8_t crc8_calc(uint8_t *addr, uint8_t len);
uint8_t IRTherm_crc8(uint8_t inCrc, uint8_t inData);
void writeEEPROM(uint8_t dev_addr, uint8_t reg_addr, uint16_t reg_data);
void I2C_WordWrite(uint8_t dev_addr, uint8_t reg_addr, uint16_t reg_data);
void timer1_init(void);
void I2c2_config(void);

//============para el display TM1637===================
#define TM1637_I2C_COMM1    0x40
#define TM1637_I2C_COMM2    0xC0
#define TM1637_I2C_COMM3    0x80

///
///      A
///     ---
///  F |   | B
///     -G-
///  E |   | C
///     ---
///      D
const uint8_t digitToSegment[] = {
 // XGFEDCBA
  0b00111111,    // 0
  0b00000110,    // 1
  0b01011011,    // 2
  0b01001111,    // 3
  0b01100110,    // 4
  0b01101101,    // 5
  0b01111101,    // 6
  0b00000111,    // 7
  0b01111111,    // 8
  0b01101111,    // 9
  0b01110111,    // A
  0b01111100,    // b
  0b00111001,    // C
  0b01011110,    // d
  0b01111001,    // E
  0b01110001     // F
  };

static const uint8_t minusSegments = 0b01000000;

#define SEG_A   0b00000001
#define SEG_B   0b00000010
#define SEG_C   0b00000100
#define SEG_D   0b00001000
#define SEG_E   0b00010000
#define SEG_F   0b00100000
#define SEG_G   0b01000000
#define SEG_DP  0b10000000

#define DEFAULT_BIT_DELAY  100
uint8_t m_brightness;

// usa dos pines del puerto B para la comunicación con el display.
#define m_pinClk GPIO_Pin_0
#define m_pinDIO GPIO_Pin_1

// Create degree Celsius symbol:
const uint8_t celsius[] = {
  SEG_A | SEG_B | SEG_F | SEG_G,  // Circle
  SEG_A | SEG_D | SEG_E | SEG_F   // C
};

void TM1637Display(void);
void setBrightness(uint8_t brightness, uint8_t on);
void setSegments(const uint8_t segments[], uint8_t length, uint8_t pos);
void borrar(void);
void showNumberDec(int num, uint8_t leading_zero, uint8_t length, uint8_t pos);
void showNumberDecEx(int num, uint8_t dots, uint8_t leading_zero, uint8_t length, uint8_t pos);
void showNumberHexEx(uint16_t num, uint8_t dots, uint8_t leading_zero, uint8_t length, uint8_t pos);
void showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots, uint8_t leading_zero, uint8_t length, uint8_t pos);
void bitDelay();
void start();
void stop();
uint8_t writeByte(uint8_t b);
void showDots(uint8_t dots, uint8_t* digits);
uint8_t encodeDigit(uint8_t digit);
void salida(uint16_t pin);
void entrada(uint16_t pin);
uint16_t convtiempo(uint16_t valor, uint8_t horamin);
//=============Fin valores display TM1637==============


int main(void)
{
    uint8_t direccion_esclavo;
    uint16_t dato= 0;
    uint16_t valor;

    Sysclk_56M();
    UART_Init();
    timer1_init();
    RTCConfig();
    I2c2_config();
    TM1637Display();// configuración display

    uint32_t reloj= RTC_GetCounter();
    UART_mensaje("Inicia...\n", 10);

    direccion_esclavo= dir_esclavo();
    direccion_esclavo= 0;
    // la dirección que encontró es 0x3B

//    UART_mensaje("Buscando acelerometro..\n", 24);
//
//    UART_mensaje("El registro de control 1 es: ", 29);
//    UART_numero(direccion_esclavo, 10);
//    UART_mensaje("\n", 1);
//    dato = mma_read(direccion_esclavo, MLX90614_REGISTER_ADDRESS);
//    dato &= 0xFF00; // Mask out the address (MSB is junk?)
//    dato |= 0x2A;
//    writeEEPROM(direccion_esclavo, MLX90614_REGISTER_ADDRESS, dato);
//
//    dato = mma_read(direccion_esclavo, MLX90614_TOMIN);
//    UART_mensaje("Tomn= ",6);
//    if(dato>27315)
//    {
//        dato= dato- 27315;
//    }
//    else
//    {
//        UART_mensaje("-",1);
//        dato= 27315- dato;
//    }
//    UART_numero(dato, 2);
//    UART_mensaje(" \r\n",3);

//    writeEEPROM(direccion_esclavo, MLX90614_EMISS, 42598);// cambio de emisividad a 0.65
//    writeEEPROM(direccion_esclavo, MLX90614_TOMIN, 27315);// cambio de Tomin a 0°C
    ///================================================================================
    // inicialización display TM1637
    setBrightness(7, 1);

    while(1)
    {
        if((RTC_GetCounter()- reloj)>= 1)
        {
//            dato = mma_read(direccion_esclavo, MLX90614_TA);
//            UART_mensaje("Temp= ",6);
//            UART_numero((dato*2-27315), 2);
//            UART_mensaje(" \r\n",3);
            dato = mma_read(direccion_esclavo, MLX90614_TOBJ1);
            valor= dato*2-27315;
            showNumberDec(valor, 0, 4, 0);
            UART_mensaje("Obj1= ",6);
            UART_numero((dato*2-27315), 2);

            UART_mensaje(" \r\n",3);
//            dato = mma_read(direccion_esclavo, MLX90614_TOMIN);
//            UART_mensaje("Tomn= ",6);
//            if(dato>=27315)
//            {
//                dato= dato- 27315;
//            }
//            else
//            {
//                UART_mensaje("-",1);
//                dato= 27315- dato;
//            }
//            UART_numero(dato, 2);
//            UART_mensaje(" \r\n",3);
//            dato = mma_read(direccion_esclavo, MLX90614_TARANGE);
//            UART_mensaje("Tran= ",6);
//            UART_numero(dato, 10);
//            UART_mensaje(" \r\n",3);
//            dato = mma_read(direccion_esclavo, MLX90614_EMISS);
//            UART_mensaje("Emis= ",6);
//            UART_numero(dato, 10);
//            UART_mensaje(" \r\n",3);

            reloj= RTC_GetCounter();
        }
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
void UART_numero(uint32_t numero, uint8_t decimal)
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
    if(decimal== 0)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, '0');
    }

    for(uint8_t i= 0; i<contador; i++)
    {
        if(i== decimal)
        {
            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
            USART_SendData(USART1, '.');
        }
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, tosend[contador- i- 1]);
    }
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, ' ');
    return;
}

void UART_mensaje(char *dato, uint8_t tamano)
{
    for(uint8_t apuntador= 0; apuntador< tamano; apuntador++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, dato[apuntador]);
    }

    return;
}

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


//======================================================================================

uint8_t crc8_calc(uint8_t *addr, uint8_t len)
// The PEC calculation includes all bits except the START, REPEATED START, STOP,
// ACK, and NACK bits. The PEC is a CRC-8 with polynomial X8+X2+X1+1.
{
  uint8_t crc = 0;
  while (len--) {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--) {
      uint8_t carry = (crc ^ inbyte) & 0x80;
      crc <<= 1;
      if (carry)
        crc ^= 0x7;
      inbyte <<= 1;
    }
  }
  return crc;
}

uint8_t IRTherm_crc8 (uint8_t inCrc, uint8_t inData)
{
	uint8_t i;
	uint8_t data;
	data = inCrc ^ inData;
	for ( i = 0; i < 8; i++ )
	{
		if (( data & 0x80 ) != 0 )
		{
			data <<= 1;
			data ^= 0x07;
		}
		else
		{
			data <<= 1;
		}
	}
	return data;
}


void delayMicroseconds(uint16_t retardo)
{
    uint16_t temporizador;
    TIM_SetCounter(TIM1, 0);

    temporizador= TIM_GetCounter(TIM1);
    while((TIM_GetCounter(TIM1)- temporizador)< retardo);
    return;
}

void checkaddress(void)
{
    uint8_t addr, direccion;
    uint8_t indicador;
    uint8_t valores[]={0, 1, 2, 88, 89, 90, 91, 92};
    uint32_t counter = SMBus_Max_Delay_Cycles;

    for (addr = 0; addr < 8; addr++)
    {
        direccion= valores[addr];
        UART_mensaje("\nDevice probed ", 15);
        UART_numero(direccion, 10);
        indicador= 0;
        counter = SMBus_Max_Delay_Cycles;

        I2C_AcknowledgeConfig(SMBus_NAME, ENABLE);
        while(I2C_GetFlagStatus(SMBus_NAME, I2C_FLAG_BUSY) && counter) --counter;
        if(counter == 0)
        {
            indicador++;
        }

        I2C_GenerateSTART(SMBus_NAME, ENABLE);
        counter = SMBus_Max_Delay_Cycles;
        while(!I2C_CheckEvent(SMBus_NAME, I2C_EVENT_MASTER_MODE_SELECT) && counter) --counter;
        if(counter == 0) {
            indicador++;
        }

        I2C_Send7bitAddress(SMBus_NAME, direccion, I2C_Direction_Transmitter);
        counter = SMBus_Max_Delay_Cycles;
        while(!I2C_CheckEvent(SMBus_NAME, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && counter) --counter;
        if(counter == 0) {
            indicador++;
        }
        if (indicador == 0)
        {
            UART_mensaje(": Device found", 14);
            //UART_numero(addr, 10);
		}
		I2C_GenerateSTOP(SMBus_NAME, ENABLE);

		delayMicroseconds(60000);
		delayMicroseconds(60000);
		delayMicroseconds(60000);
		delayMicroseconds(60000);
		delayMicroseconds(60000);
		delayMicroseconds(60000);
		delayMicroseconds(60000);
		delayMicroseconds(60000);
		delayMicroseconds(60000);
		delayMicroseconds(60000);
	}
	UART_mensaje("\nFinished\n", 10);
}


/***********************************************
 * lee un dato en la dirección reg_addr del esclavo dev_addr
 *
 ***********************************************/
uint16_t mma_read(uint8_t dev_addr, uint8_t reg_addr)
{
    uint8_t pec;
    uint8_t pecbuf[5];

    pecbuf[0] = dev_addr << 1;
    pecbuf[1] = reg_addr;

    I2C_AcknowledgeConfig(I2C2, ENABLE);
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)); // mientras esté ocupado

    I2C_GenerateSTART(I2C2, ENABLE); // envía condición de inicio
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
//    UART_mensaje("maestro: I2C_EVENT_MASTER_MODE_SELECT\r\n",39);

    I2C_Send7bitAddress(I2C2, dev_addr, I2C_Direction_Transmitter); // envía dirección esclavo para escribir
    while(!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//    UART_mensaje("maestro: I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED\r\n",53);

    I2C_SendData(I2C2, reg_addr); // envía dirección registro interno
    while(!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//    UART_mensaje("maestro: I2C_EVENT_MASTER_BYTE_TRANSMITTED\r\n",44);

    I2C_GenerateSTART(I2C2, ENABLE); // repite condición de inicio
    while(!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_MODE_SELECT));
//    UART_mensaje("maestro: I2C_EVENT_MASTER_MODE_SELECT\r\n",39);

    I2C_Send7bitAddress(I2C2, dev_addr, I2C_Direction_Receiver); // envía dirección esclavo para lectura
    while(!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
//    UART_mensaje("maestro: I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED\r\n",50);
    while(!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
//    UART_mensaje("maestro: I2C_EVENT_MASTER_BYTE_RECEIVED\r\n",41);

    uint8_t lsb_value = I2C_ReceiveData (I2C2);
//    UART_mensaje("\r\nlsb= ",7);
//    UART_numero(lsb_value, 10);
//    UART_mensaje(" \r\n",3);
    while(!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    uint8_t msb_value= I2C_ReceiveData (I2C2);
//    UART_mensaje("msb= ",5);
//    UART_numero(msb_value, 10);
//    UART_mensaje(" \r\n",3);
    uint32_t read_reg = msb_value*256 + lsb_value;

    I2C_AcknowledgeConfig (I2C2, DISABLE);
    while(!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    uint8_t crc_value = I2C_ReceiveData (I2C2);

    pecbuf[2] = pecbuf[0]+1;
    pecbuf[3] = lsb_value;
    pecbuf[4] = msb_value;
    pec = crc8_calc(pecbuf, 5);

    I2C_GenerateSTOP (I2C2, ENABLE);
    if(pec== crc_value)
    {
        return msb_value*256 + lsb_value;
    }

    return 0;
}

/***********************************************
 * busca la dirección del esclavo
 * retorna 0xFF si no la encuentra
 ***********************************************/
uint8_t dir_esclavo(void)
{
    uint8_t direccion, salida;
    uint16_t contador;

    salida= 0xff;

    I2C_AcknowledgeConfig(I2C2, ENABLE);
    for(direccion=0; direccion< 127; direccion++)
    {
        while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)); // mientras esté ocupado

        I2C_GenerateSTART(I2C2, ENABLE); // envía condición de inicio
        while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
//        UART_mensaje("maestro: I2C_EVENT_MASTER_MODE_SELECT\r\n",39);

        contador= 0;
        I2C_Send7bitAddress(I2C2, direccion, I2C_Direction_Transmitter); // envía dirección esclavo para escribir
        while((contador< 50000) && (!I2C_CheckEvent (I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)))
        {
            contador++;
        }
        if(contador< 50000)
        {
            // hubo respuesta
            //UART_mensaje("maestro: I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED\r\n",53);
            UART_mensaje("dirección: ", 11);
            UART_numero(direccion, 10);
            UART_mensaje("\n", 1);
            salida= direccion;
        }

        I2C_GenerateSTOP (I2C2, ENABLE);
        //I2C_AcknowledgeConfig (I2C2, DISABLE);
    }

    return salida;
}

void writeEEPROM(uint8_t dev_addr, uint8_t reg_addr, uint16_t reg_data)
{
	// Clear out EEPROM first:
	//mma_write(dev_addr, reg_addr, 0);
	I2C_WordWrite(dev_addr, reg_addr, 0);

	delayMicroseconds(5000); // Delay tErase

	//mma_write(dev_addr, reg_addr, reg_data);
	I2C_WordWrite(dev_addr, reg_addr, reg_data);
	delayMicroseconds(5000);

	return;
}


void I2C_WordWrite(uint8_t dev_addr, uint8_t reg_addr, uint16_t reg_data)
{
  uint8_t pec;

	pec = IRTherm_crc8(0, (dev_addr << 1));
	pec = IRTherm_crc8(pec, reg_addr);
	pec = IRTherm_crc8(pec, (reg_data & 0x00FF));
	pec = IRTherm_crc8(pec, (reg_data >> 8));

  /* Send STRAT condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C2, dev_addr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2C2, reg_addr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(I2C2, (reg_data & 0xFF));

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(I2C2, (reg_data>>8));

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(I2C2, pec);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(I2C2, ENABLE);
}

void timer1_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    // se escoge el reloj interno, el que sale de APB2, o sea, 56 Mhz, como entrada al contador Tim1
    TIM_InternalClockConfig(TIM1);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct; // creacion de estructura
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct); // inicialización de datos de la estructura
	// son 56M de entrada, dividido en 56000, queda 1000
	TIM_TimeBaseInitStruct.TIM_Prescaler = 56; // pulsos de 1us
	TIM_TimeBaseInitStruct.TIM_Period = 65535; // cuenta hasta 65536 pulsos
	//TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // conteo ascendente.
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct); // configuración con datos
	TIM_Cmd(TIM1, ENABLE);// habilitación
}

void I2c2_config(void)
{
    I2C_DeInit(I2C2);
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    // Configura pines I2C2: SCL y SDA
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // configuracion I2C2 como maestro.
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 20000;

    // habilita I2C2
    I2C_Cmd(I2C2, ENABLE);
    // aplica configuración I2C1
    I2C_Init(I2C2, &I2C_InitStructure);
}
//====================Funciones para el display TM1637==================================
// configura los pines pinClk y pinDIO del puerto B para comunicación con el display
void TM1637Display(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct; // estrutura para configurar los pines
    // configuración de los pines como entradas digitales
    GPIO_StructInit(&GPIO_InitStruct); // inicialización de la estructura
    GPIO_InitStruct.GPIO_Pin = m_pinClk | m_pinDIO;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_ResetBits(GPIOB, m_pinClk | m_pinDIO);

	// Set the pin direction and default value.
	// Both pins are set as inputs, allowing the pull-up resistors to pull them up
	return;
}

// This function sets the brightness of the display (as the name suggests).
// You can specify a brightness level from 0 (lowest brightness) to 7 (highest brightness).
// The second parameter can be used to turn the display on or off, false means off.
void setBrightness(uint8_t brightness, uint8_t on)
{
	m_brightness = (brightness & 0x7) | (on? 0x08 : 0x00);
	return;
}

// This function can be used to set the individual segments of the display.
// The first argument is the array that includes the segment information.
// The second argument specifies the number of digits to be modified (0-4).
// The third argument sets the position from which to print (0 – leftmost, 3 – rightmost).
void setSegments(const uint8_t segments[], uint8_t length, uint8_t pos)
{
    // Write COMM1
	start();
	writeByte(TM1637_I2C_COMM1);
	stop();

	// Write COMM2 + first digit address
	start();
	writeByte(TM1637_I2C_COMM2 + (pos & 0x03));

	// Write the data bytes
	for (uint8_t k=0; k < length; k++)
	  writeByte(segments[k]);

	stop();

	// Write COMM3 + brightness
	start();
	writeByte(TM1637_I2C_COMM3 + (m_brightness & 0x0f));
	stop();
	return;
}

void borrar(void)
{
    uint8_t data[] = { 0, 0, 0, 0 };
	setSegments(data, 4, 0);
	return;
}

// This is probably the function that you will use the most.
// The first argument is a number that you want to display on the screen.
// The second argument can be used to turn on or off leading zeros.
// 10 without leading zeros would print as __10 and with leading zeros as 0010.
// You can turn them on by setting this argument as true or turn them off by setting it as false.
// NOTE: leading zero is not supported with negative numbers.
// The third argument specifies the number of digits to be modified (0-4).
// The last argument sets the position from which to print (0 – leftmost, 3 – rightmost).
// Print the number 12 without leading zeros on the second and third digit:
// showNumberDec(12, false, 2, 1);
void showNumberDec(int num, uint8_t leading_zero, uint8_t length, uint8_t pos)
{
    showNumberDecEx(num, 0, leading_zero, length, pos);
    return;
}

// This function allows you to control the dots of the display.
// Only the second argument is different from the showNumberDec function.
// It allows you to set the dots between the individual digits.
// You can use the following values.
// For displays with dots between each digit:
// 0b10000000 – 0.000
// 0b01000000 – 00.00
// 0b00100000 – 000.0
// 0b11100000 – 0.0.0.0
// For displays with just a colon:
// 0b01000000 – 00:00
// For displays with dots and colons colon:
// 0b11100000 – 0.0:0.0
// So if you want to display a clock with center colon on, you would use something like:
// Print 1234 with the center colon:
// showNumberDecEx(1234, 0b11100000, false, 4, 0);
void showNumberDecEx(int num, uint8_t dots, uint8_t leading_zero, uint8_t length, uint8_t pos)
{
    showNumberBaseEx(num < 0? -10 : 10, num < 0? -num : num, dots, leading_zero, length, pos);
    return;
}

void showNumberHexEx(uint16_t num, uint8_t dots, uint8_t leading_zero, uint8_t length, uint8_t pos)
{
    showNumberBaseEx(16, num, dots, leading_zero, length, pos);
    return;
}

void showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots, uint8_t leading_zero, uint8_t length, uint8_t pos)
{
    uint8_t negative = 0;
	if (base < 0) {
	    base = -base;
		negative = 1;
	}


    uint8_t digits[4];

	if ((num == 0) && (!leading_zero)) {
		// Singular case - take care separately
		for(uint8_t i = 0; i < (length-1); i++)
			digits[i] = 0;
		digits[length-1] = encodeDigit(0);
	}
	else {
		//uint8_t i = length-1;
		//if (negative) {
		//	// Negative number, show the minus sign
		//    digits[i] = minusSegments;
		//	i--;
		//}

		for(int i = length-1; i >= 0; --i)
		{
		    uint8_t digit = num % base;

			if ((digit == 0) && (num == 0) && (leading_zero == 0))
			    // Leading zero is blank
				digits[i] = 0;
			else
			    digits[i] = encodeDigit(digit);

			if ((digit == 0) && (num == 0) && negative) {
			    digits[i] = minusSegments;
				negative = 0;
			}

			num /= base;
		}

		if(dots != 0)
		{
			showDots(dots, digits);
		}
    }
    setSegments(digits, length, pos);
    return;
}

void bitDelay()
{
    for(uint32_t tt= 0; tt< 5000; tt++) asm("nop");
    return;
}

void start()
{
    salida(m_pinDIO);
    bitDelay();
    return;
}

void stop()
{
	salida(m_pinDIO);
	bitDelay();
	entrada(m_pinClk);
	bitDelay();
	entrada(m_pinDIO);
	bitDelay();
	return;
}

uint8_t writeByte(uint8_t b)
{
  uint8_t data = b;

  // 8 Data Bits
  for(uint8_t i = 0; i < 8; i++) {
    // CLK low
    salida(m_pinClk);
    bitDelay();

	// Set data bit
    if (data & 0x01)
        entrada(m_pinDIO);
    else
        salida(m_pinDIO);

    bitDelay();

	// CLK high
    entrada(m_pinClk);
    bitDelay();
    data = data >> 1;
  }

  // Wait for acknowledge
  // CLK to zero
  salida(m_pinClk);
  entrada(m_pinDIO);
  bitDelay();

  // CLK to high
  entrada(m_pinClk);
  bitDelay();
  uint8_t ack = GPIO_ReadInputDataBit(GPIOB, m_pinDIO);
  if (ack == 0)
      salida(m_pinDIO);


  bitDelay();
  salida(m_pinClk);
  bitDelay();

  return ack;
}

void showDots(uint8_t dots, uint8_t* digits)
{
    for(int i = 0; i < 4; ++i)
    {
        digits[i] |= (dots & 0x80);
        dots <<= 1;
    }
}

uint8_t encodeDigit(uint8_t digit)
{
	return digitToSegment[digit & 0x0f];
}

void salida(uint16_t pin)
{
    // configuración del pin como salida digital
    GPIO_InitTypeDef GPIO_InitStruct; // estrutura para configurar los pines
    GPIO_StructInit(&GPIO_InitStruct); // inicialización de la estructura
    GPIO_InitStruct.GPIO_Pin = pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    return;
}

void entrada(uint16_t pin)
{
    // configuración del pin como entrada digital
    GPIO_InitTypeDef GPIO_InitStruct; // estrutura para configurar los pines
    GPIO_StructInit(&GPIO_InitStruct); // inicialización de la estructura
    GPIO_InitStruct.GPIO_Pin = pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    return;
}

// convierte a horas, minutos y segundos.
// en formato HH:MM si horamin es igual a 1
// o en formato MM:SS si horamin es igual a 0
// entrada valor está en segundos.
// salida está en HHMM o en MMSS
// valor no debe ser mayor a 43200, o sea, 12 horas.
uint16_t convtiempo(uint16_t valor, uint8_t horamin)
{
    uint8_t horas, minutos, segundos;
    uint16_t temporal, alterno;

    if(valor> 43300) return 0;
    temporal= valor;

	alterno = temporal/60;
	segundos = temporal - alterno*60;

	temporal = alterno;
	alterno = temporal/60;
	minutos = temporal - alterno*60;

	temporal = alterno;
	alterno = temporal/24;
	horas = temporal - alterno*24;

	if(horamin)
    {
        return (uint16_t) (horas*100+ minutos);
    }

    return (uint16_t) (minutos* 100+ segundos);
}
//======================fin de funciones para el display TM1637=========================

