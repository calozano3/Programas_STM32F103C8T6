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
   Báscula electrónica de 20kgrs con display de 4 digitos TM1637.
   Con reset pone a ceros la medida
** Usa un display TM1637 de cuatro digitos, 7 segmentos con :
    Micro    TM1637
    B0  ->    CLK
    B1  ->    DIO
    +3.3v ->  VCC
    GND   ->  GND
**********************************************************************/
#include "stm32f10x_conf.h"

void Sysclk_56M(void);
void UART_Init(void);
void UART_numero(uint32_t numero, uint8_t decimal);
void UART_mensaje(char *dato, uint8_t tamano);
void HX711_init(void);
void HX711_SCK_0(void);
void HX711_SCK_1(void);
uint8_t Leer_HX711_DT(void);
uint32_t HX711_lectura(void);
uint32_t HX711_promedio(void);

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

int main(void)
{
    Sysclk_56M();
    UART_Init();
    HX711_init();
    TM1637Display();

    borrar();// apaga los segmentos del display

    uint32_t lectura= 0;
    uint32_t inicio= HX711_promedio();
    inicio= HX711_promedio(); // lecturas de 10 muestras promedio

    setBrightness(7, 1);

  while(1)
  {
      lectura= HX711_promedio();

      if(inicio <= lectura)
      {
          lectura= lectura - inicio;// con reset, pone a ceros la medida
          // la ecuación es de 5/17. Para calibrar: 20/67, 40/135. Multiplica igual valor arriba y abajo
          // y resta o aumenta en la parte de abajo
          lectura= lectura*20;// 40, 20
          lectura= (uint32_t) (lectura/ 67);// 136-1, 68-1
          lectura= (uint32_t) (lectura/ 10000);
          UART_numero(lectura, 20);// envía medida por uart
          showNumberDecEx((uint16_t) lectura, 0, 0, 4, 0);
      }
      else
      {
          showNumberDecEx(0, 0, 0, 4, 0);
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

///----------------------------------------------------------------
///FUNCTION NAME: HK711_init
///FUNCTION     : configuración de pines SCK y DT del módulo HK711
///INPUT        :nada
///OUTPUT       :nada
///
///                 STM32       HK711
///                 GND----------GND   (ground in)
///                 3V3----------3.3V  (3.3V in)
///                 A0-----------SCK
///                 A1-----------DT
///----------------------------------------------------------------
void HX711_init(void)
{
    /** conexión con el módulo HK711
    A0     ------> HK711_SCK
    A1     ------> HK711_DT
    */

    GPIO_InitTypeDef GPIO_Struct;

    //configuración de los pines sck y dt
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // GPIOA PIN0 (SCK) como salida push- pull
    GPIO_Struct.GPIO_Pin = GPIO_Pin_0;
    GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_Struct);

    //GPIOA PIN1 (DT) como entrada flotante
    GPIO_Struct.GPIO_Pin = GPIO_Pin_1;
    GPIO_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_Struct);

    // configuracion de interrupcion del pin GDO0
//    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);// GDO0;
//    EXTI_InitTypeDef EXTI_InitStruct;
//    EXTI_InitStruct.EXTI_Line = EXTI_Line4;
//    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
//    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
//    EXTI_Init(&EXTI_InitStruct);
//
//    NVIC_InitTypeDef NVIC_Struct;
//    NVIC_Struct.NVIC_IRQChannel = EXTI4_IRQn;
//    NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
//    NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_Struct);

    return;

}

//---------------------------------------------------------------------------//
//FUNCTION NAME: HK711_SCK_0
//FUNCTION     : Pone el bit HK711_SCK en 0
//INPUT        : none
//OUTPUT       : none
//---------------------------------------------------------------------------//
void HX711_SCK_0(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    return;
}

//---------------------------------------------------------------------------//
//FUNCTION NAME: HK711_SCK_1
//FUNCTION     : Pone el bit HK711_SCK en 1
//INPUT        : none
//OUTPUT       : none
//---------------------------------------------------------------------------//
void HX711_SCK_1(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_0);
    return;
}

//---------------------------------------------------------------------------//
//FUNCTION NAME: Leer_HK711_DT
//FUNCTION     : lee en bit DT del módulo HK711
//INPUT        : none
//OUTPUT       : 1 o 0
//---------------------------------------------------------------------------//
uint8_t Leer_HX711_DT(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
}

uint32_t HX711_lectura(void)
{
    uint32_t Count;
    uint8_t i;

    HX711_SCK_0();
    Count = 0;
    while(Leer_HX711_DT());
    for (i=0;i<24;i++)
    {
        HX711_SCK_1();
        Count = Count << 1;
        HX711_SCK_0();
        if(Leer_HX711_DT()) Count++;
    }
    HX711_SCK_1();
    //Count = Count ^ 0x800000;
    Count = Count << (32 - i);
    HX711_SCK_0();
    return(Count);
}

uint32_t HX711_promedio(void)
{
    uint8_t veces;
    uint32_t promedio= 0;
    for(veces= 0; veces< 10; veces++)
    {
        promedio= promedio + (uint32_t) (HX711_lectura()/10);
    }
    return promedio;
}

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
// The third and fourth argument are the same as in the previous function.
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



