/*
**
**                           Main.c
** programa que usa el comando inline assembler para ejecutar instrucciones
** en lenguaje ensamblador. Ejemplo de como encender y apagar el led de la tarjeta.
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f10x_conf.h"

int main(void)
{
    //configuración del reloj del puerto
    asm("MOVW r0, #0x1000");
    asm("MOVT r0, #0x4002"); //dirección base RCC= 0x4002.1000
    asm("MOVW r2, #0x18"); //offset de RCC_APB2ENR
    asm("LDR r1, [r0, r2]"); //lee su contenido
    asm("ORR r1, r1, #0x08"); //bandera de habilitación del puerto B
    asm("STR r1, [r0, r2]");

    //configuración de los bits como salida
    asm("MOVW r0, #0x0C00"); // base puerto B
    asm("MOVT r0, #0x4001"); //dirección base 0x4001.0C00
    asm("MOVW r2, #0x04"); //offset del registro GPIOB_CRH
    asm("MOVW r1, #0x0000");
    asm("MOVT r1, #0x0001");
    asm("STR r1, [r0, r2]");

    // encender led
    asm("MOVW r2, #0x10"); //offset del registro GPIOB_BSRR
    asm("MOVT r1, #0x1000");
    asm("STR r1, [r0, r2]");

  while(1)
  {
    for (uint32_t i = 0; i < 2000000; ++i) asm("nop");
      // apagar led
      asm("MOVW r0, #0x0C00"); // base puerto B
      asm("MOVT r0, #0x4001");
      asm("MOVW r2, #0x10");
      asm("MOVT r1, #0x0000");
      asm("MOVW r1, #0x1000");
      asm("STR r1, [r0, r2]");
      for (uint32_t i = 0; i < 2000000; ++i) asm("nop");
      // encender led
      asm("MOVW r0, #0x0C00"); // base puerto B
      asm("MOVT r0, #0x4001");
      asm("MOVW r2, #0x10");
      asm("MOVW r1, #0x0000");
      asm("MOVT r1, #0x1000");
      asm("STR r1, [r0, r2]");
  }
}
