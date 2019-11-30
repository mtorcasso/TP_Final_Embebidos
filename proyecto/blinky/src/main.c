/* Copyright 2016, Pablo Ridolfi
 * All rights reserved.
 *
 * This file is part of Workspace.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
 
/** @brief This is a simple blink example.
 */

/** \addtogroup blink Bare-metal blink example
 ** @{ */

/*==================[inclusions]=============================================*/

#include "main.h"
#include "board.h"


static uint32_t counter;

char* itoa(int value, char* result, int base) {
   // check that the base if valid
   if (base < 2 || base > 36) { *result = '\0'; return result; }

   char* ptr = result, *ptr1 = result, tmp_char;
   int tmp_value;

   do {
      tmp_value = value;
      value /= base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return result;
}


void SysTick_Handler(void)
{
	if(counter > 0) counter--;
}


int main(void)
{
	uint16_t r;
	int muestra;
	static char uartBuff[10];
	Board_UARTPutSTR(DEBUG_UART, "Caca\n");
	while (1)
	{

		if (counter==0){
			counter=DELAY_MS;
			Board_LED_Toggle(LED);

			Board_ADC_ReadBegin(ADC_CH0);
			while(Board_ADC_ReadWait());
			muestra=Board_ADC_ReadEnd();
			itoa(muestra, uartBuff, 10 );
			Board_UARTPutSTR(DEBUG_UART, "CH0: ");
			Board_UARTPutSTR(DEBUG_UART, uartBuff);
			Board_UARTPutSTR(DEBUG_UART, "\r\n");

			Board_ADC_ReadBegin(ADC_CH1);
			while(Board_ADC_ReadWait());
			muestra=Board_ADC_ReadEnd();
			itoa(muestra, uartBuff, 10 );
			Board_UARTPutSTR(DEBUG_UART, "CH1: ");
			Board_UARTPutSTR(DEBUG_UART, uartBuff);
			Board_UARTPutSTR(DEBUG_UART, "\r\n");

			Board_ADC_ReadBegin(ADC_CH2);
			while(Board_ADC_ReadWait());
			muestra=Board_ADC_ReadEnd();
			itoa(muestra, uartBuff, 10 );
			Board_UARTPutSTR(DEBUG_UART, "CH2: ");
			Board_UARTPutSTR(DEBUG_UART, uartBuff);
			Board_UARTPutSTR(DEBUG_UART, "\r\n");

			Board_ADC_ReadBegin(ADC_CH3);
			while(Board_ADC_ReadWait());
			muestra=Board_ADC_ReadEnd();
			itoa(muestra, uartBuff, 10 );
			Board_UARTPutSTR(DEBUG_UART, "CH3: ");
			Board_UARTPutSTR(DEBUG_UART, uartBuff);
			Board_UARTPutSTR(DEBUG_UART, "\r\n");
		}
	}
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
