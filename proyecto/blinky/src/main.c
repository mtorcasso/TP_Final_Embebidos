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

volatile uint8_t match_flag=0;
volatile uint32_t time_diff=0xFFFFFFFF; //Inicializo motor detenido
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

#define timer_limit 1000000
void Timer2_incap_init(void){
	Chip_SCU_PinMux(6,1, MD_PLN_FAST, SCU_MODE_FUNC5); //config SCU en port 6 pin 1
	Chip_TIMER_Init(LPC_TIMER2);
	   LPC_GIMA->CAP0_IN[2][0] = (2<<4); //config GIMA en timer 2, cap0
	   Chip_RGU_TriggerReset(RGU_TIMER2_RST);

	   while (Chip_RGU_InReset(RGU_TIMER2_RST));
	    Chip_TIMER_Reset(LPC_TIMER2);
	    Chip_TIMER_TIMER_SetCountClockSrc(LPC_TIMER2, TIMER_CAPSRC_RISING_PCLK, 0);

	    //Configuro captura
	    Chip_TIMER_PrescaleSet(LPC_TIMER2, 204); // Cuenta a 204M/204= 1mhz = 1us
	    Chip_TIMER_ClearCapture(LPC_TIMER2, 0);
	    Chip_TIMER_CaptureRisingEdgeEnable(LPC_TIMER2, 0);
	    //Chip_TIMER_CaptureFallingEdgeEnable (LPC_TIMER2, 0);
	    Chip_TIMER_CaptureEnableInt(LPC_TIMER2, 0);

	    //Configuro matcheo para valor limite
	    Chip_TIMER_Reset(LPC_TIMER2);
		Chip_TIMER_MatchEnableInt(LPC_TIMER2, 2);
		Chip_TIMER_SetMatch(LPC_TIMER2, 2, timer_limit); //cada 1 segundo reseteo
		Chip_TIMER_ResetOnMatchEnable(LPC_TIMER2, 2);

	    NVIC_ClearPendingIRQ(TIMER2_IRQn);
	    NVIC_EnableIRQ(TIMER2_IRQn);
	    Chip_TIMER_Enable(LPC_TIMER2);

}


void TIMER2_IRQHandler(void)
{
	static uint32_t overflow=0;
	static uint32_t hall_set_timeold=0;
	static uint32_t hall_set_timenew=0;
	if (Chip_TIMER_CapturePending(LPC_TIMER2, 0)) {
		Chip_TIMER_ClearCapture(LPC_TIMER2, 0);

		hall_set_timeold=hall_set_timenew;
		hall_set_timenew=Chip_TIMER_ReadCapture(LPC_TIMER2, 0);

		//si el motor esta parado no cuento hasta que llegue la segunda medición
		if (time_diff!=0xFFFFFFFF) time_diff= hall_set_timenew+overflow-hall_set_timeold;
		//Como llego un pulso, ya no esta detenido (en la proxima iteracion se actulizara time_diff
		else time_diff=0xFFFFFFFE;
		overflow=0;
	}

	if (Chip_TIMER_MatchPending(LPC_TIMER2, 2)) {
			Chip_TIMER_ClearMatch(LPC_TIMER2, 2);
			if (overflow!=0){
				hall_set_timenew=0;
				overflow=0;
				time_diff=0xFFFFFFFF; //Condicion de motor detenido
			}
			else overflow=timer_limit;
			match_flag=1;
	}
}




int main(void)
{
	uint16_t r;
	int muestra;
	static char uartBuff[10];

	Timer2_incap_init();

	Board_UARTPutSTR(DEBUG_UART, "Hola\n");
	while (1)
	{
		if (match_flag==1){
			match_flag=0;
		}

		if (counter==0){
			counter=DELAY_MS;
			Board_LED_Toggle(LED);
			if (time_diff<0xFFFFFFFE){
				itoa(time_diff, uartBuff, 10 );
				Board_UARTPutSTR(DEBUG_UART, "Periodo en us: ");
				Board_UARTPutSTR(DEBUG_UART, uartBuff);
				Board_UARTPutSTR(DEBUG_UART, "\r\n");
			}
			else Board_UARTPutSTR(DEBUG_UART, "Motor detenido \r\n");

			/*
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

			*/
		}
	}
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
