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



char* itoa(int32_t value, char* result, int base) {
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
 uint32_t strlen(const char* str){
	 uint32_t i=0;
	 while(str[i]!='\0') i++;
	 return i;
 }

 /*Defino numerador y denominador para la conversión de muestras del adc*/
 /*La cuenta es v_real=v_pin_adc*(156/100) dado que hay un divisor resistivo. A su vez v_pin_adc= valor_adc*(3300mV/1024bits)*/
 /* Entonces v_real=valor_adc*(1287/256)*/
#define conv_num 1287
#define conv_den 256
#define engine_temp_table_size 25
#define mat_table_size 29

int32_t getEngineTemp(int32_t* v_table,int32_t* temp_table,int16_t scale,int16_t offset){
	int32_t muestra=0;
	unsigned char i=0;
	Board_ADC_ReadBegin(ADC_CH0);
	while(Board_ADC_ReadWait());
	//Obtengo muestra y convierto a mV
	muestra=((int32_t)Board_ADC_ReadEnd()*conv_num)/conv_den;

	char caca [10];
	itoa(muestra, caca, 10 );
	Board_UARTPutSTR(DEBUG_UART,caca);
	Board_UARTPutSTR(DEBUG_UART,"\r\n");

	//Hago un ajuste lineal entre valores de la tabla
	while ((v_table[i]<muestra) && (i<engine_temp_table_size)) i++;
	return (((((temp_table[i]-temp_table[i-1])*(muestra-v_table[i-1]))/(v_table[i]-v_table[i-1]))+temp_table[i-1])*scale/100 + offset);

}

int32_t getMAT(int32_t* v_table,int32_t* temp_table,int16_t scale,int16_t offset){
	int32_t muestra=0;
	unsigned char i=0;
	Board_ADC_ReadBegin(ADC_CH1);
	while(Board_ADC_ReadWait());
	/*Obtengo muestra y convierto a mV*/
	muestra=((int32_t)Board_ADC_ReadEnd()*conv_num)/conv_den;

	/*Hago un ajuste lineal entre valores de la tabla*/
	while ((v_table[i]<muestra) && (i<mat_table_size)) i++;
	return (((((temp_table[i]-temp_table[i-1])*(muestra-v_table[i-1]))/(v_table[i]-v_table[i-1]))+temp_table[i-1])*scale/100 + offset);

}

int32_t getMAP(int16_t scale,int16_t offset){
	int32_t muestra=0;
	unsigned char i=0;
	Board_ADC_ReadBegin(ADC_CH2);
	while(Board_ADC_ReadWait());
	/*Obtengo muestra y convierto a mV*/
	muestra=((int32_t)Board_ADC_ReadEnd()*conv_num)/conv_den;

	/*Respuesta lineal, 0v=0%, 5000mV=100% (recordar que uso 5v porque en la conversión de muestra ya adapte 3.3 a 5)*/
	/*Ademas recordar que scale es sobre 100 */
	return ((scale*muestra)/5000 + offset);
}

int32_t getTPS(int16_t scale,int16_t offset){
	int32_t muestra=0;
	unsigned char i=0;
	Board_ADC_ReadBegin(ADC_CH3);
	while(Board_ADC_ReadWait());
	/*Obtengo muestra y convierto a mV*/
	muestra=((int32_t)Board_ADC_ReadEnd()*conv_num)/conv_den;
	/*Respuesta lineal, 0v=0%, 5000mV=100% (recordar que uso 5v porque en la conversión de muestra ya adapte 3.3 a 5)*/
	/*Ademas recordar que scale es sobre 100 */
	return ((scale*muestra)/5000 + offset);
}

int32_t getRPM(){

	/*
	La condicion de timediff mayor que 0xfffffffe se da cuando todavia falta tomar un valor para determinar
	el periodo, o cuando el periodo es muy largo, es decir, motor detenido (ver comentarios handler timer)
	*/
	if (time_diff<0xFFFFFFFE){
		//Motor 4 cil, 2 pulsos por vuelta -> 60 seg/min * 1000000uS/(2*timediff S/vuelta)
		return (30000000/time_diff);
	}
	else return 0;
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

void SysTick_Handler(void)
{
	if(counter > 0) counter--;
}


int main(void)
{
	int uart_task=255; /* Variable para pedir dato por UART */
	static char uartBuff[10];	/* Buffer para itoa */

	/* Variables sensores */
	int32_t EngineTemp=0;
	int32_t MAT=0;
	int32_t MAP=0;
	int32_t TPS=0;
	int32_t RPM=0;

	/* Variables calibración. Recordar que scale va en scale/100 */
	int16_t TM_scale=100;
	int16_t MAT_scale=100;
	int16_t MAP_scale=100;
	int16_t TPS_scale=100;
	int16_t TM_offset=0;
	int16_t MAT_offset=0;
	int16_t MAP_offset=0;
	int16_t TPS_offset=0;

	/*Tension en mV*/
	int32_t tmotor_x [engine_temp_table_size]= {240,280,330,370,430,510,580,660,740,880,990,1150,1300,1490,1670,1910,2150,2430,2680,2940,3190,3470,3680,3900,4080};
	/*Temperatura en °C*/
	int32_t tmotor_y [engine_temp_table_size]= {120,115,110,105,100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5,0};

	/*Tension en mV*/
	int32_t mat_x[mat_table_size]={50,134,200,266,347,394,466,545,595,665,761,870,979,1142,1275,1446,1624,1821,2010,2240,2470,2680,2970,3240,3510,3720,3910,4070,4200};
	/*Temperatura en °C*/
	int32_t mat_y[mat_table_size]={125,120,115,110,105,100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5,0,-5,-10,-15};

	Timer2_incap_init();

	while (1)
	{
		if (match_flag==1){
			match_flag=0;
		}

		uart_task=Board_UARTGetChar(BLUETOOTH_UART);

		switch (uart_task){

		case 0:
			itoa(EngineTemp, uartBuff, 10 );
			Board_UARTPutChar(BLUETOOTH_UART,strlen(uartBuff));
			Board_UARTPutSTR(BLUETOOTH_UART, uartBuff);
			uart_task=255;
		break;

		case 1:
			itoa(MAT, uartBuff, 10 );;
			Board_UARTPutChar(BLUETOOTH_UART,strlen(uartBuff));
			Board_UARTPutSTR(BLUETOOTH_UART, uartBuff);
			uart_task=255;
		break;

		case 2:
			itoa(MAP, uartBuff, 10 );
			Board_UARTPutChar(BLUETOOTH_UART,strlen(uartBuff));
			Board_UARTPutSTR(BLUETOOTH_UART, uartBuff);
			uart_task=255;
		break;

		case 3:
			itoa(TPS, uartBuff, 10 );
			Board_UARTPutChar(BLUETOOTH_UART,strlen(uartBuff));
			Board_UARTPutSTR(BLUETOOTH_UART, uartBuff);
			uart_task=255;
		break;

		case 4:
			itoa(RPM, uartBuff, 10 );
			Board_UARTPutChar(BLUETOOTH_UART,strlen(uartBuff));
			Board_UARTPutSTR(BLUETOOTH_UART, uartBuff);
			uart_task=255;
		break;
/*
		case 5:
			TM_offset=Board_UARTGetChar(BLUETOOTH_UART);
		break;

		case 6:
			TM_scale=Board_UARTGetChar(BLUETOOTH_UART);
		break;

		case 7:
			MAT_offset=Board_UARTGetChar(BLUETOOTH_UART);
		break;

		case 8:
			MAT_scale=Board_UARTGetChar(BLUETOOTH_UART);
		break;

		case 9:
			MAP_offset=Board_UARTGetChar(BLUETOOTH_UART);
		break;

		case 10:
			MAP_scale=Board_UARTGetChar(BLUETOOTH_UART);
		break;

		case 11:
			TPS_offset=Board_UARTGetChar(BLUETOOTH_UART);
		break;

		case 12:
			TPS_scale=Board_UARTGetChar(BLUETOOTH_UART);
		break;
*/
		default:
			uart_task=255;
		break;
		}

		if (counter==0){
			counter=DELAY_MS;
			Board_LED_Toggle(LED);

			/*Actualizo valores de todos los sensores*/
			EngineTemp=getEngineTemp(tmotor_x,tmotor_y,TM_scale,TM_offset);
			MAT=getMAT(mat_x,mat_y,MAT_scale,MAT_offset);
			MAP=getMAP(MAP_scale,MAP_offset);
			TPS=getTPS(TPS_scale,TPS_offset);
			RPM=getRPM();

		}
	}
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
