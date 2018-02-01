/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    clase4.c
 * @brief   Application entry point.
 */
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */

void PORTA_IRQHandler()
{
	static uint8_t state = 0;

	/*Limpiamos la bandera del pin que causo la interrupcion*/
	PORT_ClearPinsInterruptFlags(PORTA, 1<<4);

	/*Escribimos el led segun el valor de state*/
	GPIO_WritePinOutput (GPIOB, 21, state);

	/*Si state es igual a 0 entonces se hace 1 y al revés*/
	state = ( 0 == state ) ? 1 : 0;
}

int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();


    /*Habilitar el reloj SCG*/
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortA);

    /*Configurar el puerto para encender un LED*/
    /* Input pin PORT configuration */
    port_pin_config_t config_led = {
    		kPORT_PullDisable,				/*Resistencias deshabilitadas*/
    		kPORT_SlowSlewRate,				/*SlewRate menor velocidad*/
    		kPORT_PassiveFilterEnable,		/*Filtro habilitado*/
    		kPORT_OpenDrainDisable,			/**/
    		kPORT_LowDriveStrength,			/**/
    		kPORT_MuxAsGpio,				/*Modo GPIO*/
    		kPORT_UnlockRegister };			/**/

    /* Input pin PORT configuration */
    port_pin_config_t config_switch = {
    		kPORT_PullDisable,
    		kPORT_SlowSlewRate,
    		kPORT_PassiveFilterEnable,
    		kPORT_OpenDrainDisable,
    		kPORT_LowDriveStrength,
    		kPORT_MuxAsGpio,
    		kPORT_UnlockRegister};

    PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptFallingEdge);
    /* Sets the configuration */
    PORT_SetPinConfig(PORTB, 21, &config_led);
    PORT_SetPinConfig(PORTA, 4, &config_switch);


    NVIC_EnableIRQ(PORTA_IRQn);

    /* Output pin configuration */
    gpio_pin_config_t led_config = { kGPIO_DigitalOutput, 1 };
    gpio_pin_config_t switch_config = { kGPIO_DigitalInput, 0 };

    /* Sets the configuration */
    GPIO_PinInit(GPIOB, 21, &led_config);
    GPIO_PinInit(GPIOA, 4, &switch_config);


    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;

    PRINTF("Hello World!\n");

    /* Sets the output level of one GPIO pin to the logic 1 or 0 */
    /*GPIO_WritePinOutput (GPIOB, 21, 0);*/



    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
    }
    return 0 ;
}
