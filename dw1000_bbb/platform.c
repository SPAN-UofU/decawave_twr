/*
 * platform.c
 *
 * Copyright (C) 2016 University of Utah
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Written by:
 * Anh Luong <luong@eng.utah.edu>
 */

#include "platform.h"
#include <unistd.h> // for usleep

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_sleep(unsigned int time_ms)
{
    sleep_ms(time_ms);
}

void sleep_ms(unsigned int time_ms)
{
    usleep(time_ms * 1000);
}

void spi_set_rate_low (void)
{
    //SPI_ChangeRate(SPI_BaudRatePrescaler_32);
}

void spi_set_rate_high (void)
{
    //SPI_ChangeRate(SPI_BaudRatePrescaler_4);
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
	/*
	int i=0;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;

    SPIx_CS_GPIO->BRR = SPIx_CS;

    for(i=0; i<headerLength; i++)
    {
    	SPIx->DR = headerBuffer[i];

    	while ((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

    	SPIx->DR ;
    }

    for(i=0; i<bodylength; i++)
    {
     	SPIx->DR = bodyBuffer[i];

    	while((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

		SPIx->DR ;
	}

    SPIx_CS_GPIO->BSRR = SPIx_CS;

    decamutexoff(stat) ;
	*/

    return 0;
} // end writetospi()

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
	/*
	int i=0;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;

    // Wait for SPIx Tx buffer empty
    //while (port_SPIx_busy_sending());

    SPIx_CS_GPIO->BRR = SPIx_CS;

    for(i=0; i<headerLength; i++)
    {
    	SPIx->DR = headerBuffer[i];

     	while((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

     	readBuffer[0] = SPIx->DR ; // Dummy read as we write the header
    }

    for(i=0; i<readlength; i++)
    {
    	SPIx->DR = 0;  // Dummy write as we read the message body

    	while((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

	   	readBuffer[i] = SPIx->DR ;//port_SPIx_receive_data(); //this clears RXNE bit
    }

    SPIx_CS_GPIO->BSRR = SPIx_CS;

    decamutexoff(stat) ;
    */

    return 0;
} // end readfromspi()

void hardware_init (void)
{
	/*
	clock_init();
	gpio_init();
	interrupt_init();
	spi_init();
	*/

	/*
	// enable clock for PORTs
	CLOCK_SYS_EnablePortClock(PORTA_IDX);
	CLOCK_SYS_EnablePortClock(PORTC_IDX);
	CLOCK_SYS_EnablePortClock(PORTE_IDX);

	// Init board clock
	BOARD_ClockInit();
	dbg_uart_init();

	// Configure SPI pins
	configure_spi_pins(0);
	*/
}

void reset_DW1000(void)
{
	/*
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIO used for DW1000 reset
	GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

	//drive the RSTn pin low
	GPIO_ResetBits(DW1000_RSTn_GPIO, DW1000_RSTn);

	//put the pin back to tri-state ... as input
	GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);
	*/

    sleep_ms(2);
}

decaIrqStatus_t decamutexon(void)
{
	/*
	decaIrqStatus_t s = port_GetEXT_IRQStatus();

	if(s) {
		port_DisableEXT_IRQ(); //disable the external interrupt line
	}
	return s ;   // return state before disable, value is used to re-enable in decamutexoff call
	*/
	return -1;
}

void decamutexoff(decaIrqStatus_t s)        // put a function here that re-enables the interrupt at the end of the critical section
{
	/*
	if(s) { //need to check the port state as we can't use level sensitive interrupt on the STM ARM
		port_EnableEXT_IRQ();
	}
	*/
}
