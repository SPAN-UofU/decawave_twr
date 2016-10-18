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
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#define SPI_SPEED_SLOW    				( 3000000)
#define SPI_SPEED_FAST  	  			(10000000)
#define SPI_PATH 						"/dev/spidev1.0"

static uint32_t mode 	= 0;
static uint8_t bits 	= 8;
static uint32_t speed 	= SPI_SPEED_SLOW;
static uint16_t delay 	= 0;

static int fd;

int GPIOPin = 60; /* Reset GPIO pin - GPIO1_28 or pin 12 on the P9 header */
FILE *resetGPIO = NULL;

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_sleep(unsigned int time_ms)
{
    sleep_ms(time_ms);
}

void sleep_ms(unsigned int time_ms)
{
    usleep(time_ms * 1000);
}

int spi_set_rate_low (void)
{
	speed = SPI_SPEED_SLOW;
    if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't set max speed HZ");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}

        return 0;
}

int spi_set_rate_high (void)
{
	speed = SPI_SPEED_FAST;
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't set max speed HZ");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}

        return 0;
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
	int status;

	struct spi_ioc_transfer transfer1 = {
		.tx_buf = (unsigned long)headerBuffer,
		.rx_buf = (unsigned long)NULL,
		.len = headerLength,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// send the SPI message (all of the above fields, inc. buffers)
	status = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer1);
	if(status < 0)
		return DWT_ERROR;

	struct spi_ioc_transfer transfer2 = {
		.tx_buf = (unsigned long)bodyBuffer,
		.rx_buf = (unsigned long)NULL,
		.len = bodylength,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// send the SPI message (all of the above fields, inc. buffers)
	status = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer2);
	if(status < 0)
		return DWT_ERROR;

        return DWT_SUCCESS;


} // end writetospi()

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
	int status;

	struct spi_ioc_transfer transfer1 = {
		.tx_buf = (unsigned long)headerBuffer,
		.rx_buf = (unsigned long)NULL,
		.len = headerLength,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// send the SPI message (all of the above fields, inc. buffers)
	status = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer1);
	if(status < 0)
		return DWT_ERROR;

	struct spi_ioc_transfer transfer2 = {
		.tx_buf = (unsigned long)NULL,
		.rx_buf = (unsigned long)readBuffer,
		.len = readlength,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// send the SPI message (all of the above fields, inc. buffers)
	status = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer2);
	if(status < 0)
		return DWT_ERROR;

        return DWT_SUCCESS;

} // end readfromspi()

int hardware_init (void)
{
    char setValue[4], GPIOString[4], GPIOValue[64], GPIODirection[64];
    sprintf(GPIOString, "%d", GPIOPin);
    sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", GPIOPin);
    sprintf(GPIODirection, "/sys/class/gpio/gpio%d/direction", GPIOPin);
 
    // Export the pin
    if ((resetGPIO = fopen("/sys/class/gpio/export", "ab")) == NULL){
        printf("Unable to export GPIO pin\n");
        return 1;
    }
    strcpy(setValue, GPIOString);
    fwrite(&setValue, sizeof(char), 2, resetGPIO);
    fclose(resetGPIO);
 
    // Set direction of the pin to an output
    if ((resetGPIO = fopen(GPIODirection, "rb+")) == NULL){
        printf("Unable to open direction handle\n");
        return 1;
    }
    strcpy(setValue,"out");
    fwrite(&setValue, sizeof(char), 3, resetGPIO);
    fclose(resetGPIO);

	// The following calls set up the SPI bus properties
	if((fd = open(SPI_PATH, O_RDWR))<0){
		perror("SPI Error: Can't open device.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_MODE, &mode)==-1){
		perror("SPI: Can't set SPI mode.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_MODE, &mode)==-1){
		perror("SPI: Can't get SPI mode.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)==-1){
		perror("SPI: Can't set bits per word.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits)==-1){
		perror("SPI: Can't get bits per word.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't set max speed HZ");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}
    return 0;
}

int reset_DW1000(void)
{
    char setValue[4], GPIOValue[64];
    sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", GPIOPin);
    
    // Set output to low
    if ((resetGPIO = fopen(GPIOValue, "rb+")) == NULL){
        printf("Unable to open value handle\n");
        return 1;
    }
    strcpy(setValue, "0"); // Set value low
    fwrite(&setValue, sizeof(char), 1, resetGPIO);
    fclose(resetGPIO);
    sleep_ms(2);

	// Set output to high
    if ((resetGPIO = fopen(GPIOValue, "rb+")) == NULL){
        printf("Unable to open value handle\n");
        return 1;
    }
    strcpy(setValue, "1"); // Set value high
    fwrite(&setValue, sizeof(char), 1, resetGPIO);
    fclose(resetGPIO);
    sleep_ms(2);
    return 0;
}

decaIrqStatus_t decamutexon(void) 
{
	decaIrqStatus_t s = 0;//port_GetEXT_IRQStatus();

	if(s) {
		// no interrupt lines
	}
	return s ;   // return state before disable, value is used to re-enable in decamutexoff call
}

void decamutexoff(decaIrqStatus_t s)        // put a function here that re-enables the interrupt at the end of the critical section
{
	if(s) { //need to check the port state as we can't use level sensitive interrupt on the STM ARM
		// no interrupt lines
	}
}

void dwt_readtx_sys_count(uint8 * timestamp)
{
    dwt_readfromdevice(TX_TIME_ID, TX_TIME_TX_RAWST_OFFSET, TX_TIME_TX_STAMP_LEN, timestamp) ; // Read bytes directly into buffer
}

void dwt_readrx_sys_count(uint8 * timestamp)
{
    dwt_readfromdevice(RX_TIME_ID, RX_TIME_RX_RAWST_OFFSET, RX_TIME_RX_STAMP_LEN, timestamp) ; // Read bytes directly into buffer
}