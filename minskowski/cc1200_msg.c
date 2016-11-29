 /*
 * CC1200 CW Test Application
 *
 * Copyright (C) 2015 University of Utah
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

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <time.h>

/* Defines and register set */
#include "cc1200-const.h"
#include "cc1200-rf-cfg.h"
#include "cc1200-802154g-434mhz-2gfsk-50kbps.h"

/******************************************************************************
 * Local Macro Declarations                                                    * 
 ******************************************************************************/
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define SPI_PATH 	"/dev/spidev2.0"

/*---------------------------------------------------------------------------*/
/* RF configuration */
/*---------------------------------------------------------------------------*/
/* Import the rf configuration set by CC1200_RF_CFG */
extern const cc1200_rf_cfg_t CC1200_RF_CFG;
#define CC1200_RF_CFG cc1200_802154g_434mhz_2gfsk_50kbps

// CC1200 SPI
static uint32_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 7700000;
static uint16_t delay = 1;

static int fd;

static uint8_t buf[1024];

static uint8_t tx_msg[] = {0x18, 0, 'T', 'I', 'C', 'C', '1', '2', '0', '0', 'A', 'L'};
static uint8_t rx_msg[ARRAY_SIZE(tx_msg)] = {0, };

struct timespec diff(struct timespec start, struct timespec end)
{
	struct timespec temp;
	if ((end.tv_nsec - start.tv_nsec) < 0) {
		temp.tv_sec = end.tv_sec - start.tv_sec - 1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec - start.tv_sec;
		temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	return temp;
}

static int cc1200_cmd_strobe(uint8_t cmd)
{
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	buf[transfer.len++] = cmd;

	// send the SPI message (all of the above fields, inc. buffers)
	return ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
}

static int
cc1200_get_status(uint8_t *status)
{
	int ret;
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	buf[transfer.len++] = CC1200_SNOP;

	// send the SPI message (all of the above fields, inc. buffers)
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(ret < 0)
		return ret;
	else
		*status = buf[0];
	return ret;
}

static int cc1200_write_register(uint16_t reg, uint8_t value)
{
	int ret;
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// Reg
	if (!CC1200_IS_EXTENDED_ADDR(reg)) {
		buf[transfer.len++] = CC1200_WRITE_BIT | reg;
		buf[transfer.len++] = value;
	} 
	// Extended Address
	else {
		buf[transfer.len++] = CC1200_WRITE_BIT | CC1200_EXT_REG_MASK;
		buf[transfer.len++] = CC1200_UNEXTEND_ADDR(reg);
		buf[transfer.len++] = value;
	}

	// send the SPI message (all of the above fields, inc. buffers)
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	
	return ret;
}

static void cc1200_write_reg_settings(const registerSetting_t *reg_settings,
		uint16_t sizeof_reg_settings)
{
	int i = sizeof_reg_settings / sizeof(registerSetting_t);

	if(reg_settings != NULL) {
		while(i--) {
			cc1200_write_register(reg_settings->addr,
					reg_settings->val);
			reg_settings++;
		}
	}
}

static int cc1200_read_register(uint16_t reg, uint8_t *data)
{
	int ret;
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// Reg
	if (!CC1200_IS_EXTENDED_ADDR(reg)) {
		buf[transfer.len++] = CC1200_READ_BIT | reg;
	} 
	// Extended Address
	else {
		buf[transfer.len++] = CC1200_READ_BIT | CC1200_EXT_REG_MASK;
		buf[transfer.len++] = CC1200_UNEXTEND_ADDR(reg);
	}

	buf[transfer.len++] = CC1200_SNOP;

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(ret < 0)
		return ret;
	else
		*data = buf[transfer.len-1];

	// send the SPI message (all of the above fields, inc. buffers)
	return ret;
}


static int
cc1200_write_txfifo(uint8_t *data, uint8_t len)
{
	int ret;
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	buf[transfer.len++] = CC1200_FIFO | CC1200_WRITE_BIT | CC1200_BURST_BIT;
	transfer.len += len;

	int j;
	for (j = 0; j < len; j++)
	{
		buf[j+1] = data[j];
	}

	// send the SPI message (all of the above fields, inc. buffers)
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);

	return ret;
}

static int
cc1200_read_rxfifo(uint8_t *data, uint8_t len)
{
	int ret;
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)data,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	
	buf[transfer.len++] = CC1200_FIFO | CC1200_READ_BIT | CC1200_BURST_BIT;
	transfer.len += len;

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(ret < 0)
		return ret;

	// send the SPI message (all of the above fields, inc. buffers)
	return ret;
}

/// MAIN ///
int main(int argc, char* argv[]){
	uint8_t partnum = 0;
	uint8_t partver = 0;
	uint8_t isTX = 0;
	
	if(argc != 2)
	{
		printf("usage: %s T/RX\n", argv[0]);
		return 0;
	}
	else if(argc == 2)
	{
		isTX = atoi(argv[1]);
	}

	// The following calls set up the CC1200 SPI bus properties
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

	// Check that the properties have been set
	printf("SPI Mode is: %d\n", mode);
	printf("SPI Bits is: %d\n", bits);
	printf("SPI Speed is: %d\n", speed);

	// Reset Radio
	cc1200_cmd_strobe(CC1200_SRES);

	// Get Chip Info
	cc1200_read_register(CC1200_PARTNUMBER, &partnum);
	cc1200_read_register(CC1200_PARTVERSION, &partver);
	printf("CC1200 Chip Number: 0x%x Chip Version: 0x%x\n", partnum, partver);

	// Write registers to radio
	cc1200_write_reg_settings(CC1200_RF_CFG.register_settings, CC1200_RF_CFG.size_of_register_settings);

	uint8_t rxbytes;
	uint8_t status;

	// T/RX
	if (!isTX) {
		printf("RX Mode!\n");

		// RX
		cc1200_cmd_strobe(CC1200_SRX);

		while(1)
		{
			cc1200_read_register(CC1200_MARC_STATUS1, &status);
			if(status == CC1200_MARC_STATUS1_RX_SUCCEED)
			{
				cc1200_read_register(CC1200_NUM_RXBYTES, &rxbytes);
				if (rxbytes != 0)
				{
					cc1200_read_register(CC1200_MARCSTATE, &status);
					if ((status & 0x1F) == CC1200_MARC_STATE_RX_FIFO_ERR)
					{
						cc1200_cmd_strobe(CC1200_SFRX);
					}
					else
					{
						cc1200_read_rxfifo(rx_msg, rxbytes);

						printf("MSG Received! DATA: %s\n", rx_msg);

						cc1200_cmd_strobe(CC1200_SFRX);
					}
				}

				cc1200_cmd_strobe(CC1200_SRX);
			}
		}
	}
	else {
		printf("TX Mode!\n");

		while(1)
		{
			tx_msg[1] = sizeof(tx_msg);
			printf("size: %d\n", sizeof(tx_msg));

			// Write data into FIFO
			cc1200_write_txfifo(tx_msg, sizeof(tx_msg));

			// Check status
			cc1200_get_status(&status);
			if ((status & 0xF0) == CC1200_STATUS_BYTE_TX_FIFO_ERR) {
				printf("cc1200 tx fifo error\n");
				cc1200_cmd_strobe(CC1200_SFTX);
				continue;
			}

			// TX
			cc1200_cmd_strobe(CC1200_STX);

			// Check if TX completed
			cc1200_read_register(CC1200_MARC_STATUS1, &status);
			while(status != CC1200_MARC_STATUS1_TX_SUCCEED)
			{
				cc1200_read_register(CC1200_MARC_STATUS1, &status);
				usleep(1000);
			};

			printf("MSG SENT! %s\n", tx_msg);

			usleep(1000000);
		}
	}

	close(fd);               //close the file
	return 0;
}
