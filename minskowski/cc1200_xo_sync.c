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
#include "cc1200-802154g-434mhz-2gfsk-50kbps-cw.h"

// Standard header files
#include <sys/mman.h>
#include <errno.h>
#include <sys/time.h>
#include <pruss/prussdrv.h>
#include <pruss/pruss_intc_mapping.h>
#include "SPI_bin.h"

/******************************************************************************
 * Local Macro Declarations                                                    * 
 ******************************************************************************/
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define PRU_NUM 	0
#define OFFSET_SHAREDRAM 0		//equivalent with 0x00002000

#define PRUSS0_SHARED_DATARAM    4

// Standard
#define	HIGH							1
#define	LOW								0

#define SPI_PATH 	"/dev/spidev2.0"
#define XO_PATH 	"/dev/spidev2.1"

#define pi 3.14159265358979323846

#define verbose		0

/*---------------------------------------------------------------------------*/
/* RF configuration */
/*---------------------------------------------------------------------------*/
/* Import the rf configuration set by CC1200_RF_CFG */
extern const cc1200_rf_cfg_t CC1200_RF_CFG;
#define CC1200_RF_CFG cc1200_802154g_434mhz_2gfsk_50kbps_cw

/******************************************************************************
 * Global variable Declarations                                                * 
 ******************************************************************************/
struct IQSample {
	uint8_t status0;
	uint8_t status1;
	uint8_t magn2;
	uint8_t magn1;
	uint8_t magn0;
	uint8_t ang1;
	uint8_t ang0;
} iqsample;

// CC1200 SPI
static uint32_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 7700000;
static uint16_t delay = 1;

// XO SPI
static uint32_t xo_mode = 0;
static uint8_t xo_bits = 16;
static uint32_t xo_speed = 20000000;
static uint16_t xo_delay = 1;

static void *sharedMem;
static unsigned int *sharedMem_int;
static struct IQSample *sharedMem_struct;

static int max_samples_size = 1000;
//static int avg_count = 10;

static int fd, xo_fd;

static uint8_t buf[1024];

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

	//hex_dump(buf, transfer.len, 32, "RX");

	// send the SPI message (all of the above fields, inc. buffers)
	return ret;
}

static void xo_transfer(int fd, uint16_t const *tx, uint16_t const *rx, size_t len)
{
	int ret;

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = xo_delay,
		.speed_hz = xo_speed,
		.bits_per_word = xo_bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		perror("XO SPI: can't send spi message");
		return;
}

/// MAIN ///
int main(int argc, char* argv[]){
	int i, j = 0;
	uint8_t partnum = 0;
	uint8_t partver = 0;
	uint8_t isTX = 0;
	int num_samples = 0;

	if(argc != 2)
	{
		printf("TX MODE\n");
		//return 0;
		isTX = 1;
	}
	else if(argc == 2)
	{
		printf("RX MODE\n");
		num_samples = atoi(argv[1]);
			
		if(num_samples <= 0)
		{
			printf("usage: %s [samples]\n", argv[0]);
			printf("RX Mode: samples > 0\n");
			return 0;
		}
	}
	else
	{
		printf("usage: %s [samples]\n", argv[0]);
		printf("RX Mode: samples > 0\n");
		return 0;
	}


	// The following calls set up the XO SPI bus properties
	if((xo_fd = open(XO_PATH, O_RDWR))<0){
		perror("XO SPI Error: Can't open device.");
		return -1;
	}
	if(ioctl(xo_fd, SPI_IOC_WR_MODE, &xo_mode)==-1){
		perror("XO SPI: Can't set SPI mode.");
		return -1;
	}
	if(ioctl(xo_fd, SPI_IOC_RD_MODE, &xo_mode)==-1){
		perror("XO SPI: Can't get SPI mode.");
		return -1;
	}
	if(ioctl(xo_fd, SPI_IOC_WR_BITS_PER_WORD, &xo_bits)==-1){
		perror("XO SPI: Can't set bits per word.");
		return -1;
	}
	if(ioctl(xo_fd, SPI_IOC_RD_BITS_PER_WORD, &xo_bits)==-1){
		perror("XO SPI: Can't get bits per word.");
		return -1;
	}
	if(ioctl(xo_fd, SPI_IOC_WR_MAX_SPEED_HZ, &xo_speed)==-1){
		perror("XO SPI: Can't set max speed HZ");
		return -1;
	}
	if(ioctl(xo_fd, SPI_IOC_RD_MAX_SPEED_HZ, &xo_speed)==-1){
		perror("XO SPI: Can't get max speed HZ.");
		return -1;
	}

	// Check that the properties have been set
	printf("XO SPI Mode is: %d\n", xo_mode);
	printf("XO SPI Bits is: %d\n", xo_bits);
	printf("XO SPI Speed is: %d\n", xo_speed);

	uint16_t default_tx[] = {0, 0};
	uint16_t default_rx[] = {0, 0};
	
	int vco_ctrl = 0x7FFFFF;
	default_tx[0] = (0x3000 + ((vco_ctrl>>12)&0xFFF)) & 0xFFFF; // DAC A
	default_tx[1] = (0xB000 + (vco_ctrl&0xFFF)) & 0xFFFF;		// DAC B
	xo_transfer(xo_fd, default_tx, default_rx, sizeof(default_tx));
	
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

	// T/RX
	if (!isTX) {
		cc1200_cmd_strobe(CC1200_SRX);

		///////// START PRU /////////
		unsigned int ret;
		tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
		
		/* Initializing PRU */
		prussdrv_init();
		ret = prussdrv_open(PRU_EVTOUT_0);
		if (ret){
			printf("\tERROR: prussdrv_open open failed\n");
			return (ret);
		}

		prussdrv_pruintc_init(&pruss_intc_initdata);
		prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, &sharedMem);

		int samples_size = 100;
		sharedMem_int = (unsigned int *) sharedMem;
		sharedMem_int[OFFSET_SHAREDRAM + 0] = samples_size; //samples

		/* Executing PRU. */
		prussdrv_pru_write_memory(PRUSS0_PRU0_IRAM, PRU_NUM, PRUcode, sizeof(PRUcode));

		// Trash the first 100 samples
		prussdrv_pru_enable(PRU_NUM);

		ret = prussdrv_pru_wait_event(PRU_EVTOUT_0);

		prussdrv_pru_disable(PRU_NUM);
		
		// initialize
		float fs = 45044.4;
		float dt = 1.0 / fs;
		float pll_mult = 433.999939 / 40.0; //433.999939
		
		// maybe don't need this with kalman filter?
		//int fmin = 39999607;
		//int fmax = 40000423;
		//int fnom = 40000000;
		//int fmid = 40000017;
		//int frange = fmax - fmin;
		float frange = 800.0;
		
		// time
		struct timespec prev_time;
		clock_gettime(CLOCK_REALTIME, &prev_time);

		samples_size = 0;

		double sum_xo_offset = 0.0, min_xo_offset = 0.0, max_xo_offset = 0.0;

		for(j = 0;j < num_samples; j++)
		{
			//samples_size += (max_samples_size/num_samples);
			samples_size = max_samples_size;
			prussdrv_pruintc_init(&pruss_intc_initdata);
			sharedMem_int[OFFSET_SHAREDRAM + 0] = samples_size; //samples
			prussdrv_pru_enable(PRU_NUM);

			ret = prussdrv_pru_wait_event(PRU_EVTOUT_0);

			sharedMem_struct = ( struct IQSample *) sharedMem;

			// Print out AGC_GAIN3
			uint8_t agc_gain;
			cc1200_read_register(CC1200_AGC_GAIN3, &agc_gain);
		
			long unsigned int last_phase = 0;
			int cumulative_phase = 0;
			double cw_offset = 0.0;
			double xo_offset = 0.0;
			long unsigned int sum_mag = 0;
			
			//long unsigned int cp_start = 0, cp_end = 0;

			// y = ax + b
			//double sumx = 0.0, sumxsq = 0.0, sumy = 0.0, sumxy = 0.0, a = 0.0, b = 0.0, denom = 0.0;

			for(i = 0; i < samples_size; i++) {
				long unsigned int mag = (sharedMem_struct[i].magn0&0xff) + ((sharedMem_struct[i].magn1<<8)&0xff00) + ((sharedMem_struct[i].magn2<<16)&0x010000);
				long unsigned int ang = (sharedMem_struct[i].ang0&0xff) + ((sharedMem_struct[i].ang1<<8)&0xff00);
				
				sum_mag += mag;

				int dp = ang - last_phase;
				last_phase = ang;

				// Phase difference wrapping
				if (dp < -512)
					dp += 1024;
				if (dp > 511)
					dp -= 1024;

				if (i > 0)
				{
					cumulative_phase += dp;
				}

				/*
				if(i < avg_count+1)
					cp_start += ang;
				if(i > samples_size-(avg_count+1))
					cp_end	 += ang;
				*/

				/*
				double x = i;
				double y = dp;
				sumx 	+= x;
				sumxsq 	+= x*x;
				sumy 	+= y;
				sumxy 	+= x*y;
				*/
				//printf("Set# %d Sample# %d MAGN: %lu ANGL: %lu AGC: %d PD: %d CP: %d\n", j, i, mag, ang, agc_gain, dp, cumulative_phase);	
			}

			//long unsigned int first = (sharedMem_struct[0].ang0&0xff) + ((sharedMem_struct[0].ang1<<8)&0xff00);
			//long unsigned int last 	= (sharedMem_struct[samples_size-1].ang0&0xff) + ((sharedMem_struct[samples_size-1].ang1<<8)&0xff00);

			// Update XO
			cw_offset = (cumulative_phase / 1024.0 / ((samples_size - 1) * dt));
			xo_offset = cw_offset / pll_mult;
			xo_offset += 3.688943753; // 138.88
			
			// Linear method
			//int ln_dp = (cp_end/avg_count) - (cp_start/avg_count);
			//double lxo_offset = (ln_dp / 1024.0 / (samples_size * dt)) / pll_mult;
			
			/*
			denom = samples_size * sumxsq - (sumx * sumx);
			a = (sumy * sumxsq - sumx * sumxy) / denom;
			b = (samples_size * sumxy - sumx * sumy) / denom;
			int ln_dp = (a * samples_size + b);
			double lxo_offset = ( ln_dp / 1024.0 / (samples_size * dt)) / pll_mult;
			*/

			// Compute increment value
			int increment = (int)(xo_offset / frange * 0xFFFFFF);
			vco_ctrl += increment;
			default_tx[0] = (0x3000 + ((vco_ctrl>>12)&0xFFF)) & 0xFFFF; // DAC A
			default_tx[1] = (0xB000 + (vco_ctrl&0xFFF)) & 0xFFFF;		// DAC B
			
			//printf("M: %ld CW: %6.2fHz XO: %4.2fHz CP: %d XO CTRL: %04x %04x %ld.%09lu\n", (sum_mag/num_samples), cw_offset, xo_offset, cumulative_phase, default_tx[0], default_tx[1], prev_time.tv_sec, prev_time.tv_nsec);
			printf("%d %ld %6.2f %4.2f %d %ld.%09lu\n", j, (sum_mag/num_samples), cw_offset, xo_offset, cumulative_phase, prev_time.tv_sec, prev_time.tv_nsec);
			//printf("M: %ld CP: %ld LPD: %d CCW: %6.1fHz CXO: %4.4fHz LXO: %4.4fHz y = %fx + %f \n", (sum_mag/num_samples), cumulative_phase, ln_dp, cw_offset, xo_offset, lxo_offset, a, b);
			//if(j < num_samples-1)
			//	xo_transfer(xo_fd, default_tx, default_rx, sizeof(default_tx));

			clock_gettime(CLOCK_REALTIME, &prev_time);

			/* Disable PRU*/
			prussdrv_pru_disable(PRU_NUM);

			// Stats
			sum_xo_offset += xo_offset;
			if (j == 0)
			{
				min_xo_offset = xo_offset;
				max_xo_offset = xo_offset;
			}
			if (min_xo_offset > xo_offset) 
				min_xo_offset = xo_offset;
			if (max_xo_offset < xo_offset)
				max_xo_offset = xo_offset;
		}

		//printf("Average: %4.9f Min: %4.9f Max: %4.9f\n", (sum_xo_offset/num_samples), min_xo_offset, max_xo_offset);

		/* Exit PRU */
		prussdrv_exit();

		///////// END PRU /////////

	}
	else {
		cc1200_cmd_strobe(CC1200_STX);
	}

	close(fd);               //close the file
	return 0;
}
