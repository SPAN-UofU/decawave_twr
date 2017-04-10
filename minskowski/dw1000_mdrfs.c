/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   RX then send a response example code
 *
 *           This is a simple code example that turns on the DW1000 receiver to receive a frame, (expecting the frame as sent by the companion simple
 *           example "TX then wait for response example code"). When a frame is received and validated as the expected frame a response message is
 *           sent, after which the code returns to await reception of another frame.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * Written by:
 * Peter Hillyard <peterhillyard@gmail.com>
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
#include <signal.h>
#include <math.h>

// DW1000
#include "deca_device_api.h"
#include "deca_regs.h"
#include "platform.h"

// CC1200
#include "cc1200-const.h"
#include "cc1200-rf-cfg.h"
#include "cc1200-802154g-434mhz-2gfsk-50kbps.h"
#include "cc1200-802154g-434mhz-2gfsk-50kbps-cw.h"
#include "cc1200.h"

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

#define DW1000_PATH 	"/dev/spidev1.0"
#define CC1200_PATH 	"/dev/spidev2.0"
#define XO_PATH 		"/dev/spidev2.1"

/////////////// PRU ///////////////
#define PRU_NUM 	0
#define OFFSET_SHAREDRAM 0		//equivalent with 0x00002000
#define PRUSS0_SHARED_DATARAM    4

struct IQSample {
	uint8_t status0;
	uint8_t status1;
	uint8_t magn2;
	uint8_t magn1;
	uint8_t magn0;
	uint8_t ang1;
	uint8_t ang0;
} iqsample;

static void *sharedMem;
static unsigned int *sharedMem_int;
static struct IQSample *sharedMem_struct;

static int num_cycles = 100;
static int num_samples = 1000;

/////////////// DW1000 ///////////////
/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_16M,//DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,//DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC8,//DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    3,               /* TX preamble code. Used in TX only. */
    3,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,//DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (128 + 1 + 8 - 8)//(1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* The frame sent in this example is a blink encoded as per the ISO/IEC 24730-62:2013 standard. It is a 14-byte frame composed of the following fields:
 *     - byte 0: frame control (0xC5 to indicate a multipurpose frame using 64-bit addressing).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10: encoding header (0x43 to indicate no extended ID, temperature, or battery status is carried in the message).
 *     - byte 11: EXT header (0x02 to indicate tag is listening for a response immediately after this message).
 *     - byte 12/13: frame check-sum, automatically set by DW1000. */
static uint8 sync_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0x43, 0x02, 0, 0};
static uint8 ref_msg[] = {0x41, 0x8C, 0, 0x9A, 0x60, 0, 0, 0, 0, 0, 0, 0, 0, 'D', 'W', 0x10, 0x00, 0, 0, 0, 0};

/* Index to access to sequence number of the blink frame in the sync_msg and ref_msg array. */
#define BLINK_FRAME_SN_IDX 1
#define BLINK_FRAME_SRC_IDX 2

/* Indexes to access to sequence number and destination address of the data frame in the ref_msg array. */
#define DATA_FRAME_SN_IDX 2
#define DATA_FRAME_DEST_IDX 5

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 100//0

/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). See NOTE 2 below. */
#define TX_TO_RX_DELAY_UUS 60

/* Receive response timeout, expressed in UWB microseconds. See NOTE 3 below. */
#define RX_RESP_TO_UUS 5000

/* Buffer to store received frame. See NOTE 4 below. */
#define FRAME_LEN_MAX 127
//static uint8 rx_sync_buffer[FRAME_LEN_MAX];
static uint8 rx_ref_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

/* Hold copies of timestamps */
typedef unsigned long long uint64;
typedef int64_t int64;
static uint64 t_tx1_ts; /* time when sync node transmits (in dtu) */
static uint64 t_rx1_ts; /* system counter when sync node transmits */
static uint64 t_tx2_ts; /* time when ref node receives (in dtu) */
static uint64 t_rx2_ts; /* system counter when sync node receives */
static uint64 t_tx1_stc; /* time when sync node receives (in dtu) */
static uint64 t_rx1_stc; /* system counter when sync node receives */
static uint64 t_tx2_stc; /* time when sync node transmits (in dtu) */
static uint64 t_rx2_stc; /* system counter when sync node transmits */

/////////////// CC1200 ///////////////
extern const cc1200_rf_cfg_t CC1200_RF_CW_CFG;
#define CC1200_RF_CW_CFG cc1200_802154g_434mhz_2gfsk_50kbps_cw

extern const cc1200_rf_cfg_t CC1200_RF_CFG;
#define CC1200_RF_CFG cc1200_802154g_434mhz_2gfsk_50kbps

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

typedef struct {
	uint8  phr;
	uint8  len;
	uint64 t_rx2_ts;
	uint64 t_rx2_stc;
	uint64 my_delta_ts; /* Diff between T_tx2 and T_rx2 timestamps (dtu) */
	uint64 my_delta_stc; /* Diff between T_tx2 and T_rx2 system counter (dtu) */
}cc1200_msg_t;

cc1200_msg_t cc1200_msg = {
	0x18,
	sizeof(cc1200_msg_t),
	0xDEAD,
	0xBEEF
};

#define N_SAMPLES 200//2000

//static uint8_t tx_msg[] = {0x18, 0, 0, 'T', 'I', 'C', 'C', '1', '2', '0', '0', 'A', 'L'};
//static uint8_t rx_msg[ARRAY_SIZE(tx_msg)] = {0, };

/* Data in CC1200 packet */

/////////////// XO ///////////////
static int xo_fd;

static uint32_t xo_mode = 0;
static uint8_t xo_bits = 16;
static uint32_t xo_speed = 20000000;
static uint16_t xo_delay = 1;

static uint16_t default_tx[] = {0, 0};
static uint16_t default_rx[] = {0, 0};

static int vco_ctrl = 0x7FFFFF;

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

static int xo_init(char * path)
{
	// The following calls set up the XO SPI bus properties
	if((xo_fd = open(path, O_RDWR))<0){
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

	default_tx[0] = (0x3000 + ((vco_ctrl>>12)&0xFFF)) & 0xFFFF; // DAC A
	default_tx[1] = (0xB000 + (vco_ctrl&0xFFF)) & 0xFFFF;		// DAC B
	xo_transfer(xo_fd, default_tx, default_rx, sizeof(default_tx));

	return 0;
}

/* Declaration of static functions */
uint64 get_tx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);
uint64 get_tx_syscount_u64(void);
uint64 get_rx_syscount_u64(void);
uint64 compute_offset(uint64 t_rx2, uint64 t_tx1, uint64 d);
uint64 compute_prop_delay(uint64 t_tx1, uint64 t_rx1, uint64 d);
double dtu_2_s(uint64 d);
double compare (const void * a, const void * b);

// Interrupt
static volatile int keepRunning = N_SAMPLES;

void intHandler(int dummy) {
	keepRunning = 0;
}

/**
 * Application entry point.
 */
int main(int argc, char* argv[])
{
	double tof_array[N_SAMPLES] = {0};
	int tof_counter = 0;
	uint8_t isREF = 0;
	uint16_t ant_delay = 16486;//16463;//0;
	
	if(argc != 2)
	{
		printf("usage: %s REF/SYNC\n", argv[0]);
		return 0;
	}
	else if(argc == 2)
	{
		isREF = atoi(argv[1]);
		//ant_delay = (uint16_t) atoi(argv[2]);
	}

	// Register exit
	signal(SIGINT, intHandler);

	// Initialize XO
	xo_init(XO_PATH);

    /* Start with board specific hardware init. */
    hardware_init(DW1000_PATH);

    /* Reset and initialise DW1000. See NOTE 5 below.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        printf("%s\n", "INIT FAILED");
        while (1)
        { };
    }
    spi_set_rate_high();

    /* Configure DW1000. See NOTE 6 below. */
    dwt_configure(&config);

    /* Apply default antenna delay value.*/
    dwt_setrxantennadelay(ant_delay);
    dwt_settxantennadelay(ant_delay);

    // Init CC1200
    cc1200_init(CC1200_PATH);

	// Write registers to radio
	cc1200_write_reg_settings(CC1200_RF_CFG.register_settings, CC1200_RF_CFG.size_of_register_settings);

	uint8_t rxbytes;
	uint8_t status;

	/* Testing Purpose */
	//uint32_t samples = 0;
	double epsilon_dt = 0;

    // Run SYNC program
    if(!isREF)
    {
    	/* Set delay to turn reception on after transmission of the frame. See NOTE 2 below. */
    	/* Set response frame timeout. */
    	dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);    
    	dwt_setrxtimeout(RX_RESP_TO_UUS);

	    /* Loop forever sending and receiving frames periodically. */
	    //while (1)
	    /* Testing Purpose */
	    //while (samples < N_SAMPLES)
	    while(keepRunning != 0)
	    {	
	    	// Reset Radio
			cc1200_cmd_strobe(CC1200_SRES);

	    	// Write registers to radio
			cc1200_write_reg_settings(CC1200_RF_CW_CFG.register_settings, CC1200_RF_CW_CFG.size_of_register_settings);

	    	// RX
			cc1200_cmd_strobe(CC1200_SRX);

			///////// START PRU /////////
			unsigned int ret;
			tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
			
			//Initializing PRU
			prussdrv_init();
			ret = prussdrv_open(PRU_EVTOUT_0);
			if (ret){
				printf("\tERROR: prussdrv_open open failed\n");
				return (ret);
			}

			prussdrv_pruintc_init(&pruss_intc_initdata);
			prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, &sharedMem);

			sharedMem_int = (unsigned int *) sharedMem;
			sharedMem_int[OFFSET_SHAREDRAM + 0] = num_samples; //samples

			/* Executing PRU. */
			prussdrv_pru_write_memory(PRUSS0_PRU0_IRAM, PRU_NUM, PRUcode, sizeof(PRUcode));

			float fs = 45044.4;
			float dt = 1.0 / fs;
			float pll_mult = 433.999939 / 38.4;
			float frange = 800.0;

			int i, j = 0;
			for(j = 0;j < num_cycles; j++)
			{
				if(num_samples%2 == 0)
				{
					num_samples++;
				}

				float nvar_x = (num_samples*num_samples - 1) * num_samples / 12.0;

				prussdrv_pruintc_init(&pruss_intc_initdata);
				sharedMem_int[OFFSET_SHAREDRAM + 0] = num_samples; //samples
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
				
				float sum_xy = 0.0;
				int x_i = (-(num_samples-1)) / 2;

				for(i = 0; i < num_samples; i++) {
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

						sum_xy += cumulative_phase * x_i;
						x_i++;
					}

				}

				double beta = sum_xy / (nvar_x); //dib
				cw_offset = (beta / 1024.0 / dt);
				xo_offset = cw_offset / pll_mult;
				xo_offset += (2.728371123 - 0.337881717433);

				if(j > 10)
				{
					// Compute increment value	
					int increment = (int)(xo_offset / frange * 0xFFFFFF);
					vco_ctrl += increment;

					default_tx[0] = (0x3000 + ((vco_ctrl>>12)&0xFFF)) & 0xFFFF; // DAC A
					default_tx[1] = (0xB000 + (vco_ctrl&0xFFF)) & 0xFFFF;		// DAC B
				
					xo_transfer(xo_fd, default_tx, default_rx, sizeof(default_tx));
				}

				//printf("%d %ld %4.2f %4.9f %d %ld.%09lu\n", j, (sum_mag/num_cycles), cw_offset, xo_offset, cumulative_phase, prev_time.tv_sec, prev_time.tv_nsec);
				//printf("%d %ld %4.2f %4.9f %d\n", j, (sum_mag/num_cycles), cw_offset, xo_offset, cumulative_phase);
			
				// Disable PRU
				prussdrv_pru_disable(PRU_NUM);

			}

			// Exit PRU
			prussdrv_exit();
			
			//printf("DONE SYNC TX MSG! ");

	        /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
	        dwt_writetxdata(sizeof(sync_msg), sync_msg, 0); /* Zero offset in TX buffer. */
	        dwt_writetxfctrl(sizeof(sync_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

	        /* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
	        if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) == DWT_ERROR)
	        {
	            /* Print a row of zeros */
	            printf("0 0 0 0 TX ERROR\n");
	            continue;
	        }

	        /* Poll DW1000 until TX frame sent event set. */
	        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
	        { };

	        /* Get the transmitted timestamp and the system counter and print to console */
	        t_tx1_ts = get_tx_timestamp_u64();
	        t_tx1_stc = get_tx_syscount_u64();

	        // Stop CW and reconfigure radio for data
	        cc1200_cmd_strobe(CC1200_SRES);
	        cc1200_write_reg_settings(CC1200_RF_CFG.register_settings, CC1200_RF_CFG.size_of_register_settings);
	        cc1200_cmd_strobe(CC1200_SRX);

	        /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
	        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	        { };

	        /* Prepare to wait for response */
	        if (status_reg & SYS_STATUS_RXFCG)
	        {
	            // int i;

	            // /* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading
	            //  * the RX buffer. */
	            // for (i = 0 ; i < FRAME_LEN_MAX; i++ )
	            // {
	            //     rx_sync_buffer[i] = 0;
	            // }

	            // /* A frame has been received, copy it to our local buffer. */
	            // frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
	            // if (frame_len <= FRAME_LEN_MAX)
	            // {
	            //     dwt_readrxdata(rx_sync_buffer, frame_len, 0);
	            // }

	            /* Get the received timestamp and the system counter and print to console */
	            t_rx1_ts = get_rx_timestamp_u64();
	            t_rx1_stc = get_rx_syscount_u64();
	            //printf("%lld %lld %lld %lld\n", t_tx1_ts, t_tx1_stc, t_rx1_ts, t_rx1_stc);

	            /* Clear good RX frame event in the DW1000 status register. */
	            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    	        /* Poll for message from CC1200 that contains detla and rx timestamp */
				uint16_t retry = 300;
				uint8_t gotMsg = 0;
				cc1200_msg_t rx_msg = {0, };
				for(retry = 300; retry > 0; retry--)
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
								int rx_fifo_bytes = rxbytes - 2;
								cc1200_read_rxfifo((uint8_t *)&rx_msg, rx_fifo_bytes);

								//printf("MSG Received! DATA: %02x %02x %lld %lld %lld %lld rxbytes: %d ", rx_msg.phr, rx_msg.len, rx_msg.t_rx2_ts, rx_msg.t_rx2_stc, rx_msg.my_delta_ts, rx_msg.my_delta_stc, rxbytes);
								//printf("retry = %d\n", (250-retry));

								//int i;
								//for (i = 0; i < rx_fifo_bytes; i++)
								//	printf("%02x ", rx_msg[i]);
								//printf(" bytes: %d\n", rxbytes);

								cc1200_cmd_strobe(CC1200_SFRX);

								gotMsg = 1;
								break;
							}
						}

						cc1200_cmd_strobe(CC1200_SRX);
					}
					// if(retry % 10 == 0)
					// {
					// 	printf("retry=%d.\n", retry);
					// }
				}

				if(gotMsg)
				{
			        uint64 my_delta_ts = rx_msg.my_delta_ts; // this is delta in our diagram
			        uint64 my_delta_stc = rx_msg.my_delta_stc; // this is delta in our diagram
			        t_rx2_ts = rx_msg.t_rx2_ts; // T_rx2 in our diagram
			        t_rx2_stc = rx_msg.t_rx2_stc; // T_rx2 in our diagram

			        /* Compute time-sync parameters delta and phi */
			        uint64 Delta_ts = compute_prop_delay(t_tx1_ts,t_rx1_ts,my_delta_ts);
			        uint64 Delta_stc = compute_prop_delay(t_tx1_stc,t_rx1_stc,my_delta_stc);
			        //uint64 phi_ts = compute_offset(t_rx2_ts,t_tx1_ts,my_delta_ts);
			        //uint64 phi_stc = compute_offset(t_rx2_stc,t_tx1_stc,my_delta_stc);

			        double tof = dtu_2_s(Delta_ts);
			        
			        //t_tx1_ts
			        //			t_rx2_ts
			        //			t_tx2_ts
			        //t_rx1_ts

			        uint64 t_tx1_ts_p = t_tx1_ts / 2;
			        uint64 t_rx1_ts_p = t_rx1_ts / 2;
			        uint64 t_rx2_ts_p = t_rx2_ts / 2;
			        uint64 t_tx2_ts_p = (t_rx2_ts + my_delta_ts) / 2;

			        if(t_tx1_ts_p > t_rx1_ts_p)
			        {
			        	t_rx1_ts_p += powl(2, 63);
			        }

			        if(t_rx2_ts_p > t_tx2_ts_p)
			        {
			        	t_rx2_ts_p += powl(2, 63);
			        }

			        double epsilon = ((t_rx1_ts_p + t_tx1_ts_p) - (t_tx2_ts_p + t_rx2_ts_p));
			        double epsilon_t = dtu_2_s(epsilon);

			        printf("delta_ts: %lld delta_stc: %lld ", Delta_ts, Delta_stc);
			        printf("offset: %3.9e sec ", tof);
			        printf("range: %4.3f m epsilon: %3.9e\n", tof*299792458.0, epsilon_t-epsilon_dt);

			        epsilon_dt = epsilon_t;

			        
			        if(tof < 10e-9)
			        {
			        	tof_array[tof_counter] = tof;
			        	tof_counter++;

			        	if(tof_counter == N_SAMPLES)
			        	{
			        		qsort (tof_array, N_SAMPLES, sizeof(double), compare);

			        		// tof_array[ELEMENTS/2]

			        		// perform average
			        		double tof_avg = 0.0;
			        		for(int iii = 0; iii < N_SAMPLES; iii++)
			        		{
			        		 	tof_avg += tof_array[iii];
			        		}
			        		tof_avg = tof_avg/(double) N_SAMPLES;
			        		printf("\n\n\n\n%3.9e seconds or %3.9e m median \n %3.9e seconds or %3.9e m avg\n\n\n\n", tof_array[N_SAMPLES/2], tof_array[N_SAMPLES/2]*299792458.0, tof_avg, tof_avg*299792458.0);
			        		return 0;

			        		// reset for next round
			        		tof_counter=0;
			        	}
			        }
					

			       	/* Testing purpose */
		        	//samples++;
		        	keepRunning--;
			    }

		        fflush(stdout);

		        // WRITE CODE HERE

		        /* Execute a delay between transmissions. */
		        sleep_ms(TX_DELAY_MS);

		        /* Increment the blink frame sequence number (modulo 256). */
		        sync_msg[BLINK_FRAME_SN_IDX]++;

	        }
	        // We did not get a response
	        else
	        {
	        	/* Execute missed message routine */
	        	//missed_msg_routine();

	            /* Print a row of zeros */
	            //printf("0 0 0 0 RX ERROR\n");

	            /* Clear RX error/timeout events in the DW1000 status register. */
	            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

	            /* Reset RX to properly reinitialise LDE operation. */
	            dwt_rxreset();
	        }

	    }

    } // End of SYNC program

    // Run REF program
    else
    {
    	/* Loop forever sending and receiving frames periodically. */
	    //while (1)
	    while(keepRunning != 0)
	    {
	    	// Start TX CW
	    	// Reset Radio
			cc1200_cmd_strobe(CC1200_SRES);

	    	// Write registers to radio
			cc1200_write_reg_settings(CC1200_RF_CW_CFG.register_settings, CC1200_RF_CW_CFG.size_of_register_settings);

			// Transmit CW
			cc1200_cmd_strobe(CC1200_STX);

	        /* Activate reception immediately. See NOTE 4 below. */
	        dwt_rxenable(DWT_START_RX_IMMEDIATE);

	        /* Poll until a frame is properly received or an error occurs. See NOTE 5 below.
	         * STATUS register is 5 bytes long but, as the events we are looking at are in the lower bytes of the register, we can use this simplest API
	         * function to access it. */
	        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
	        { };

	        if (status_reg & SYS_STATUS_RXFCG)
	        {
	        	// Stop CW and reconfigure radio for data
	        	cc1200_cmd_strobe(CC1200_SRES);
	        	cc1200_write_reg_settings(CC1200_RF_CFG.register_settings, CC1200_RF_CFG.size_of_register_settings);

	            /* A frame has been received, read it into the local buffer. */
	            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
	            if (frame_len <= FRAME_LEN_MAX)
	            {
	                dwt_readrxdata(rx_ref_buffer, frame_len, 0);
	            }

	            /* Get the RX timestamp and the system counter and print to console*/
	            t_rx2_ts = get_rx_timestamp_u64();
	            t_rx2_stc = get_rx_syscount_u64();

	            /* Clear good RX frame event in the DW1000 status register. */
	            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

	            /* Validate the frame is the one expected as sent by "TX then wait for a response" example. */
	            if ((frame_len == 14) && (rx_ref_buffer[0] == 0xC5) && (rx_ref_buffer[10] == 0x43) && (rx_ref_buffer[11] == 0x2))
	            {
	                // int i;

	                // /* Copy source address of blink in response destination address. */
	                // for (i = 0; i < 8; i++)
	                // {
	                //     ref_msg[DATA_FRAME_DEST_IDX + i] = rx_ref_buffer[BLINK_FRAME_SRC_IDX + i];
	                // }

	                /* Write response frame data to DW1000 and prepare transmission. See NOTE 6 below.*/
	                dwt_writetxdata(sizeof(ref_msg), ref_msg, 0); /* Zero offset in TX buffer. */
	                dwt_writetxfctrl(sizeof(ref_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

	                /* Send the response. */
	                if(dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_ERROR)
	                {
	                	/* Print a row of zeros */
			            printf("0 0 0 0 TX ERROR\n");
			            continue;
	                }

	                /* Poll DW1000 until TX frame sent event set. */
	                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
	                { };

	                /* Get the TX timestamp and the system counter and print to console */
	                t_tx2_ts = get_tx_timestamp_u64();
	                t_tx2_stc = get_tx_syscount_u64();
	                printf("%lld %lld %lld %lld\n", t_rx2_ts, t_rx2_stc, t_tx2_ts, t_tx2_stc);

	                /* Clear TX frame sent event. */
	                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

	                /* Increment the data frame sequence number (modulo 256). */
	                ref_msg[DATA_FRAME_SN_IDX]++;

	                /* compute the time elapsed between reception and transmission (delta) */
	        
		        	// WRITE CODE HERE
		        	// send out my_delta_ts, my_delta_stc, t_rx2_ts, and t_rx2_stc
					//printf("size: %d\n", sizeof(tx_msg));
			        
			        usleep(2000);

					// Write data into FIFO
					//cc1200_write_txfifo((uint8 *)&cc1200_tx_msg, sizeof(cc1200_tx_msg));
			        uint8_t retry = 0;
					for(retry = 0; retry < 3; retry++)
					{
						//tx_msg[1] = sizeof(tx_msg);
						//tx_msg[2]++;

						//int i;
						//for (i = 0; i < sizeof(tx_msg); i++)
						//	printf("%02x ", tx_msg[i]);
						//printf(" size: %d\n", sizeof(tx_msg));

						// Write data into FIFO
						//cc1200_write_txfifo(tx_msg, sizeof(tx_msg));

						cc1200_msg.phr = 0x18;
						cc1200_msg.len = sizeof(cc1200_msg);
						cc1200_msg.t_rx2_ts = t_rx2_ts;
						cc1200_msg.t_rx2_stc = t_rx2_stc;
						if(t_tx2_ts > t_rx2_ts) // no wrapping
						{
							cc1200_msg.my_delta_ts  = t_tx2_ts - t_rx2_ts; // this is delta in our diagram
			        		cc1200_msg.my_delta_stc = t_tx2_stc - t_rx2_stc; // this is delta in our diagram
						}
						else // wrapped
						{
							cc1200_msg.my_delta_ts  = (0 - t_tx2_ts) + t_rx2_ts; // this is delta in our diagram
			        		cc1200_msg.my_delta_stc = (0 - t_tx2_stc) + t_rx2_stc; // this is delta in our diagram	
						}

						printf("%02x %02x %lld %lld %lld %lld size: %d\n", cc1200_msg.phr, cc1200_msg.len, cc1200_msg.t_rx2_ts, cc1200_msg.t_rx2_stc, cc1200_msg.my_delta_ts, cc1200_msg.my_delta_stc, sizeof(cc1200_msg));

						// Write data into FIFO
						cc1200_write_txfifo((uint8_t *)&cc1200_msg, sizeof(cc1200_msg));				

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

						printf("MSG SENT! retry = %d\n", retry);

						cc1200_cmd_strobe(CC1200_SFTX);

						retry = 3;
					}

	            }
	            // We received a DW message carrying a different payload than we anticipated
	            else
	            {
	                /* Print a row of zeros */
	                printf("0 0 0 0 RX FROM INVALID SOURCE\n");
	            }

	        }
	        else
	        {
	            /* Print a row of zeros */
	            printf("0 0 0 0 RX ERROR\n");

	            /* Clear RX error events in the DW1000 status register. */
	            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

	            /* Reset RX to properly reinitialise LDE operation. */
	            dwt_rxreset();
	        }

	    }

    } // end REF program

    cc1200_cmd_strobe(CC1200_SRES);
	close(xo_fd);               //close the file
	hardware_close();
	cc1200_close();
}  // end main

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn compute_offset()
 *
 * @brief Get the offset (phi) between the sync and ref node.
 * 
 * Input params
 * @param  t_rx2 - timestamp when ref gets first message
 * @param  t_tx1 - timestamp when sync sends first message
 * @param  d     - prop. delay between ref and sync
 *
 *
 * @return  64-bit value offset.
 */
uint64 compute_offset(uint64 t_rx2, uint64 t_tx1, uint64 d)
{
    /* 32 subtractions give correct differences */
    uint32 t_rx2_32 = (uint32) t_rx2;
    uint32 t_tx1_32 = (uint32) t_tx1;
    uint32 d_32     = (uint32) d;

    return (uint64)(t_rx2_32 - t_tx1_32 - d_32);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn compute_prop_delay()
 *
 * @brief Get the propagation delay (Delta) between the sync and ref node.
 *
 * Input params
 * @param  t_tx1 - timestamp when sync sends first message
 * @param  t_rx1 - timestamp when sync receives response
 * @param  d     - difference between receive and send timestamps on the ref node
 *
 * @return  64-bit value propagation delay.
 */
uint64 compute_prop_delay(uint64 t_tx1, uint64 t_rx1, uint64 d)
{
    return (t_rx1 - t_tx1 - d)/2;
}



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_syscount_u64()
 *
 * @brief Get the TX system counter in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read system counter.
 */
uint64 get_tx_syscount_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtx_sys_count(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_syscount_u64()
 *
 * @brief Get the RX system counter in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read system counter.
 */
uint64 get_rx_syscount_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrx_sys_count(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

double dtu_2_s(uint64 d)
{
	return (double)d*(1.0/499.2e6/128.0);
}

double compare (const void * a, const void * b)
{
  return ( *(double*)a - *(double*)b );
}
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The device ID is a hard coded constant in the blink to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW1000 unique ID by combining the Lot ID & Part Number values programmed into the
 *    DW1000 during its manufacture. However there is no guarantee this will not conflict with someone else's implementation. We recommended that
 *    customers buy a block of addresses from the IEEE Registration Authority for their production items. See "EUI" in the DW1000 User Manual.
 * 2. TX to RX delay can be set to 0 to activate reception immediately after transmission. But, on the responder side, it takes time to process the
 *    received frame and generate the response (this has been measured experimentally to be around 70 µs). Using an RX to TX delay slightly less than
 *    this minimum turn-around time allows the application to make the communication efficient while reducing power consumption by adjusting the time
 *    spent with the receiver activated.
 * 3. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive a complete frame sent by the "RX then send a response"
 *    example at the 110k data rate used (around 3 ms).
 * 4. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW1000 supports an extended frame
 *    length (up to 1023 bytes long) mode which is not used in this example.
 * 5. In this example, LDE microcode is not loaded upon calling dwt_initialise(). This will prevent the IC from generating an RX timestamp. If
 *    time-stamping is required, DWT_LOADUCODE parameter should be used. See two-way ranging examples (e.g. examples 5a/5b).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 7. dwt_writetxdata() takes the full size of sync_msg as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our sync_msg could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts".
 * 9. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *    DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
