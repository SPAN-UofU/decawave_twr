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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "platform.h"

#define SPI_PATH 	"/dev/spidev2.0"

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
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
#define TX_DELAY_MS 1000

/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). See NOTE 2 below. */
#define TX_TO_RX_DELAY_UUS 60

/* Receive response timeout, expressed in UWB microseconds. See NOTE 3 below. */
#define RX_RESP_TO_UUS 5000

/* Buffer to store received frame. See NOTE 4 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_sync_buffer[FRAME_LEN_MAX];
static uint8 rx_ref_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

/* Hold copies of timestamps */
typedef unsigned long long uint64;
static uint64 t_tx1_ts; /* time when sync node transmits (in dtu) */
static uint64 t_rx1_ts; /* system counter when sync node transmits */
static uint64 t_tx2_ts; /* time when ref node receives (in dtu) */
static uint64 t_rx2_ts; /* system counter when sync node receives */
static uint64 t_tx1_stc; /* time when sync node receives (in dtu) */
static uint64 t_rx1_stc; /* system counter when sync node receives */
static uint64 t_tx2_stc; /* time when sync node transmits (in dtu) */
static uint64 t_rx2_stc; /* system counter when sync node transmits */

/* Data in CC1200 packet */
static uint64 my_delta_ts; /* Diff between T_tx2 and T_rx2 timestamps (dtu) */
static uint64 my_delta_stc; /* Diff between T_tx2 and T_rx2 system counter (dtu) */

/* Declaration of static functions */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static uint64 get_tx_syscount_u64(void);
static uint64 get_rx_syscount_u64(void);
static uint64 compute_offset(uint64 t_rx2, uint64 t_tx1, uint64 d);
static uint64 compute_prop_delay(uint64 t_tx1, uint64 t_rx1, uint64 d);

/**
 * Application entry point.
 */
int main(int argc, char* argv[])
{
    /* Start with board specific hardware init. */
    hardware_init(SPI_PATH);

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
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    // Run SYNC program
    if(atoi(argv[3]))
    {
    	/* Set delay to turn reception on after transmission of the frame. See NOTE 2 below. */
    	/* Set response frame timeout. */
    	dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);    
    	dwt_setrxtimeout(RX_RESP_TO_UUS);

	    /* Loop forever sending and receiving frames periodically. */
	    while (1)
	    {
	        /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
	        dwt_writetxdata(sizeof(sync_msg), sync_msg, 0); /* Zero offset in TX buffer. */
	        dwt_writetxfctrl(sizeof(sync_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

	        /* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
	        if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) == DWT_ERROR)
	        {
	            /* Print a row of zeros */
	            printf("0 0 0 0\n");
	            continue;
	        }

	        /* Poll DW1000 until TX frame sent event set. */
	        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
	        { };

	        /* Get the transmitted timestamp and the system counter and print to console */
	        t_tx1_ts = get_tx_timestamp_u64();
	        t_tx1_stc = get_tx_syscount_u64();

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
	            printf("%lld %lld %lld %lld\n", t_tx1_ts, t_tx1_stc, t_rx1_ts, t_rx1_stc);

	            /* Clear good RX frame event in the DW1000 status register. */
	            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
	        }
	        // We did not get a response
	        else
	        {
	        	/* Execute missed message routine */
	        	missed_msg_routine();

	            /* Print a row of zeros */
	            printf("0 0 0 0\n");

	            /* Clear RX error/timeout events in the DW1000 status register. */
	            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

	            /* Reset RX to properly reinitialise LDE operation. */
	            dwt_rxreset();
	        }

	        /* Poll for message from CC1200 that contains detla and rx timestamp */
	        // while(!message_received())
	        // { };
	        //
	        // my_delta_ts = unpack_packet(); // this is delta in our diagram
	        // my_delta_stc = unpack_packet(); // this is delta in our diagram
	        // t_rx2_ts = unpack_packet(); // T_rx2 in our diagram
	        // t_rx2_stc = unpack_packet(); // T_rx2 in our diagram

	        /* Compute time-sync parameters delta and phi */
	        // Delta_ts = compute_prop_delay(t_tx1_ts,t_rx1_ts,my_delta_ts);
	        // Delta_stc = compute_prop_delay(t_tx1_stc,t_rx1_stc,my_delta_stc);
	        // phi_ts = compute_offset(t_rx2_ts,t_tx1_ts,my_delta_ts);
	        // phi_stc = compute_offset(t_rx2_stc,t_tx1_stc,my_delta_stc);

	        // WRITE CODE HERE

	        /* Execute a delay between transmissions. */
	        sleep_ms(TX_DELAY_MS);

	        /* Increment the blink frame sequence number (modulo 256). */
	        sync_msg[BLINK_FRAME_SN_IDX]++;
	    }

    } // End of SYNC program

    // Run REF program
    else
    {
    	/* Loop forever sending and receiving frames periodically. */
	    while (1)
	    {
	        /* Activate reception immediately. See NOTE 4 below. */
	        dwt_rxenable(DWT_START_RX_IMMEDIATE);

	        /* Poll until a frame is properly received or an error occurs. See NOTE 5 below.
	         * STATUS register is 5 bytes long but, as the events we are looking at are in the lower bytes of the register, we can use this simplest API
	         * function to access it. */
	        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
	        { };

	        if (status_reg & SYS_STATUS_RXFCG)
	        {
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
	            if ((frame_len == 14) && (rx_buffer[0] == 0xC5) && (rx_buffer[10] == 0x43) && (rx_buffer[11] == 0x2))
	            {
	                // int i;

	                // /* Copy source address of blink in response destination address. */
	                // for (i = 0; i < 8; i++)
	                // {
	                //     ref_msg[DATA_FRAME_DEST_IDX + i] = rx_buffer[BLINK_FRAME_SRC_IDX + i];
	                // }

	                /* Write response frame data to DW1000 and prepare transmission. See NOTE 6 below.*/
	                dwt_writetxdata(sizeof(ref_msg), ref_msg, 0); /* Zero offset in TX buffer. */
	                dwt_writetxfctrl(sizeof(ref_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

	                /* Send the response. */
	                dwt_starttx(DWT_START_TX_IMMEDIATE);

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
	            }
	            // We received a DW message carrying a different payload than we anticipated
	            else
	            {
	                /* Print a row of zeros */
	                printf("0 0 0 0\n");
	            }
	        }
	        else
	        {
	            /* Print a row of zeros */
	            printf("0 0 0 0\n");

	            /* Clear RX error events in the DW1000 status register. */
	            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

	            /* Reset RX to properly reinitialise LDE operation. */
	            dwt_rxreset();
	        }

	        /* compute the time elapsed between reception and transmission (delta) */
	        my_delta_ts = t_tx2_ts - t_rx2_ts; // this is delta in our diagram
	        my_delta_stc = t_tx2_stc - t_rx2_stc; // this is delta in our diagram

        	// WRITE CODE HERE
        	// send out my_delta_ts, my_delta_stc, t_rx2_ts, and t_rx2_stc

    } // end REF program

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
static uint64 compute_offset(uint64 t_rx2, uint64 t_tx1, uint64 d)
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
static uint64 compute_prop_delay(uint64 t_tx1, uint64 t_rx1, uint64 d)
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
static uint64 get_tx_timestamp_u64(void)
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
static uint64 get_rx_timestamp_u64(void)
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
static uint64 get_tx_syscount_u64(void)
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
static uint64 get_rx_syscount_u64(void)
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
