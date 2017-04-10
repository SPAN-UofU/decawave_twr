/*
 * CC1200 Driver
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

#include "cc1200-rf-cfg.h"
#include "cc1200-const.h"

/*
 * This is a setup for the following configuration:
 *
 * 802.15.4g
 * =========
 * Table 68f: Frequency band identifier 1 (450–470 MHz)
 * Table 68g: Modulation scheme identifier 0 (Filtered FSK)
 * Table 68h: Mode #1 (50kbps)
 */

/* Base frequency in kHz */
#define RF_CFG_CHAN_CENTER_F0           450006.25
/* Channel spacing in kHz */
#define RF_CFG_CHAN_SPACING             12.5
/* The minimum channel */
#define RF_CFG_MIN_CHANNEL              0
/* The maximum channel */
#define RF_CFG_MAX_CHANNEL              1598
/* The maximum output power in dBm */
#define RF_CFG_MAX_TXPOWER              CC1200_CONST_TX_POWER_MAX
/* The carrier sense level used for CCA in dBm */
#define RF_CFG_CCA_THRESHOLD            (-91)

/*---------------------------------------------------------------------------*/
/* 
 * Register settings exported from SmartRF Studio using the standard template
 * "trxEB RF Settings Performance Line".
 */

/*
// Device address = 0 
// Bit rate = 50 
// Whitening = false 
// Carrier frequency = 433.999939 
// Address config = No address check 
// RX filter BW = 13.888889
// Packet length mode = Variable 
// Deviation = 24.948120 
// Packet length = 255 
// Symbol rate = 50 
// Packet bit length = 0 
// Manchester enable = false 
// Modulation format = 2-GFSK 

static const registerSetting_t preferredSettings[]= 
{
  {CC1200_IOCFG3,            0x28}, //MAGN_VALID
//  {CC1200_IOCFG2,            0x2D}, //AGC_UPDATE
//  {CC1200_IOCFG1,            0x2E}, //ADC_CLOCK
  {CC1200_IOCFG0,            0x06}, //PKT_SYNC_RXTX
  {CC1200_SYNC3,             0x6F},
  {CC1200_SYNC2,             0x4E},
  {CC1200_SYNC1,             0x90},
  {CC1200_SYNC0,             0x4E},
  {CC1200_SYNC_CFG1,         0xE5},
  //{CC1200_SYNC_CFG0,         0x23},
  {CC1200_SYNC_CFG0,         0x33},
  {CC1200_DEVIATION_M,       0x47},
  {CC1200_MODCFG_DEV_E,      0x0B},
  {CC1200_DCFILT_CFG,        0x56}, // 138.888kHz IF
  //{CC1200_DCFILT_CFG,        0x16}, // zero-if
  {CC1200_PREAMBLE_CFG1,     0x00},
  {CC1200_PREAMBLE_CFG0,     0xBA},
  {CC1200_IQIC,              0xC8},
  //{CC1200_CHAN_BW,           0x84},
  {CC1200_CHAN_BW,           0xA5}, //45.0444kHz
  {CC1200_MDMCFG1,           0x06},
  {CC1200_MDMCFG0,           0x05},
  //{CC1200_SYMBOL_RATE2,      0x94},
  //{CC1200_SYMBOL_RATE1,      0x7A},
  //{CC1200_SYMBOL_RATE0,      0xE1},
  {CC1200_SYMBOL_RATE2,      0x72}, // bit rate = 11ksps
  {CC1200_SYMBOL_RATE1,      0x06}, // symbol rate = 11ksps
  {CC1200_SYMBOL_RATE0,      0xBC}, // RX filter BW = 11.261261kHz
  {CC1200_AGC_REF,           0x27},
  {CC1200_AGC_CS_THR,        0xF1},
  {CC1200_AGC_GAIN_ADJUST,   0x00},
  {CC1200_AGC_CFG2,          0x20}, // 138.888kHz IF
  //{CC1200_AGC_CFG2,          0x60}, // zero-if
  {CC1200_AGC_CFG1,          0x11},
  {CC1200_AGC_CFG0,          0x90},
  {CC1200_FIFO_CFG,          0x00},
  {CC1200_FS_CFG,            0x14},
  {CC1200_PKT_CFG2,          0x05},
  {CC1200_PKT_CFG0,          0x20},
  {CC1200_PA_CFG1,           0x3F},
  {CC1200_PKT_LEN,           0xFF},
  //{CC1200_IF_MIX_CFG,        0x00}, // zero-if
  //{CC1200_IF_MIX_CFG,        0x04}, // -208.33kHz IF
  //{CC1200_IF_MIX_CFG,        0x08}, // -138.89kHz IF
  //{CC1200_IF_MIX_CFG,        0x0C}, // -104.17kHz IF
  //{CC1200_IF_MIX_CFG,        0x10}, // zero-if
  //{CC1200_IF_MIX_CFG,        0x14}, // 208.33kHz IF
  {CC1200_IF_MIX_CFG,        0x18}, // 138.888kHz IF
  //{CC1200_IF_MIX_CFG,        0x1C}, // 104.17kHz IF
  {CC1200_TOC_CFG,           0x03},
  {CC1200_MDMCFG2,           0x03},
  {CC1200_FREQ2,             0x56},
  {CC1200_FREQ1,             0xCC},
  {CC1200_FREQ0,             0xCC},
  {CC1200_IF_ADC1,           0xEE},
  {CC1200_IF_ADC0,           0x10},
  {CC1200_FS_DIG1,           0x04},
  {CC1200_FS_DIG0,           0x50},
  {CC1200_FS_CAL1,           0x40},
  {CC1200_FS_CAL0,           0x0E},
  {CC1200_FS_DIVTWO,         0x03},
  {CC1200_FS_DSM0,           0x33},
  {CC1200_FS_DVC1,           0xF7},
  {CC1200_FS_DVC0,           0x0F},
  {CC1200_FS_PFD,            0x00},
  {CC1200_FS_PRE,            0x6E},
  {CC1200_FS_REG_DIV_CML,    0x1C},
  {CC1200_FS_SPARE,          0xAC},
  {CC1200_FS_VCO0,           0xB5},
  {CC1200_IFAMP,             0x05},
  {CC1200_XOSC5,             0x0E},
  {CC1200_XOSC1,             0x03},
  {CC1200_SERIAL_STATUS,     0x08},
};
*/

// Address Config = No address check 
// Bit Rate = 10 
// Carrier Frequency = 433.999969 
// Deviation = 24.902344 
// Device Address = 0 
// Manchester Enable = false 
// Modulation Format = 2-GFSK 
// Packet Bit Length = 0 
// Packet Length = 255 
// Packet Length Mode = Variable 
// RX Filter BW = 10.810811 
// Symbol rate = 10 
// Whitening = false 

static const registerSetting_t preferredSettings_cw[]= 
{
  {CC1200_IOCFG3,            0x28}, //MAGN_VALID
  {CC1200_IOCFG0,            0x06}, //PKT_SYNC_RXTX
  {CC1200_SYNC3,             0x6F},
  {CC1200_SYNC2,             0x4E},
  {CC1200_SYNC1,             0x90},
  {CC1200_SYNC0,             0x4E},
  {CC1200_SYNC_CFG1,         0xE5},
  {CC1200_SYNC_CFG0,         0x33},
  {CC1200_DEVIATION_M,       0x54},
  {CC1200_MODCFG_DEV_E,      0x0B},
  {CC1200_DCFILT_CFG,        0x56},
  {CC1200_PREAMBLE_CFG1,     0x00},
  {CC1200_PREAMBLE_CFG0,     0xBA},
  {CC1200_IQIC,              0xC8},
  {CC1200_CHAN_BW,           0xA5},
  {CC1200_MDMCFG1,           0x06},
  {CC1200_MDMCFG0,           0x05},
  {CC1200_SYMBOL_RATE2,      0x71},
  {CC1200_SYMBOL_RATE1,      0x11},
  {CC1200_SYMBOL_RATE0,      0x11},
  {CC1200_AGC_REF,           0x42},
  {CC1200_AGC_CS_THR,        0xF1},
  {CC1200_AGC_CFG1,          0x11},
  {CC1200_AGC_CFG0,          0x90},
  {CC1200_FIFO_CFG,          0x00},
  {CC1200_FS_CFG,            0x14},
  {CC1200_PKT_CFG2,          0x05},
  {CC1200_PKT_CFG0,          0x20},
  {CC1200_PA_CFG1,           0x3F},
  {CC1200_PA_CFG0,           0x55},
  {CC1200_PKT_LEN,           0xFF},
  {CC1200_IF_MIX_CFG,        0x18},
  {CC1200_TOC_CFG,           0x03},
  {CC1200_MDMCFG2,           0x03},
  {CC1200_FREQ2,             0x5A},
  {CC1200_FREQ1,             0x6A},
  {CC1200_FREQ0,             0xAA},
  {CC1200_IF_ADC1,           0xEE},
  {CC1200_IF_ADC0,           0x10},
  {CC1200_FS_DIG1,           0x04},
  {CC1200_FS_DIG0,           0x50},
  {CC1200_FS_CAL1,           0x40},
  {CC1200_FS_CAL0,           0x0E},
  {CC1200_FS_DIVTWO,         0x03},
  {CC1200_FS_DSM0,           0x33},
  {CC1200_FS_DVC1,           0xF7},
  {CC1200_FS_DVC0,           0x0F},
  {CC1200_FS_PFD,            0x00},
  {CC1200_FS_PRE,            0x6E},
  {CC1200_FS_REG_DIV_CML,    0x1C},
  {CC1200_FS_SPARE,          0xAC},
  {CC1200_FS_VCO0,           0xB5},
  {CC1200_IFAMP,             0x05},
  {CC1200_XOSC5,             0x0E},
  {CC1200_XOSC1,             0x03},
  {CC1200_SERIAL_STATUS,     0x08},
};
/*---------------------------------------------------------------------------*/
/* Global linkage: symbol name must be different in each exported file! */
const cc1200_rf_cfg_t cc1200_802154g_434mhz_2gfsk_50kbps_cw = {
  .register_settings = preferredSettings_cw,
  .size_of_register_settings = sizeof(preferredSettings_cw),
  .chan_center_freq0 = RF_CFG_CHAN_CENTER_F0,
  .chan_spacing = RF_CFG_CHAN_SPACING,
  .min_channel = RF_CFG_MIN_CHANNEL,
  .max_channel = RF_CFG_MAX_CHANNEL,
  .max_txpower = RF_CFG_MAX_TXPOWER,
  .cca_threshold = RF_CFG_CCA_THRESHOLD,
};
/*---------------------------------------------------------------------------*/
