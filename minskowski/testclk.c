/*
 * Chronos Test XO Control Application
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
#include <termios.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
   perror(s);
   abort();
}

static const char *device = "/dev/spidev2.1";
static uint32_t mode = 0;
static uint8_t bits = 16;
static uint32_t speed = 20000000;
static uint16_t delay = 1;
static int verbose = 1;

uint16_t default_tx[] = {0x37FF, 0xBFFF};

uint16_t default_rx[ARRAY_SIZE(default_tx)] = {0, };

static void hex_dump(const void *src, size_t length, size_t line_size, char *prefix)
{
   int i = 0;
   const unsigned char *address = src;
   const unsigned char *line = address;
   unsigned char c;

   printf("%s | ", prefix);
   while (length-- > 0) {
      printf("%02X ", *address++);
      if (!(++i % line_size) || (length == 0 && i % line_size)) {
         if (length == 0) {
            while (i++ % line_size)
               printf("__ ");
         }
         printf(" | ");  /* right close */
         while (line < address) {
            c = *line++;
            printf("%c", (c < 33 || c == 255) ? 0x2E : c);
         }
         printf("\n");
         if (length > 0)
            printf("%s | ", prefix);
      }
   }
}

static void transfer(int fd, uint16_t const *tx, uint16_t const *rx, size_t len)
{
   int ret;

   struct spi_ioc_transfer tr = {
      .tx_buf = (unsigned long)tx,
      .rx_buf = (unsigned long)rx,
      .len = len,
      .delay_usecs = delay,
      .speed_hz = speed,
      .bits_per_word = bits,
   };

   if (mode & SPI_TX_QUAD)
      tr.tx_nbits = 4;
   else if (mode & SPI_TX_DUAL)
      tr.tx_nbits = 2;
   if (mode & SPI_RX_QUAD)
      tr.rx_nbits = 4;
   else if (mode & SPI_RX_DUAL)
      tr.rx_nbits = 2;
   if (!(mode & SPI_LOOP)) {
      if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
         tr.rx_buf = 0;
      else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
         tr.tx_buf = 0;
   }

   ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
   if (ret < 1)
      pabort("can't send spi message");

   if (verbose)
      hex_dump(tx, len, 32, "TX");
   hex_dump(rx, len, 32, "RX");
}

int main(int argc, char *argv[])
{
   int ret = 0;
   int fd;
   //uint16_t *tx;
   //uint16_t *rx;
   //int size;

   if(argc != 3){
       printf("Usage: %s <default> <parameters>\n", argv[0]);
       printf("e.g. %s <min / down half / mid / up half / max / 40MHz (1-6)> <N/A>\n", argv[0]);
       printf("e.g. %s <0> <hhhhhh>\n", argv[0]);
       return -1;
   }

   fd = open(device, O_RDWR);
   if (fd < 0)
      pabort("can't open device");

   /*
    * spi mode
    */
   ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
   if (ret == -1)
      pabort("can't set spi mode");

   ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
   if (ret == -1)
      pabort("can't get spi mode");

   /*
    * bits per word
    */
   ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
   if (ret == -1)
      pabort("can't set bits per word");

   ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
   if (ret == -1)
      pabort("can't get bits per word");

   /*
    * max speed hz
    */
   ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
   if (ret == -1)
      pabort("can't set max speed hz");

   ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
   if (ret == -1)
      pabort("can't get max speed hz");

   printf("spi mode: 0x%x\n", mode);
   printf("bits per word: %d\n", bits);
   printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

   //transfer(fd, default_tx, default_rx, sizeof(default_tx));

   uint16_t min[2]         = {0x3000, 0xB000}; // 39.999 559 3XX MHz
   uint16_t down_half[2]   = {0x33FF, 0xBFFF}; // 39.999 776 0XX MHz
   uint16_t mid[2]         = {0x37FF, 0xBFFF}; // 39.999 995 0XX MHz
   uint16_t m40[2]         = {0x3816, 0xB000}; // 40.000 000 0XX MHz
   uint16_t up_half[2]     = {0x3BFF, 0xBFFF}; // 40.000 213 5XX MHz
   uint16_t max[2]         = {0x3FFF, 0xBFFF}; // 40.000 425 5XX MHz
   uint32_t input          = strtol(argv[2], NULL, 16);
   uint16_t custom[2]      = {0x37FF, 0xBFFF};
   custom[0]               = (0x3000 + ((input>>12)&0xFFF)) & 0xFFFF; // DAC A
   custom[1]               = (0xB000 + (input&0xFFF)) & 0xFFFF;    // DAC B
   printf("A %04x B %04x\n", custom[0], custom[1]);
   uint16_t* msg;

   switch(atoi(argv[1]))
   {
      case 0:
         msg = custom;
         break;
      case 1: 
         msg = min; 
         break;
      case 2: 
         msg = down_half; 
         break;
      case 3: 
         msg = mid; 
         break;
      case 4: 
         msg = up_half;
         break;
      case 5: 
         msg = max; 
         break;
      case 6: 
         msg = m40; 
         break;
      default:
         msg = mid;
   }

   transfer(fd, msg, default_rx, sizeof(msg));

   close(fd);

   return ret;

 
}
