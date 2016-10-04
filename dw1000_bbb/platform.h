/*
 * platform.h
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

#include "deca_types.h"
#include "deca_device_api.h"
#include <stdint.h>
#include <fcntl.h>

#define DECA_MAX_SPI_HEADER_LENGTH      (3)                     // max number of bytes in header (for formating & sizing)

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn hardware_init()
 *
 * @brief Initialise all peripherals at once.
 *
 * @param none
 *
 * @return none
 */
int hardware_init();

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn reset_DW1000()
 *
 * @brief Hardware reset DW1000.
 *
 * @param none
 *
 * @return none
 */
int reset_DW1000();

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_low()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
int spi_set_rate_low();

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_high()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */
int spi_set_rate_high();

void sleep_ms(unsigned int time_ms);
